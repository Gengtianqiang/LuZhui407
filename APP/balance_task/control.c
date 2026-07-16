#include "control.h"
#include "imu_output.h"
#include "Vofa.h"
#include "jdy_driver.h"
#include "mesh_mode.h"
#ifndef MESH_KEY_MODE
#define MESH_KEY_MODE 5U
#endif

#ifndef MESH_KEY_START
#define MESH_KEY_START 7U
#endif

#ifndef MESH_KEY_L1
#define MESH_KEY_L1 12U
#endif

#ifndef MESH_FUNC_FIND_NODE
#define MESH_FUNC_FIND_NODE ((1U << 6) | MESH_KEY_START)
#endif

#ifndef MESH_FUNC_RREE_CONTROL
#define MESH_FUNC_RREE_CONTROL ((1U << 5) | MESH_KEY_MODE)
#endif





extern mesh_datasend_pkt_t my_mesh_send_pkt;

float RC_Velocity = RC_Velocity_DISU;
PID_Para_t PID_Control_Motor_A = {
	.Kp = 0.38 * 90,
	.Ki = 0.1 * 10,
	.Kd = 0.01,

	.Target = 50,
	.Error0 = 0,
	.Error1 = 0,
	.ErrorInt = 0,
};

PID_Para_t PID_Control_Motor_B = {
	.Kp = 0.38 * 90,
	.Ki = 0.1 * 10,
	.Kd = 0.01,

	.Target = 50,
	.Error0 = 0,
	.Error1 = 0,
	.ErrorInt = 0,
};

PID_Para_t PID_Control_Motor_C = {
	.Kp = 95,
	.Ki = 8,
	.Kd = 0.00,

	.Target = 0,
	.Error0 = 0,
	.Error1 = 0,
	.ErrorInt = 0,
};

PID_Para_t PID_Control_Motor_D = {
	.Kp = 95,
	.Ki = 8,
	.Kd = 0.00,

	.Target = 63,
	.Error0 = 0,
	.Error1 = 0,
	.ErrorInt = 0,
};

typedef struct {
    float Aoa_angle_init;  // 初始AOA角度
    _Bool Aoa_ready;       // AOA数据就绪标志
} PDOA_TYPE_t;

PDOA_TYPE_t pdoa_type;

struct {
    uint32_t lst_time;     // 上次时间戳
    uint8_t delay_flag;    // 延时标志
    float v;               // 速度值
    float act_v;           // 实际速度
    float last_acc;        // 上次加速度
} integral_x = {0};

float linear_acc = 0.0f;   // 线性加速度
float linear_acc_lst = 0.0f; // 上次线性加速度
float linear_v = 0.0f;     // 线性速度

// 小车三轴目标移动速度，单位：m/s
float Move_X = 0.0f, Move_Z = 0.0f;

// 是否允许进行自动回充
uint8_t Allow_Recharge = 0;

// 平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control = {0};

// 电机的参数结构体
Motor_parameter MOTOR_A = {0}, MOTOR_B = {0}, MOTOR_C = {0}, MOTOR_D = {0};

// 初始化机器人参数结构体
Robot_Parament_InitTypeDef Robot_Parament = {0};
float Encoder_precision;    // 编码器精度
float Wheel_perimeter;      // 轮子周长，单位：m
float Wheel_spacing;        // 驱动轮轮距，单位：m
float Wheel_axlespacing;    // 小车前后轴的轴距，单位：m
float Omni_turn_radiaus;    // 全向轮转弯半径，单位：m

// 编码器倍频数，取决于编码器初始化设置
#define EncoderMultiples 4
// 编码器数据读取频率
#define CONTROL_FREQUENCY 100
// 速度控制PID参数
float Velocity_KP = 95.0f, Velocity_KI = 8.0f;

/************ 与车型相关的变量 ************/

/* 11/12 更新 */
extern Motor_t Motor_Instance;

#ifndef PI
#define PI 3.1415926f
#endif

#define SAMPLE_PERIOD 0.017857f // 1/56s

// 梯形积分法计算
float update_velocity(float acc_x)
{
    if (integral_x.delay_flag == 0)
    {
        integral_x.delay_flag++;
        integral_x.lst_time = HAL_GetTick();
    }
    else if (integral_x.delay_flag == 1)
    {
        // 一分钟后
        if (HAL_GetTick() - integral_x.lst_time >= (17u * 60u))
        {
            integral_x.v = 0.0f;
            integral_x.delay_flag = 2;
        }
    }
    if (integral_x.delay_flag == 2)
    {
        // integral_x.v += (acc_x + integral_x.last_acc) * 0.5f * SAMPLE_PERIOD;  // 梯形积分计算
        // integral_x.last_acc = acc_x;  // 更新上一次加速度
        integral_x.v += acc_x * SAMPLE_PERIOD; // 简单积分处理
    }
    return integral_x.v;
}

void Car_init(void)
{
    // Car_Mode = 5;
    #define TOP_4WD_BS_wheelspacing 0.311f    // 半轮距
    #define TOP_4WD_BS_axlespacing 0.308f     // 半轴距
    #define MD60N_47 47                       // 电机减速比
    #define Photoelectric_500 500             // 光电编码器线数
    #define _4WD_225 0.225f                   // 四轮驱动轮直径
    // TOP_4WD_BS - 适配4轮毂式麦克纳姆轮车型
    Robot_Init(TOP_4WD_BS_wheelspacing, TOP_4WD_BS_axlespacing, MD60N_47, Photoelectric_500, _4WD_225);
}

static void BLE_SendMeshAck(uint16_t to_maddr, uint8_t ack_key)
{
    if (to_maddr == 0U ||
        jdy_handle.p_mesh_submode == NULL ||
        jdy_handle.p_mesh_submode->p_parser == NULL)
    {
        return;
    }

    my_mesh_send_pkt.to_maddr = to_maddr;
    my_mesh_send_pkt.key = ack_key;
    my_mesh_send_pkt.L = 0x7F;
    my_mesh_send_pkt.R = 0x7F;
    my_mesh_send_pkt.valid = 1;

    jdy_handle.p_mesh_submode->p_parser->pf_mesh_datasend_handler(&jdy_handle, &my_mesh_send_pkt);
}

static bool BLE_TryReadFromJDY_Geng(uint8_t *L, uint8_t *R, uint8_t *key)
{
    static uint8_t ble_locked = 0;
    static uint16_t master_maddr = 0;
    static uint8_t find_node_echo_left = 0;
    static uint32_t find_node_echo_tick = 0;

    if (L == NULL || R == NULL || key == NULL)
    {
        return false;
    }

    if (jdy_handle.init_status != JDY_INIT ||
        jdy_handle.p_mesh_submode == NULL ||
        jdy_handle.p_mesh_submode->p_parser == NULL ||
        jdy_handle.p_mesh_submode->mash_init_flag != JDY_INIT)
    {
        return false;
    }

    if (find_node_echo_left > 0U && (HAL_GetTick() - find_node_echo_tick) >= 50U)
    {
        BLE_SendMeshAck(master_maddr, MESH_FUNC_FIND_NODE);
        find_node_echo_left--;
        find_node_echo_tick = HAL_GetTick();
    }

    if (jdy_handle.p_mesh_submode->p_parser->pf_mesh_datarecv_handler(&jdy_handle, &ringBuffer4) != JDY_OK)
    {
        return false;
    }

    mesh_datarecv_pkt_t *pkt = &jdy_handle.p_mesh_submode->p_parser->recv_pkt;

    if (pkt->header != 0xF1DD || pkt->end != 0x0D0A)
    {
        return false;
    }

    if (pkt->key == MESH_KEY_START)
    {
        master_maddr = pkt->from_maddr;
        ble_locked = 1;
        find_node_echo_left = 10;
        find_node_echo_tick = 0;

        BLE_SendMeshAck(master_maddr, MESH_FUNC_FIND_NODE);
        return false;
    }

    if (pkt->key == MESH_KEY_MODE)
    {
        if (ble_locked && pkt->from_maddr == master_maddr)
        {
            BLE_SendMeshAck(master_maddr, MESH_FUNC_RREE_CONTROL);
        }

        ble_locked = 0;
        master_maddr = 0;
        find_node_echo_left = 0;
        return false;
    }

    if (!ble_locked || pkt->from_maddr != master_maddr)
    {
        return false;
    }

    *L = pkt->L;
    *R = pkt->R;
    *key = pkt->key;
    return true;
}

void BLE_control(void)
{
    static bool isCarInited = false;
    if (isCarInited == false)
    {
        Car_init();
        isCarInited = true;
    }

    uint8_t ble_L = EMA_TARGET;
    uint8_t ble_R = EMA_TARGET;
    uint8_t ble_key = KEY_TARGET;
    
    if (BLE_TryReadFromJDY_Geng(&ble_L, &ble_R, &ble_key) == true)
    {
        ema_set_new_data(&emaL, ble_L);
        ema_set_new_data(&emaR, ble_R);
        key_set_new_data(&emaKey, ble_key);
   
    }

    // 做PS(50Hz)滤波同步，因为控制周期是10ms(100Hz)
    int LX = ema_filter(&emaL) - EMA_TARGET;
    int RY = ema_filter(&emaR) - EMA_TARGET;
    uint8_t BLEKey = key_filter(&emaKey);

    // 方向控制
    RY = -RY;

    // 阈值过滤小幅度动作
    int Threshold = 20;
    if (LX > -Threshold && LX < Threshold)
        LX = 0;
    if (RY > -Threshold && RY < Threshold)
        RY = 0;
    if (LX == 0)
        Move_X = Move_X / 1.2f;
    if (RY == 0)
        Move_Z = Move_Z / 1.2f;

    // 高速档
    if (BLEKey == MESH_KEY_L1)
        RC_Velocity = RC_Velocity_GAOSU;
    else
        RC_Velocity = RC_Velocity_DISU;

    // 限幅
    if (RC_Velocity < 0)
        RC_Velocity = 0;

    // 对PS2手柄控制指令进行处理
    Move_X = LX;
    Move_Z = RY;
    Move_X = Move_X * RC_Velocity / 128;
    // 2024.5.22日优化，改成两个档位控制，就不需要处理 Move_Z 了，简化逻辑

    // 左侧摇杆按下的情况下，不识别右侧的按键，强制原地转圈（四轮原地旋转轮子震动大）
    if (LX == 0)
    {
        RY = 0;
        Move_Z = 0;
    }

    // 进行两个档位的控制选择。
    // 高速档和低速档控制效果不同
    if (RY > 0)
    {
        // 高速档
        if (RC_Velocity == RC_Velocity_GAOSU)
        {
            // 高速档情况下，方向效果过度明显
            // 转角半幅度大
            Move_Z = 0.5;
        }
        // 低速档
        else if (RC_Velocity == RC_Velocity_DISU)
        {
            // 低速档情况下，方向效果明显
            Move_Z = 0.8;
        }
        else
        {
            Move_Z = 0;
        }
    }
    else if (RY < 0)
    {
        // 高速档
        if (RC_Velocity == RC_Velocity_GAOSU)
        {
            // 高速档情况下，方向效果过度明显
            // 转角半幅度大
            Move_Z = -0.5;
        }
        // 低速档
        else if (RC_Velocity == RC_Velocity_DISU)
        {
            // 低速档情况下，方向效果明显
            Move_Z = -0.8;
        }
        else
        {
            Move_Z = 0;
        }
    }
    else
    {
        Move_Z = 0;
    }

    // Z轴数据方向转换
    if (Move_X < 0)
        Move_Z = -Move_Z;

    // 单位转换，mm/s -> m/s
    Move_X = Move_X / 1000;

    // 得到控制目标值，进行运动学分解
    // 获取 MOTOR_A.Target、 MOTOR_B.Target、 MOTOR_C.Target、 MOTOR_D.Target
    Drive_Motor(Move_X, 0.0f, Move_Z);

    // 如果电池电压存在异常
    if (Volt < 12.0f)
    {
        Set_Motor_PWM(Motor_A, 0);
        Set_Motor_PWM(Motor_B, 0);
        Set_Motor_PWM(Motor_C, 0);
        Set_Motor_PWM(Motor_D, 0);
        return;
    }

    // 如果超声波检测有问题？

    // 开环控制  计算各电机PWM值，PWM代表轮组实际转速
    MOTOR_A.Motor_Pwm = Incremental_no_PI('A', MOTOR_A.Target);
    MOTOR_B.Motor_Pwm = Incremental_no_PI('B', MOTOR_B.Target);
    MOTOR_C.Motor_Pwm = Incremental_no_PI('C', MOTOR_C.Target);
    MOTOR_D.Motor_Pwm = Incremental_no_PI('D', MOTOR_D.Target);

    // 根据不同小车型号设置不同的PWM控制极性
    MOTOR_A.Motor_Pwm = -MOTOR_A.Motor_Pwm;
    MOTOR_B.Motor_Pwm = -MOTOR_B.Motor_Pwm;
    MOTOR_C.Motor_Pwm = -MOTOR_C.Motor_Pwm;
    MOTOR_D.Motor_Pwm = -MOTOR_D.Motor_Pwm;

    // 将目标速度值转换为PWM值
    Set_Motor_PWM(Motor_A, MOTOR_A.Motor_Pwm);
    Set_Motor_PWM(Motor_B, MOTOR_B.Motor_Pwm);
    Set_Motor_PWM(Motor_C, MOTOR_C.Motor_Pwm);
    Set_Motor_PWM(Motor_D, MOTOR_D.Motor_Pwm);
}

/*25/11/12更新*/
void BLE_PID_control(void)
{
    static bool isCarInited = false;
    if (isCarInited == false)
    {
        Car_init();
        isCarInited = true;
    }

    if (JDY_isDataReady(&myJDY) == true)
    {
        BleData ble = JDY_GetData(&myJDY);
        ema_set_new_data(&emaL, ble.L);
        ema_set_new_data(&emaR, ble.R);
        key_set_new_data(&emaKey, ble.key);
    }

    // 做PS(50Hz)滤波同步，因为控制周期是10ms(100Hz)
    int LX = ema_filter(&emaL) - EMA_TARGET;
    int RY = ema_filter(&emaR) - EMA_TARGET;
    uint8_t BLEKey = key_filter(&emaKey);

    // 方向控制
    RY = -RY;

    // 阈值过滤小幅度动作
    int Threshold = 20;
    if (LX > -Threshold && LX < Threshold)
        LX = 0;
    if (RY > -Threshold && RY < Threshold)
        RY = 0;
    if (LX == 0)
        Move_X = Move_X / 1.2f;
    if (RY == 0)
        Move_Z = Move_Z / 1.2f;

    // 高速档
    if (BLEKey == MESH_KEY_L1)
        RC_Velocity = RC_Velocity_GAOSU;
    else
        RC_Velocity = RC_Velocity_DISU;

    // 限幅
    if (RC_Velocity < 0)
        RC_Velocity = 0;

    // 对PS2手柄控制指令进行处理
    Move_X = LX;
    Move_Z = RY;
    Move_X = Move_X * RC_Velocity / 128;

    // 左侧摇杆按下的情况下，不识别右侧的按键，强制原地转圈（四轮原地旋转轮子震动大）
    if (LX == 0)
    {
        RY = 0;
        Move_Z = 0;
    }

    // 进行两个档位的控制选择。
    // 高速档和低速档控制效果不同
    if (RY > 0)
    {
        // 高速档
        if (RC_Velocity == RC_Velocity_GAOSU)
        {
            // 高速档情况下，方向效果过度明显
            // 转角半幅度大
            Move_Z = 0.5;
        }
        // 低速档
        else if (RC_Velocity == RC_Velocity_DISU)
        {
            // 低速档情况下，方向效果明显
            Move_Z = 1.1;
        }
        else
        {
            Move_Z = 0;
        }
    }
    else if (RY < 0)
    {
        // 高速档
        if (RC_Velocity == RC_Velocity_GAOSU)
        {
            // 高速档情况下，方向效果过度明显
            // 转角半幅度大
            Move_Z = -0.5;
        }
        // 低速档
        else if (RC_Velocity == RC_Velocity_DISU)
        {
            // 低速档情况下，方向效果明显
            Move_Z = -1.1;
        }
        else
        {
            Move_Z = 0;
        }
    }
    else
    {
        Move_Z = 0;
    }

    // Z轴数据方向转换
    if (Move_X < 0)
        Move_Z = -Move_Z;

    // 单位转换，mm/s -> m/s
    Move_X = Move_X / 1000;

    // 得到控制目标值，进行运动学分解
    // 获取 MOTOR_A.Target、 MOTOR_B.Target、 MOTOR_C.Target、 MOTOR_D.Target
    Drive_Motor(Move_X, 0.0f, Move_Z);
    
    // 如果电池电压存在异常，则设置目标速度为0
    if (LX == 0x7F || RY == 0x7F)
    {
        MOTOR_C.Target = 0;
    }
    
    C_Encoder = -Encoder_Get_MotorC();
    MOTOR_C.Motor_Pwm = Incremental_PI_C(C_Encoder, MOTOR_C.Target * 100);
    //    MOTOR_C.Motor_Pwm = -MOTOR_C.Motor_Pwm;

    Set_Motor_PWM(Motor_C, MOTOR_C.Motor_Pwm);
}

// 开启pdoa跟随
float BLE_follow_control_test = 0.02;
void BLE_follow_control(void)
{
    static bool isCarInited = false;
    if (isCarInited == false)
    {
        Car_init();
        isCarInited = true;
    }
		if(proto_data.distance_cm>50) {
			if (abs(proto_data.distance_cm - FOLLOW_DISTANCE) > FOLLOW_DISTANCE_ERROR && proto_data.distance_cm!=0)
			{
					// 比例控制
					Move_X = (float)((proto_data.distance_cm - FOLLOW_DISTANCE)*BLE_follow_control_test); 
			}
			else if(abs(proto_data.distance_cm - FOLLOW_DISTANCE) < FOLLOW_DISTANCE_ERROR && proto_data.distance_cm!=0) 
			{
					Move_X = 0;
			}
		} else Move_X = 0;
		
//		if(proto_data.PdoaisAvailable==false) {
//				Move_X = 0;
//		}
		
    if(pdoa_type.Aoa_ready==true && 0 != proto_data.aoa_deg) 
    {
        pdoa_type.Aoa_ready = true;
        pdoa_type.Aoa_angle_init = proto_data.aoa_deg;
    }
    
    if (fabs(proto_data.aoa_deg - pdoa_type.Aoa_angle_init) > FOLLOW_ANGLE_ERROR && 0 != proto_data.aoa_deg &&
        abs(proto_data.distance_cm - FOLLOW_DISTANCE) > FOLLOW_DISTANCE_ERROR)
    {

            float angle_error =0;

			angle_error = proto_data.aoa_deg - pdoa_type.Aoa_angle_init;

			/*执行转向*/
				float pid_output = pid_update(&heading_pid, angle_error, 1.0);
				if (fabs(pid_output) < 1.0f)
				{
					pid_output = 0;
				}
					differential_drive_control(pid_output, 0);
			
        // if(proto_data.aoa_deg - pdoa_type.Aoa_angle_init>0)
        //     // 右转
        //     Move_Z = 0.5f;
        // else if(proto_data.aoa_deg - pdoa_type.Aoa_angle_init<0)
        //     // 左转
        //     Move_Z = -0.5f;
    }
    
    if (fabs(proto_data.aoa_deg - pdoa_type.Aoa_angle_init) < FOLLOW_ANGLE_ERROR && 0 != proto_data.aoa_deg||proto_data.PdoaisAvailable==false)
    {
        // 不转向
        Move_Z = 0.0f;
    }
    
    // x轴数据限幅
    if (fabs(Move_X) > 1) 
    {
        if(Move_X>0) Move_X = 1;
        if(Move_X<0) Move_X = -1;
    }
    
    // 得到控制目标值，进行运动学分解
    Drive_Motor(Move_X, 0.0f, Move_Z);

    // 如果电池电压存在异常
    if (Volt < 12.0f)
    {
        Set_Motor_PWM(Motor_A, 0);
        Set_Motor_PWM(Motor_B, 0);
        Set_Motor_PWM(Motor_C, 0);
        Set_Motor_PWM(Motor_D, 0);
        Buzzer_Start_Once(100);
        return;
    }

    // 如果超声波检测有问题？

    // 开环控制  计算各电机PWM值，PWM代表轮组实际转速
    MOTOR_A.Motor_Pwm = Incremental_no_PI('A', MOTOR_A.Target);
    MOTOR_B.Motor_Pwm = Incremental_no_PI('B', MOTOR_B.Target);
    MOTOR_C.Motor_Pwm = Incremental_no_PI('C', MOTOR_C.Target);
    MOTOR_D.Motor_Pwm = Incremental_no_PI('D', MOTOR_D.Target);

    // 根据不同小车型号设置不同的PWM控制极性
    MOTOR_A.Motor_Pwm = -MOTOR_A.Motor_Pwm;
    MOTOR_B.Motor_Pwm = -MOTOR_B.Motor_Pwm;
    MOTOR_C.Motor_Pwm = -MOTOR_C.Motor_Pwm;
    MOTOR_D.Motor_Pwm = -MOTOR_D.Motor_Pwm;

    // 将目标速度值转换为PWM值
    Set_Motor_PWM(Motor_A, MOTOR_A.Motor_Pwm);
    Set_Motor_PWM(Motor_B, MOTOR_B.Motor_Pwm);
    Set_Motor_PWM(Motor_C, MOTOR_C.Motor_Pwm);
    Set_Motor_PWM(Motor_D, MOTOR_D.Motor_Pwm);
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：aw，aw_init
返回  值：方向速度
50ms计算一次
**************************************************************************/
static float error_away1 = 0;
float My_PID_WEIZHI_L(float Qiwang, float Shiji, float kP, float kI, float kD)
{
    float error_L;          // 偏差值
    float JIFEN_WEIZHI_L;   // 积分项
    float WEIFEN_DL;        // 微分项
    float BILI_DL;          // 比例项
    
    error_L = Qiwang - Shiji;
    BILI_DL = error_L * kP;                   // 计算比例项
    JIFEN_WEIZHI_L += error_L * kI;           // 计算积分项
    WEIFEN_DL = (error_L - error_away1) * kD; // 计算微分项
    error_away1 = error_L;
    
    float ZUIZHONGJIEGUO = (BILI_DL + JIFEN_WEIZHI_L + WEIFEN_DL);
    return ZUIZHONGJIEGUO;
}

/**************************************************************************
函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表当前偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)
////// Target 范围是 -3.5~3.5
**************************************************************************/
int32_t Incremental_PI_Left(float Encoder, float Target)
{
    static float Bias, Last_bias;
    static float Pwm;
    
    Encoder = Encoder / 200.0f; // 实测 IMU 中 Move_X(Target) 对应关系是200倍
    Bias = Target - Encoder;    // 计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    
    if (Pwm > 33600)
        Pwm = 33600;
    if (Pwm < -33600)
        Pwm = -33600;
    
    Last_bias = Bias; // 保存上一次偏差
    return (int32_t)Pwm;
}

int32_t Incremental_PI_Right(float Encoder, float Target)
{
    static float Bias, Last_bias;
    static float Pwm;
    
    Encoder = Encoder / 200.0f; // 实测 IMU 中 Move_X(Target) 对应关系是200倍
    Bias = Target - Encoder;    // 计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    
    if (Pwm > 33600)
        Pwm = 33600;
    if (Pwm < -33600)
        Pwm = -33600;
    
    Last_bias = Bias; // 保存上一次偏差
    return (int32_t)Pwm;
}

/**************************************************************************
函数功能：开环控制，替代PI控制器，用于没有编码器的时候
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
**************************************************************************/
int32_t Incremental_no_PI(char ABCD, float Target)
{
    // static float last_Target=0, bias=0, maxbias=3.5;
    static float amplitude = 3.5; // 轮子目标速度限制
    // Target 的值在 -amplitude ~ +amplitude 之间，是在 Drive_Motor() 中定义的
    
    static double Pwm;
    // 做一个限幅滤波处理，防止 Target 变化得太陡
    // bias=Target-last_Target;
    // if(bias<-maxbias || bias>maxbias) {
    //     if(bias<0) Target=last_Target-maxbias;
    //     if(bias>0) Target=last_Target+maxbias;
    // }
    // last_Target=Target;

    // Pwm 在 -16800~+16800 之间，和 Target 在 -amplitude ~ +amplitude 之间对应
    // 算出来比例就是 16800 / 3.5 = 4800
    // Pwm = 4800*Target;
    // if (Pwm > 16800)Pwm = 16800;
    // if (Pwm < -16800)Pwm = -16800;
    // amplitude=amplitude;

    // 16800 替换成 33600
    // 算出来比例就是 33600 / 3.5 = 9600
    Pwm = 9600 * Target;
    if (Pwm > 33600)
        Pwm = 33600;
    if (Pwm < -33600)
        Pwm = -33600;
    amplitude = amplitude;

    // 清除PWM震荡位，该位为1时代表需要清除 PWM  //2025.3.22 移除，因为不需要清除PWM
    //  if (start_clear)  ……
    (void)ABCD;

    return (int32_t)Pwm;
}

/**************************************************************************
函数功能：运动学逆解，根据三轴目标速度计算各轮子的目标速度
入口参数：X和Y，Z轴方向的目标移动速度
返回  值：无
**************************************************************************/
uint32_t time_count = 0;
void Drive_Motor(float Vx, float Vy, float Vz)
{
    static float amplitude = 3.5; // 轮子目标速度限制

    Vx = target_limit_float(Vx, -amplitude, amplitude);
    Vy = target_limit_float(Vy, -amplitude, amplitude);
    Vz = target_limit_float(Vz, -amplitude, amplitude);

    if (Allow_Recharge == 0)
        Smooth_control(Vx, Vy, Vz); // 对输入速度进行平滑处理
    else
        smooth_control.VX = Vx,
        smooth_control.VY = Vy,
        smooth_control.VZ = Vz;

    // // 获取平滑处理后的数据
   Vx = smooth_control.VX;
   Vy = smooth_control.VY;
   Vz = smooth_control.VZ;

    // time_count++;
    // if (time_count >= 100)
    // {
    //     time_count = 0;
        
    //     JDY_DEBUG_OUT("Vx:%.2f, Vy:%.2f, Vz:%.2f\r\n", smooth_control.VX, smooth_control.VY, smooth_control.VZ);
    // }
        
    // 运动学逆解
    MOTOR_A.Target = +0 + Vx - Vz * (Wheel_axlespacing + Wheel_spacing);
    MOTOR_B.Target = -0 + Vx - Vz * (Wheel_axlespacing + Wheel_spacing);
    MOTOR_C.Target = +0 + Vx + Vz * (Wheel_axlespacing + Wheel_spacing);
    MOTOR_D.Target = -0 + Vx + Vz * (Wheel_axlespacing + Wheel_spacing);

    // 轮子(电机)目标速度限制
    MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
    MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
    MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
    MOTOR_D.Target = target_limit_float(MOTOR_D.Target, -amplitude, amplitude);
}

/**************************************************************************
函数功能：限幅函数
入口参数：数值
返回  值：无
**************************************************************************/
float target_limit_float(float insert, float low, float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

int target_limit_int(int insert, int low, int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

/**************************************************************************
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx, float vy, float vz)
{
    float step = 0.04; // 平滑处理步进值

    // x轴刹车处理 (有一点点滤波)
    // 2024.5.23 添加，防止小车停车太猛
    // 平滑处理步进值
    float shache_step = 0.08;
    
    // 当x vy vz 是正数
    if (smooth_control.VX > 0)
    {
        if (vx <= 0)
        {
            smooth_control.VX -= shache_step;
            if (smooth_control.VX < 0)
                smooth_control.VX = 0;
            // y z
            smooth_control.VY = 0;
            smooth_control.VZ = 0;
        }
    }
    
    // 当x vy vz 是负数
    if (smooth_control.VX < 0)
    {
        if (vx >= 0)
        {
            smooth_control.VX += shache_step;
            if (smooth_control.VX > 0)
                smooth_control.VX = 0;
            // y z
            smooth_control.VY = 0;
            smooth_control.VZ = 0;
        }
    }

    // X轴速度平滑
    if (vx > smooth_control.VX)
    {
        smooth_control.VX += step;
        if (smooth_control.VX > vx)
            smooth_control.VX = vx;
    }
    else if (vx < smooth_control.VX)
    {
        smooth_control.VX -= step;
        if (smooth_control.VX < vx)
            smooth_control.VX = vx;
    }
    else
        smooth_control.VX = vx;

    // Y轴速度平滑
    if (vy > smooth_control.VY)
    {
        smooth_control.VY += step;
        if (smooth_control.VY > vy)
            smooth_control.VY = vy;
    }
    else if (vy < smooth_control.VY)
    {
        smooth_control.VY -= step;
        if (smooth_control.VY < vy)
            smooth_control.VY = vy;
    }
    else
        smooth_control.VY = vy;

    // Z轴速度平滑
    if (vz > smooth_control.VZ)
    {
        smooth_control.VZ += step;
        if (smooth_control.VZ > vz)
            smooth_control.VZ = vz;
    }
    else if (vz < smooth_control.VZ)
    {
        smooth_control.VZ -= step;
        if (smooth_control.VZ < vz)
            smooth_control.VZ = vz;
    }
    else
        smooth_control.VZ = vz;

    // 0速时保证稳态稳定
    if (vx == 0 && smooth_control.VX < 0.05f && smooth_control.VX > -0.05f)
        smooth_control.VX = 0;
    if (vy == 0 && smooth_control.VY < 0.05f && smooth_control.VY > -0.05f)
        smooth_control.VY = 0;
    if (vz == 0 && smooth_control.VZ < 0.05f && smooth_control.VZ > -0.05f)
        smooth_control.VZ = 0;
}

/**************************************************************************
函数功能：初始化小车参数
入口参数：轮距 轴距 电机减速比 电机编码器线数 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(float wheelspacing, float axlespacing, int gearratio, int Accuracy, float tyre_diameter)
{
    Robot_Parament.WheelSpacing = wheelspacing;   // 半轮距
    Robot_Parament.AxleSpacing = axlespacing;     // 半轴距
    Robot_Parament.GearRatio = gearratio;         // 电机减速比
    Robot_Parament.EncoderAccuracy = Accuracy;    // 编码器线数
    Robot_Parament.WheelDiameter = tyre_diameter; // 驱动轮直径

    // 电机(轮子)转1圈对应的编码器数值
    Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio;
    // 驱动轮周长
    Wheel_perimeter = Robot_Parament.WheelDiameter * PI;
    // 半轮距
    Wheel_spacing = Robot_Parament.WheelSpacing;
    // 半轴距
    Wheel_axlespacing = Robot_Parament.AxleSpacing;
}
