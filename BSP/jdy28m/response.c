#include "response.h"
#include <string.h>
#include <stdio.h>
#include "buzzer/buzzer.h"

Response myResponse = 
{
    .link = {0},
    .dev = {0},
    .newData = 
    {
        .isUpdata = false,
        .Data = {
           .key = MESH_KEY_NONE,
           .L = 0x7F,
           .R = 0x7F,
        },
    },
};



mesh_datarecv_pkt   default_mesh_pkt = {
    .header = 0xF1DD,
		.netID = 0x00,
    .from_maddr = 0x0000,
    .to_maddr = 0x0000,
    .key = MESH_KEY_NONE,
    .L = 0x7F,
    .R = 0x7F,
    .end = 0x0D0A
};

mesh_msg_t mesh_msg = {

    .is_4g_signal = false,
    .src_node = 0,
    .dst_node = 0,
    .msg_type = MSG_TYPE_START,
    .msg_body = {0},
};



void ResponseMesh_LOOP( Response *myResponse, Parser *myParser, Transmit *myTransmit ){
    // 判断数据是否准备好
    if(MeshData_isReady(myParser) != true){ return; }
    
    // 取数据
    mesh_datarecv_pkt* pkt = Get_MeshData(myParser);
    
    /// 响应按键 （即时处理）
    if(pkt->key == MESH_KEY_START){         // 初次连接 （响应Start按键）
        if(myResponse->link.isConnected == false){
            myResponse->link.isConnected = true;
            myResponse->link.Master_maddr = pkt->from_maddr;
            myResponse->link.netID = pkt->netID;
            // 回应 原地址 (from_addr) 一个 回应
            TransmitData_Updata(myTransmit, pkt->from_maddr, MESH_FUNC_FIND_NODE);
            // 响一下
            Buzzer_Start_Once(100);
        }
        else{   // 再次连接 (手柄休眠后再次连接)
            if(myResponse->link.Master_maddr == pkt->from_maddr){
                // 回应 原地址 (from_addr) 一个 回应
                TransmitData_Updata(myTransmit, pkt->from_maddr, MESH_FUNC_FIND_NODE);
                // 响一下
                Buzzer_Start_Once(100);
            }
        }
    }else if(pkt->key == MESH_KEY_MODE){    // 断开连接 （响应 Mode 按键）
        if(myResponse->link.isConnected == true && myResponse->link.Master_maddr == pkt->from_maddr){
            myResponse->link.isConnected = false;
            myResponse->link.Master_maddr = 0x0000;
            // 回应 原地址 (from_addr) 一个 回应
            TransmitData_Updata(myTransmit, pkt->from_maddr, MESH_FUNC_RREE_CONTROL);
        }
    }

    // 响应摇杆（非即时处理，交给后面程序处理）
    if(myResponse->link.isConnected == true && myResponse->link.Master_maddr == pkt->from_maddr){
        myResponse->newData.isUpdata = true;
        myResponse->newData.Data.key = pkt->key;
        myResponse->newData.Data.L = pkt->L;
        myResponse->newData.Data.R = pkt->R;
    }
}

void ResponseAT_LOOP( Response *myResponse, Parser *myParser ){
    // 判断数据是否准备好
    if(ATData_isReady(myParser)!= true){ return; }

    // 取数据
    char* buf = (char*)Get_ATData(myParser);
    uint16_t len = Get_ATData_len(myParser);

    /// AT指令之后 处理返回的值
    if(      strncmp(buf, "+MADDR=", 7) == 0 ){
        memcpy(myResponse->dev.MADDR, buf+7, sizeof(myResponse->dev.MADDR));  // 拷贝数据 
        sscanf((char*)myResponse->dev.MADDR, "%4hx", &myResponse->link.Slave_maddr); // 将字符数组转换成 16 进制数
    }
    else if( strncmp(buf, "+MAC=", 5) == 0 ){
        memcpy(myResponse->dev.MAC, buf+5, sizeof(myResponse->dev.MAC));  // 拷贝数据 
    }
    else if( strncmp(buf, "+STAT=", 6) == 0 ){
        memcpy(&myResponse->dev.STAT, buf+6, sizeof(myResponse->dev.STAT));  // 拷贝数据 
    }
    else if( strncmp(buf, "+BAUD=", 6) == 0 ){
        memcpy(&myResponse->dev.BAUD, buf+6, sizeof(myResponse->dev.BAUD));  // 拷贝数据 
    }
    else if( strncmp(buf, "+ROLE=", 6) == 0 ){
        memcpy(&myResponse->dev.ROLE, buf+6, sizeof(myResponse->dev.ROLE));  // 拷贝数据 
    }
    else if( strncmp(buf, "+NETID=", 7) == 0 ){
        memcpy(myResponse->dev.NETID, buf+7, sizeof(myResponse->dev.NETID));  // 拷贝数据 
    }
    else if( strncmp(buf, "+ENLOG=", 7) == 0 ){
        memcpy(&myResponse->dev.ENLOG, buf+7, sizeof(myResponse->dev.ENLOG));  // 拷贝数据 
    }
}

void BLE_Node1_APP_Loop(mesh_msg_t* msg) {
// 	  static uint32_t last_heartbeat_check = 0;
//    static bool is_started = false;
//    static uint8_t feedback_count = 0;
// 		// 1. 接到4G信号
// 		if (msg->is_4g_signal) {
//            if (!is_started) {
//                // 广播启动信号给节点2-5
// //                mesh_broadcast(MSG_TYPE_START, NULL, 0);
               
//                feedback_count = 0;
//             //    Vofa_Printf("[Node1] 正在广播启动信号\n");
//                if(feedback_count==4) {
//                     is_started = true;  
//                     Vofa_Printf("[Node1] 所有节点已启动，开始接收数据\n");
//                }
//            }
//        }
// 		 // 2. 接收处理：监听Mesh网络中的消息
      
//            switch (msg->msg_type) {
//                case MSG_TYPE_FEEDBACK:
//                    // 子节点反馈启动成功
//                    feedback_count++;
//                    Vofa_Printf("[Node1] 收到节点%d的启动反馈，已收到%d个反馈\n", msg->src_node, feedback_count);
//                    break;
//                case MSG_TYPE_HEARTBEAT:
//                    // 处理子节点心跳（电量、是否倒下）
//                    Vofa_Printf("[Node1] 收到节点%d心跳：电量=%d%%，是否倒下=%d\n", 
//                           msg->src_node, msg->msg_body.heartbeat.battery, msg->msg_body.heartbeat.is_fallen);
//                    break;
//                case MSG_TYPE_PODA:
//                    // 接收节点2的poda数据（最终汇聚到1）
//                    Vofa_Printf("[Node1] 收到节点%d的poda数据：%s\n", msg->src_node, msg->msg_body.pdoa_packed_3bytes);
//                    // 可选：上传到4G服务器
//                    // 4G_upload_poda_data(msg.data.poda);
//                    break;
//                default:
//                    break;
//            }

//        // 3. 周期任务：检查子节点在线状态（基于心跳超时）
//     //    if (HAL_GetTick() - last_heartbeat_check > 5000) { // 5秒检查一次
//     //     //    check_node_online_status();
//     //        last_heartbeat_check = HAL_GetTick();
//     //    }
	
}

void BLE_NodeX_APP_Loop(mesh_msg_t* msg) {
//    static uint32_t last_heartbeat_send = 0;
//    static uint32_t last_poda_send = 0;
//    static bool is_started = false;

//        // 1. 事件驱动：监听中心节点的广播消息

//            if (msg->msg_type == MSG_TYPE_START && msg->src_node == 1) {
//                // 收到中心节点的启动信号
//                if (!is_started) {
                   
//                    // 发送启动反馈给节点1,直到开始启动
//                 //    mesh_send(1, MSG_TYPE_FEEDBACK, NULL, 0);
//                    Vofa_Printf("[Node%d] 收到启动信号，已反馈给节点1\n", msg->dst_node);

//                    if(1) {
//                         is_started = true;
//                         Vofa_Printf("[Node%d] 已启动，开始发送数据\n", msg->dst_node);
//                    }
//                }
//            }
//         //    else if (msg->msg_type == MSG_TYPE_PODA) {
//         //        // 按层级转发poda数据（如节点3收到节点2的poda，转发给节点1；节点4收到3的，转发给2，以此类推）
//         //        if (msg->dst_node == 3 && msg->src_node == 2) {
//         //            mesh_send(1, MSG_TYPE_PODA, msg->msg_body.poda, sizeof(msg->msg_body.poda));
//         //        }
//         //        else if (msg->dst_node == 4 && msg->src_node == 3) {
//         //            mesh_send(2, MSG_TYPE_PODA, msg->msg_body.poda, sizeof(msg->msg_body.poda));
//         //        }
//         //        else if (msg->dst_node == 5 && msg->src_node == 4) {
//         //            mesh_send(3, MSG_TYPE_PODA, msg->msg_body.poda, sizeof(msg->msg_body.poda));
//         //        }
//         //    }

//     //     if(msg->timecounter>100) {
//     //    // 2. 周期任务1：发送心跳包（1秒一次）
//     //    if (is_started && HAL_GetTick() - last_heartbeat_send > 1000) {
//     //        heartbeat_data_t hb = {
//     //            .battery = get_battery_level(),
//     //            .is_fallen = check_is_fallen()
//     //        };
//     //        mesh_send(1, MSG_TYPE_HEARTBEAT, &hb, sizeof(hb));
//     //        last_heartbeat_send = HAL_GetTick();
//     //    }

//     //    // 3. 周期任务2：发送poda数据（100ms一次，按层级发送）
//     //    if (is_started && HAL_GetTick() - last_poda_send > 100) {
//     //        poda_data_t poda = get_poda_data();
//     //        if (node_id == 2) {
//     //            mesh_send(1, MSG_TYPE_PODA, &poda, sizeof(poda));
//     //        }
//     //        else if (node_id == 3) {
//     //            mesh_send(2, MSG_TYPE_PODA, &poda, sizeof(poda));
//     //        }
//     //        else if (node_id == 4) {
//     //            mesh_send(3, MSG_TYPE_PODA, &poda, sizeof(poda));
//     //        }
//     //        else if (node_id == 5) {
//     //            mesh_send(4, MSG_TYPE_PODA, &poda, sizeof(poda));
//     //        }
//     //        last_poda_send = HAL_GetTick();
//     //    }


   
}

bool Response_isNewData(Response *myResponse){
    return myResponse->newData.isUpdata;
}

BleData Response_GetNewData(Response *myResponse){
    myResponse->newData.isUpdata = false;
    return myResponse->newData.Data;
}
