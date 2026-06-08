# 4G DTU 通信协议 V2

## 一、设计原则

- **所有指令由上位机（Server）下发，小车（Car）应答**
- 下行帧简洁：帧头 + 功能位(1B) + CRC + 帧尾
- 应答数据：电量/俯仰角/目标点 → 4 字节，其余 → 1 字节

## 二、帧格式

```
+--------+--------+------+------+------+-----------+-----------+--------+
| Header | Header | Cmd  | Dir  | Seq  | Data      | CRC16     | Footer |
| 0xF1   | 0xDD   | 1B   | 1B   | 1B   | 0~4 B     | 2B        | 0x0D0A |
+--------+--------+------+------+------+-----------+-----------+--------+
```

| 偏移 | 字段 | 字节 | 说明 |
|------|------|------|------|
| 0 | Header | 2 | `0xF1 0xDD` |
| 2 | Cmd | 1 | 功能码 |
| 3 | Dir | 1 | `0x01`=下行(Server→Car)，`0x02`=上行应答(Car→Server) |
| 4 | Seq | 1 | 序列号（每发一帧自增，0 则重发） |
| 5 | Data | N | 数据载荷，长度由 Cmd+Dir 决定 |
| 5+N | CRC16 | 2 | CRC-16/MODBUS，**校验范围：Cmd + Dir + Seq + Data** |
| 7+N | Footer | 2 | `0x0D 0x0A` |

## 三、指令表

### 3.1 下行帧（Server → Car，Dir=0x01）

| Cmd | 名称 | Data 长度 | Data 含义 |
|-----|------|-----------|-----------|
| `0x01` | 查询电量 | 0 | — |
| `0x02` | 查询俯仰角 | 0 | — |
| `0x03` | 一键报警 | 0 | — |
| `0x04` | 一键停止 | 0 | — |
| `0x05` | 一键出发 | 0 | — |
| `0x06` | 一键返回 | 0 | — |
| `0x07` | 设置目标点 | 4 | `[X_L][X_H][Y_L][Y_H]` (int16×2, cm) |

> 下行帧总长：Cmd 0x01~0x06 共 **8 字节**；Cmd 0x07 共 **12 字节**

### 3.2 上行应答帧（Car → Server，Dir=0x02）

| Cmd | 名称 | Data 长度 | Data 含义 |
|-----|------|-----------|-----------|
| `0x01` | 应答电量 | 4 | `[percent_L][0][0][0]` 电量百分比 int32 |
| `0x02` | 应答俯仰角 | 4 | `[angle_L][angle_H][0][0]` 角度 int16（0.1°） |
| `0x03` | 应答报警 | 1 | `[result]` 0=成功 1=失败 |
| `0x04` | 应答停止 | 1 | `[result]` 0=成功 1=失败 |
| `0x05` | 应答出发 | 1 | `[result]` 0=成功 1=失败 |
| `0x06` | 应答返回 | 1 | `[result]` 0=成功 1=失败 |
| `0x07` | 应答设置目标点 | 4 | 回显收到的 `[X_L][X_H][Y_L][Y_H]` |

> 上行帧总长：Cmd 0x01/0x02/0x07 共 **12 字节**；Cmd 0x03~0x06 共 **9 字节**

### 3.3 数据长度速查

```
DataLen(Cmd, Dir):
  Dir=0x01 (下行):      Cmd==0x07 ? 4 : 0
  Dir=0x02 (上行应答):  Cmd==0x01||Cmd==0x02||Cmd==0x07 ? 4 : 1
```

## 四、示例

```
# 查询电量
下行: F1 DD 01 01 01 [CRC] 0D 0A           ← Cmd=0x01 Dir=0x01 Seq=1 Data=空
上行: F1 DD 01 02 01 58 00 00 00 [CRC] 0D 0A  ← Cmd=0x01 Dir=0x02 Seq=1 Data=88%

# 查询俯仰角 (-12.5° = -125)
下行: F1 DD 02 01 02 [CRC] 0D 0A
上行: F1 DD 02 02 02 83 FF 00 00 [CRC] 0D 0A  ← Data=[0x83 0xFF 0x00 0x00] = -125

# 一键停止
下行: F1 DD 04 01 03 [CRC] 0D 0A           ← Cmd=0x04 Dir=0x01 Seq=3 Data=空
上行: F1 DD 04 02 03 00 [CRC] 0D 0A         ← Cmd=0x04 Dir=0x02 Seq=3 Data=0(成功)

# 设置目标点 X=100cm Y=200cm
下行: F1 DD 07 01 04 64 00 C8 00 [CRC] 0D 0A   ← Data=[0x64 0x00 0xC8 0x00]
上行: F1 DD 07 02 04 64 00 C8 00 [CRC] 0D 0A   ← 回显相同坐标
```

## 五、解包代码

```c
/**
 * 4G DTU Protocol Parser V2
 * 文件：BSP/4G_DTU/bsp_4g_protocol.c
 */

#include "bsp_4g.h"
#include <string.h>

/* ========== 帧常量 ========== */
#define DTU_HEADER1         0xF1u
#define DTU_HEADER2         0xD Du
#define DTU_FOOTER1         0x0Du
#define DTU_FOOTER2         0x0Au

#define DTU_DIR_DOWN        0x01u   /* Server→Car */
#define DTU_DIR_UP          0x02u   /* Car→Server */

#define DTU_ACK_OK          0x00u
#define DTU_ACK_FAIL        0x01u

#define DTU_FRAME_BUF_SIZE  32u

/* ========== 解析状态机 ========== */
typedef enum {
    PARSE_HDR1 = 0,
    PARSE_HDR2,
    PARSE_CMD,
    PARSE_DIR,
    PARSE_SEQ,
    PARSE_DATA,
    PARSE_CRC1,
    PARSE_CRC2,
    PARSE_FTR1,
    PARSE_FTR2,
} parse_state_t;

/* ========== 协议帧 ========== */
#pragma pack(1)
typedef struct {
    uint8_t  header1;
    uint8_t  header2;
    uint8_t  cmd;
    uint8_t  dir;
    uint8_t  seq;
    uint8_t  data[4];     /* 最多 4 字节 */
    uint16_t crc;
    uint8_t  footer1;
    uint8_t  footer2;
} dtu_frame_t;
#pragma pack()

static parse_state_t  g_state = PARSE_HDR1;
static dtu_frame_t    g_frame;
static uint8_t        g_data_idx;
static uint8_t        g_data_len;     /* 本帧期望的数据长度 */

/* ========== CRC-16/MODBUS ========== */
static uint16_t dtu_crc16(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ========== 根据 Cmd+Dir 查 Data 长度 ========== */
static uint8_t dtu_data_len(uint8_t cmd, uint8_t dir)
{
    if (dir == DTU_DIR_DOWN) {
        /* 下行：仅设置目标点带 4B 数据 */
        return (cmd == MSG_4G_SET_XY) ? 4 : 0;
    } else {
        /* 上行应答：电量/俯仰角/目标点 4B，其余 1B */
        if (cmd == MSG_4G_BATTERY     ||
            cmd == MSG_4G_PITCH_ANGLE ||
            cmd == MSG_4G_SET_XY)
            return 4;
        else
            return 1;
    }
}

/* ========== 发送帧 ========== */
static void dtu_send_frame(DTU_t *self, uint8_t cmd, uint8_t dir,
                           uint8_t seq, const uint8_t *data, uint8_t data_len)
{
    uint8_t buf[DTU_FRAME_BUF_SIZE];
    uint8_t idx = 0;

    buf[idx++] = DTU_HEADER1;
    buf[idx++] = DTU_HEADER2;
    buf[idx++] = cmd;
    buf[idx++] = dir;
    buf[idx++] = seq;

    if (data_len > 0 && data != NULL) {
        memcpy(&buf[idx], data, data_len);
        idx += data_len;
    }

    /* CRC 校验 Cmd + Dir + Seq + Data */
    uint16_t crc = dtu_crc16(&buf[2], 3 + data_len);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)(crc >> 8);

    buf[idx++] = DTU_FOOTER1;
    buf[idx++] = DTU_FOOTER2;

    self->send_fun(buf, idx);
}

/* ========== 事件回调：收到一个完整帧 ========== */
static void dtu_on_frame(DTU_t *self, dtu_frame_t *f)
{
    if (f->dir == DTU_DIR_DOWN) {
        /* ---- 处理下行指令 ---- */
        uint8_t rsp_data[4] = {0};
        uint8_t rsp_len;

        switch (f->cmd) {

        case MSG_4G_BATTERY:       /* 查询电量 */
            {
                int32_t bat = 88;  /* TODO: 从实际传感器读取 */
                rsp_data[0] = (uint8_t)(bat & 0xFF);
                rsp_data[1] = (uint8_t)((bat >> 8) & 0xFF);
                rsp_data[2] = (uint8_t)((bat >> 16) & 0xFF);
                rsp_data[3] = (uint8_t)((bat >> 24) & 0xFF);
                rsp_len = 4;
            }
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        case MSG_4G_PITCH_ANGLE:   /* 查询俯仰角 */
            {
                int32_t angle = 0; /* TODO: 从 IMU 读取，单位 0.1° */
                rsp_data[0] = (uint8_t)(angle & 0xFF);
                rsp_data[1] = (uint8_t)((angle >> 8) & 0xFF);
                rsp_data[2] = (uint8_t)((angle >> 16) & 0xFF);
                rsp_data[3] = (uint8_t)((angle >> 24) & 0xFF);
                rsp_len = 4;
            }
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        case MSG_4G_ALARM_START:   /* 一键报警 */
            self->state  = MSG_4G_ALARM_START;
            self->rx_flag = 1;
            rsp_data[0] = DTU_ACK_OK;
            rsp_len = 1;
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        case MSG_4G_ONEKEY_STOP:   /* 一键停止 */
            self->state  = MSG_4G_ONEKEY_STOP;
            self->rx_flag = 1;
            rsp_data[0] = DTU_ACK_OK;
            rsp_len = 1;
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        case MSG_4G_ONEKEY_START:  /* 一键出发 */
            self->state  = MSG_4G_ONEKEY_START;
            self->rx_flag = 1;
            rsp_data[0] = DTU_ACK_OK;
            rsp_len = 1;
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        case MSG_4G_ONEKEY_RETURN: /* 一键返回 */
            self->state  = MSG_4G_ONEKEY_RETURN;
            self->rx_flag = 1;
            rsp_data[0] = DTU_ACK_OK;
            rsp_len = 1;
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        case MSG_4G_SET_XY:        /* 设置目标点 */
            /* 回显坐标 */
            memcpy(rsp_data, f->data, 4);
            rsp_len = 4;
            self->state  = MSG_4G_SET_XY;
            self->rx_flag = 1;
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;

        default:
            /* 未知指令，仍应答 */
            rsp_data[0] = DTU_ACK_FAIL;
            rsp_len = 1;
            dtu_send_frame(self, f->cmd, DTU_DIR_UP, f->seq, rsp_data, rsp_len);
            break;
        }
    }
    else if (f->dir == DTU_DIR_UP) {
        /* Car 收到应答（通常不需要处理，仅 debug） */
        DTU_DEBUG_OUT("DTU: recv uplink ACK cmd=0x%02X\n", f->cmd);
    }
}

/* ========== 字节级解析器 ========== */
bool dtu_parse_byte(DTU_t *self, uint8_t byte)
{
    switch (g_state) {

    case PARSE_HDR1:
        if (byte == DTU_HEADER1) {
            g_frame.header1 = byte;
            g_state = PARSE_HDR2;
        }
        break;

    case PARSE_HDR2:
        if (byte == DTU_HEADER2) {
            g_frame.header2 = byte;
            g_state = PARSE_CMD;
        } else if (byte != DTU_HEADER1) {
            g_state = PARSE_HDR1;
        }
        /* byte==0xF1: 留在 HDR2，相当于重新开始匹配 */
        break;

    case PARSE_CMD:
        if (byte >= 0x01 && byte <= 0x07) {
            g_frame.cmd = byte;
            g_state = PARSE_DIR;
        } else {
            g_state = PARSE_HDR1;   /* 非法 Cmd */
        }
        break;

    case PARSE_DIR:
        if (byte == DTU_DIR_DOWN || byte == DTU_DIR_UP) {
            g_frame.dir = byte;
            g_state = PARSE_SEQ;
        } else {
            g_state = PARSE_HDR1;
        }
        break;

    case PARSE_SEQ:
        g_frame.seq = byte;
        g_data_len  = dtu_data_len(g_frame.cmd, g_frame.dir);
        g_data_idx  = 0;
        if (g_data_len > 0) {
            g_state = PARSE_DATA;
        } else {
            g_state = PARSE_CRC1;
        }
        break;

    case PARSE_DATA:
        g_frame.data[g_data_idx++] = byte;
        if (g_data_idx >= g_data_len) {
            g_state = PARSE_CRC1;
        }
        break;

    case PARSE_CRC1:
        g_frame.crc = byte;
        g_state = PARSE_CRC2;
        break;

    case PARSE_CRC2:
        g_frame.crc |= (uint16_t)byte << 8;
        g_state = PARSE_FTR1;
        break;

    case PARSE_FTR1:
        if (byte == DTU_FOOTER1) {
            g_frame.footer1 = byte;
            g_state = PARSE_FTR2;
        } else {
            g_state = PARSE_HDR1;
        }
        break;

    case PARSE_FTR2:
        g_state = PARSE_HDR1;
        if (byte == DTU_FOOTER2) {
            g_frame.footer2 = byte;
            /* ---- 帧尾正确，开始 CRC 校验 ---- */
            uint8_t crc_buf[8];  /* Cmd+Dir+Seq+Data(4B) max = 8B */
            crc_buf[0] = g_frame.cmd;
            crc_buf[1] = g_frame.dir;
            crc_buf[2] = g_frame.seq;
            if (g_data_len > 0) {
                memcpy(&crc_buf[3], g_frame.data, g_data_len);
            }
            uint16_t calc = dtu_crc16(crc_buf, 3 + g_data_len);
            if (calc == g_frame.crc) {
                dtu_on_frame(self, &g_frame);
                return true;
            } else {
                DTU_DEBUG_OUT("DTU CRC Error: calc=0x%04X recv=0x%04X\n", calc, g_frame.crc);
            }
        }
        /* 帧尾不对也返回 HEADER1 */
        break;
    }
    return false;
}

/* ========== 环形缓冲区批量解析（供 parser_fun 调用） ========== */
dtu_status_t dtu_parser(DTU_t *const self, uint8_t *buf, uint16_t len)
{
    if (NULL == self) return DTU_ERRORPARAMETER;

    while (RingByteBuffer_size(&ring5_rx_DMA_buf) > 0) {
        uint8_t byte = RingByteBuffer_popByte(&ring5_rx_DMA_buf);
        dtu_parse_byte(self, byte);
    }

    return DTU_OK;
}
```

## 六、协议特点

1. **所有指令上位机发起**：Car 只应答，不主动上报，协议简单无歧义
2. **下行极简**：大部分指令只有 8 字节（帧头2+功能1+方向1+序列1+CRC2+帧尾2），设置目标点 12 字节
3. **CRC 精确覆盖**：只校验 Cmd+Dir+Seq+Data（不含帧头帧尾），避免帧同步字节损坏漏检
4. **序列号机制**：Seq 自增，应答帧回传相同 Seq，可检测丢帧/重发
5. **无 Length 字段**：Data 长度由 Cmd+Dir 查表决定，减少一字节开销
6. **帧头重入**：状态机在任意状态遇到 `0xF1` 都能重新同步，抗干扰
