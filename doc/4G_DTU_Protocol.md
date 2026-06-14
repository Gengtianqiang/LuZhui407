# 4G DTU 通信协议说明

> 适用代码：`BSP/4G_DTU/bsp_4g.c`、`BSP/4G_DTU/bsp_4g.h`  
> 当前协议为 ASCII 字符串协议，不是二进制帧协议。

---

## 1. 通信方向

当前 4G DTU 通信采用：

```text
上位机 / 4G 云端 / 手机端  --->  小车
小车                     --->  上位机 / 4G 云端 / 手机端
```

上位机发送控制指令，小车解析后执行对应动作，并通过 UART5 DMA 返回一段字符串作为应答。

发送接口：

```c
HAL_UART_Transmit_DMA(&huart5, buf, len);
```

---

## 2. 下行帧格式

当前代码使用的帧格式为：

```text
@ + 指令码 + 数据区 + a
```

其中：

| 字段 | 类型 | 说明 |
|---|---|---|
| `@` | char | 帧头，固定为 `@` |
| 指令码 | char | 位于 `buf[1]`，例如 `'1'`、`'2'`、`'7'` |
| 数据区 | string | 可选，只有设置坐标时使用 |
| `a` | char | 帧尾，固定为 `a` |

代码中的定义：

```c
#define FRAME_HEAD '@'
#define FRAME_TAIL 'a'
```

解析时会检查：

```c
buf[0] == '@'
buf[len - 1] == 'a'
```

如果帧头或帧尾不正确，则该帧无效，不会执行对应指令，也不会返回正常应答。

---

## 3. 指令总表

| 指令码 | 接收示例 | 功能 | 小车内部状态 |
|---|---|---|---|
| `'1'` | `@1a` | 查询电池电压 | `MSG_4G_BATTERY` |
| `'2'` | `@2a` | 查询俯仰角 | `MSG_4G_PITCH_ANGLE` |
| `'3'` | `@3a` | 一键报警 / 关闭报警 | `MSG_4G_ALARM_START` |
| `'4'` | `@4a` | 一键停止 / 取消停止 | `MSG_4G_ONEKEY_STOP` |
| `'5'` | `@5a` | 一键出发 | `MSG_4G_ONEKEY_START` |
| `'6'` | `@6a` | 一键返回 | `MSG_4G_ONEKEY_RETURN` |
| `'7'` | `@7[2.0,2.0]1a` | 设置目标点 XY 坐标 | `MSG_4G_SET_XY` |

---

## 4. 各指令详细说明

### 4.1 查询电池电压

#### 下发格式

```text
@1a
```

#### 执行动作

小车读取变量：

```c
Volt
```

#### 返回格式

```text
Battery level is %.1fv\n
```

#### 返回示例

如果：

```c
Volt = 12.3;
```

则返回：

```text
Battery level is 12.3v
```

---

### 4.2 查询俯仰角

#### 下发格式

```text
@2a
```

#### 执行动作

小车读取 IMU 俯仰角：

```c
myimu.euler.pitch
```

#### 返回格式

```text
Pitch angle is %.1f\n
```

#### 返回示例

如果：

```c
myimu.euler.pitch = -3.5;
```

则返回：

```text
Pitch angle is -3.5
```

---

### 4.3 一键报警

#### 下发格式

```text
@3a
```

#### 执行动作

每收到一次该指令，切换一次报警状态：

```c
self->buzzer_flag += 1;
if(self->buzzer_flag == 2)
{
    self->buzzer_flag = 0;
}
```

即：

| 当前状态 | 收到 `@3a` 后 |
|---|---|
| 关闭 | 打开 |
| 打开 | 关闭 |

#### 返回格式

```text
Alarm %s\n
```

#### 返回示例

报警打开时：

```text
Alarm ON
```

报警关闭时：

```text
Alarm OFF
```

---

### 4.4 一键停止

#### 下发格式

```text
@4a
```

#### 执行动作

每收到一次该指令，切换一次停止状态：

```c
self->stop_flag += 1;
if(self->stop_flag == 2)
{
    self->stop_flag = 0;
}
```

即：

| 当前状态 | 收到 `@4a` 后 |
|---|---|
| 未停止 | 进入停止 |
| 已停止 | 取消停止 |

#### 返回格式

```text
One-key stop %s\n
```

#### 返回示例

停止激活时：

```text
One-key stop Activated
```

停止取消时：

```text
One-key stop Deactivated
```

---

### 4.5 一键出发

#### 下发格式

```text
@5a
```

#### 执行动作

设置出发标志位：

```c
self->start_flag = 1;
```

#### 返回内容

```text
One-key start received.
```

---

### 4.6 一键返回

#### 下发格式

```text
@6a
```

#### 执行动作

设置返回标志位：

```c
self->return_flag = 1;
```

#### 返回内容

```text
One-key return received.
```

---

### 4.7 设置目标点 XY 坐标

#### 下发格式

```text
@7[x,y]1a
```

其中：

| 字段 | 说明 |
|---|---|
| `@` | 帧头 |
| `7` | 设置坐标指令 |
| `[x,y]` | 目标点坐标 |
| `1` | 当前代码中未使用，可作为保留位 |
| `a` | 帧尾 |

> 注意：代码真正解析的是中括号 `[]` 里面的内容。只要能找到 `[` 和 `]`，并且里面是 `数字,数字` 格式，就可以解析。

#### 合法示例

```text
@7[2,2]1a
@7[2.0,2.0]1a
@7[10,20]1a
@7[1.5,6.8]1a
@7[-1.2,3.4]1a
```

#### 解析方式

代码会先找到：

```c
left_bracket = strchr((char*)buf, '[');
right_bracket = strchr((char*)buf, ']');
```

然后把中括号内的内容复制到 `xy_buf`。

例如收到：

```text
@7[10.5,20.3]1a
```

则：

```c
xy_buf = "10.5,20.3";
```

然后使用：

```c
sscanf(xy_buf, "%f,%f", &x, &y)
```

解析两个浮点数。

解析成功后保存到：

```c
self->point.x = x;
self->point.y = y;
```

#### 返回格式

```text
Set XY OK: x=%.2f, y=%.2f\n
```

#### 返回示例

收到：

```text
@7[10,20]1a
```

小车内部保存：

```c
self->point.x = 10.0f;
self->point.y = 20.0f;
```

返回：

```text
Set XY OK: x=10.00, y=20.00
```

收到：

```text
@7[1.5,6.8]1a
```

小车内部保存：

```c
self->point.x = 1.5f;
self->point.y = 6.8f;
```

返回：

```text
Set XY OK: x=1.50, y=6.80
```

---

## 5. 返回指令 / 应答总表

| 上位机发送 | 小车动作 | 小车返回 |
|---|---|---|
| `@1a` | 查询电压 | `Battery level is %.1fv\n` |
| `@2a` | 查询俯仰角 | `Pitch angle is %.1f\n` |
| `@3a` | 切换报警状态 | `Alarm ON\n` 或 `Alarm OFF\n` |
| `@4a` | 切换停止状态 | `One-key stop Activated\n` 或 `One-key stop Deactivated\n` |
| `@5a` | 设置 `start_flag = 1` | `One-key start received.\n` |
| `@6a` | 设置 `return_flag = 1` | `One-key return received.\n` |
| `@7[x,y]1a` | 设置 `self->point.x/y` | `Set XY OK: x=%.2f, y=%.2f\n` |

---

## 6. 初始化返回

DTU 驱动初始化完成后，会主动发送：

```text
The car is ready.
```

对应代码：

```c
self->send_fun("The car is ready.\n", strlen("The car is ready.\n"));
```

该消息表示小车 4G DTU 通信部分已经初始化完成。

---

## 7. 错误上报

当前代码中有两个错误标志位：

```c
self->imu_error_flag
self->jdy_error_flag
```

### 7.1 IMU 错误

当：

```c
self->imu_error_flag == 1
```

小车会发送：

```text
imu error
```

发送后延时 500 ms。

### 7.2 JDY 错误

当：

```c
self->jdy_error_flag == 1
```

小车会发送：

```text
jdy error
```

发送后延时 500 ms。

---

## 8. 长时间无通信时的保活 / 复位相关发送

代码中有一个超时判断：

```c
if(self->p_time->getSysTickCnt() - dtu_reast_titk > 300000)
```

如果超过该时间没有收到新数据，小车会依次发送：

```text
+++
```

延时 500 ms 后发送：

```text
atk
```

再延时 500 ms 后发送：

```text
AT+PWR\r\n
```

再延时 500 ms 后，会再次发送当前 `ack_buf` 中的内容，也就是：

```text
AT+PWR\r\n
```

因此长时间无通信时，实际发送序列为：

```text
+++
atk
AT+PWR\r\n
AT+PWR\r\n
```

---

## 9. 无效帧处理

### 9.1 帧头错误

例如：

```text
#1a
```

因为第一个字符不是 `@`，解析失败。

不会执行任何指令。

### 9.2 帧尾错误

例如：

```text
@1b
```

因为最后一个字符不是 `a`，解析失败。

不会执行任何指令。

### 9.3 未定义指令

例如：

```text
@8a
```

当前 `switch` 中没有 `'8'` 对应的处理分支，因此不会执行有效动作。

---

## 10. 当前协议特点

1. **协议简单**：只需要判断帧头、指令码、帧尾。
2. **指令码是 ASCII 字符**：例如 `'1'`，不是数值 `0x01`。
3. **没有 CRC 校验**：当前 CRC 相关代码被保留但未启用。
4. **没有长度字段**：通过 `strlen((char*)buf)` 获取长度。
5. **设置坐标使用字符串解析**：坐标格式为 `[x,y]`，通过 `sscanf` 转成 float。
6. **所有正常指令都会调用 `ack_fun` 返回字符串应答**。
7. **`@7[x,y]1a` 中 `]` 后面的 `1` 当前没有参与解析**，可以作为后续扩展字段。

---

## 11. 推荐上位机发送格式

建议上位机统一使用以下格式：

```text
@1a              查询电池电压
@2a              查询俯仰角
@3a              切换报警
@4a              切换停止
@5a              一键出发
@6a              一键返回
@7[10.0,20.0]1a 设置目标坐标 x=10.0, y=20.0
```

每条指令单独发送，不要多条指令粘在一起发送。当前解析使用 `strlen` 和首尾字符判断，若多帧粘包，可能导致解析异常。
