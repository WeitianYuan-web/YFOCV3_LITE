# Motor CAN Protocol V1.0

## 1. 基础定义

```text
CAN              CAN 2.0 Standard
Identifier       11 bit
DLC              8 Byte
Baudrate         1 Mbps
Byte Order       Little Endian
Motor ID         1 ~ 63
```

CAN ID 分配：

| CAN ID | 方向 | 功能 |
|---|---|---|
| `0x100 + ID` | 主机 → 电机 | Motion Command |
| `0x180 + ID` | 主机 → 电机 | Control Gains |
| `0x200 + ID` | 主机 → 电机 | Management Command |
| `0x280 + ID` | 电机 → 主机 | Command ACK |
| `0x300 + ID` | 电机 → 主机 | Motion Feedback |
| `0x380 + ID` | 电机 → 主机 | Status Response |
| `0x3C0 + ID` | 电机 → 主机 | Encoder Calibration Report |

---

# 2. Motion Command

```text
CAN ID = 0x100 + MotorID
Direction = Master → Motor
DLC = 8
```

## Payload

| Byte | 数据 | 类型 | 分辨率 |
|---|---|---|---|
| 0~3 | Position Desired | `int32` | `0.0001 rad/LSB` |
| 4~5 | Velocity Desired | `int16` | `0.01 rad/s/LSB` |
| 6~7 | Torque Feedforward | `int16` | `0.001 Nm/LSB` |

```text
Byte0  Position[7:0]
Byte1  Position[15:8]
Byte2  Position[23:16]
Byte3  Position[31:24]

Byte4  Velocity[7:0]
Byte5  Velocity[15:8]

Byte6  TorqueFF[7:0]
Byte7  TorqueFF[15:8]
```

换算：

```text
Position(rad)   = PositionRaw × 0.0001
Velocity(rad/s) = VelocityRaw × 0.01
TorqueFF(Nm)    = TorqueRaw × 0.001
```

控制律：

\[
Torque_{cmd}
=
K_p(Position_{des}-Position)
+
K_d(Velocity_{des}-Velocity)
+
Torque_{ff}
\]

## 应答

**每收到一帧有效 Motion Command，电机返回一帧 Motion Feedback。**

```text
Motion Command
0x100 + ID
      ↓
Motion Feedback
0x300 + ID
```

Motion Command **没有额外 ACK**，`0x300 + ID` 即为它的反馈帧。

---

# 3. Motion Feedback

```text
CAN ID = 0x300 + MotorID
Direction = Motor → Master
DLC = 8
```

## Payload

| Byte | 数据 | 类型 | 分辨率 |
|---|---|---|---|
| 0~3 | Position Actual | `int32` | `0.0001 rad/LSB` |
| 4~5 | Velocity Actual | `int16` | `0.01 rad/s/LSB` |
| 6~7 | Torque Actual | `int16` | `0.001 Nm/LSB` |

```text
Byte0  Position[7:0]
Byte1  Position[15:8]
Byte2  Position[23:16]
Byte3  Position[31:24]

Byte4  Velocity[7:0]
Byte5  Velocity[15:8]

Byte6  Torque[7:0]
Byte7  Torque[15:8]
```

换算：

```text
Position(rad)   = PositionRaw × 0.0001
Velocity(rad/s) = VelocityRaw × 0.01
Torque(Nm)      = TorqueRaw × 0.001
```

位置采用连续多圈机械角度：

```text
Position = 0                  机械零点
Position = +2π                正向 1 圈
Position = +20π               正向 10 圈
Position = -2π                反向 1 圈
```

---

# 4. Control Gains

设置电机内部运控模式的 `Kp` 和 `Kd`。

```text
CAN ID = 0x180 + MotorID
Direction = Master → Motor
DLC = 8
```

## Payload

| Byte | 内容 |
|---|---|
| 0 | Sequence |
| 1 | Reserved = 0 |
| 2~3 | Kp `uint16` |
| 4~5 | Kd `uint16` |
| 6~7 | Reserved = 0 |

参数定义：

```text
Kp:
    uint16
    0.01 Nm/rad / LSB
    Range = 0 ~ 655.35 Nm/rad

Kd:
    uint16
    0.001 Nm·s/rad / LSB
    Range = 0 ~ 65.535 Nm·s/rad
```

## 应答

需要 ACK：

```text
Control Gains
0x180 + ID
      ↓
Command ACK
0x280 + ID
```

ACK 中：

```text
Command = 0x20   // SET_GAINS
```

---

# 5. Management Command

```text
CAN ID = 0x200 + MotorID
Direction = Master → Motor
DLC = 8
```

统一格式：

| Byte | 内容 |
|---|---|
| 0 | Command |
| 1 | Sequence |
| 2~7 | Arguments / Reserved |

Command 定义：

```text
0x01  ENABLE
0x02  DISABLE
0x03  SET_ZERO
0x04  CLEAR_FAULT
0x05  START_ENCODER_CALIBRATION

0x10  GET_STATUS
```

---

# 6. ENABLE

请求：

```text
CAN ID = 0x200 + ID

Byte0 = 0x01
Byte1 = Sequence
Byte2~7 = 0
```

执行：

```text
检查故障
    ↓
启动驱动/PWM/FOC
    ↓
进入 READY
    ↓
发送 ACK
```

应答：

```text
0x280 + ID
```

---

# 7. DISABLE

请求：

```text
Byte0 = 0x02
Byte1 = Sequence
```

执行：

```text
Torque Command = 0
       ↓
关闭 PWM
       ↓
State = DISABLED
       ↓
ACK
```

应答：

```text
0x280 + ID
```

---

# 8. SET_ZERO

请求：

```text
Byte0 = 0x03
Byte1 = Sequence
```

作用：

```text
将当前输出轴连续机械位置设置为：

Position = 0
```

应答：

```text
0x280 + ID
```

---

# 9. CLEAR_FAULT

请求：

```text
Byte0 = 0x04
Byte1 = Sequence
```

仅在实际故障条件已经消失时清除 Fault。

应答：

```text
0x280 + ID
```

如果故障条件仍存在：

```text
Result = FAULT_ACTIVE
```

---

# 10. START_ENCODER_CALIBRATION

启动编码器校准。

请求：

```text
CAN ID = 0x200 + MotorID

Byte0 = 0x05
Byte1 = Sequence
Byte2~7 = 0
```

校准仅允许在：

```text
Motor State = DISABLED
```

状态下启动。

收到命令后：

```text
检查当前状态
      ↓
检查基础硬件状态
      ↓
接受校准任务
      ↓
Motor State = CALIBRATING
      ↓
返回 ACK
      ↓
开始编码器校准
```

## 应答

首先返回：

```text
Command ACK
CAN ID = 0x280 + MotorID
```

ACK：

```text
Command = 0x05
Sequence = 请求中的 Sequence
Result = OK
State = CALIBRATING
```

这里的 `ACK = OK` 表示：

> 校准任务已经成功启动。

**不表示校准已经完成。**

校准完成状态由 `Encoder Calibration Report` 上报。

---

# 11. Encoder Calibration Report

编码器校准过程中，由电机主动上报。

```text
CAN ID = 0x3C0 + MotorID
Direction = Motor → Master
DLC = 8
```

## Payload

| Byte | 内容 |
|---|---|
| 0 | Sequence |
| 1 | Calibration State |
| 2 | Progress |
| 3 | Calibration Stage |
| 4~5 | Error Code `uint16` |
| 6~7 | Reserved |

### Sequence

与 `START_ENCODER_CALIBRATION` 请求中的 Sequence 一致。

用于主机确认当前进度属于哪一次校准任务。

---

## Calibration State

```text
0x00  IDLE
0x01  RUNNING
0x02  SUCCESS
0x03  FAILED
```

---

## Progress

```text
uint8
Range = 0 ~ 100

0   = 0%
25  = 25%
50  = 50%
100 = 100%
```

---

## Calibration Stage

```text
0x00  IDLE
0x01  PRE_CHECK
0x02  MOTOR_ALIGNMENT
0x03  ENCODER_SAMPLING
0x04  PARAMETER_CALCULATION
0x05  PARAMETER_SAVE
0x06  COMPLETE
```

---

## Error Code

```text
0x0000  NO_ERROR

0x0001  INVALID_STATE
0x0002  ENCODER_SIGNAL_ERROR
0x0003  CALIBRATION_TIMEOUT
0x0004  CALIBRATION_DATA_INVALID
0x0005  PARAMETER_SAVE_FAILED
0x0006  MOTOR_CONTROL_ERROR

0x00FF  INTERNAL_ERROR
```

---

## 上报规则

校准过程中：

```text
State = RUNNING
```

电机周期主动发送：

```text
0x3C0 + ID
```

推荐周期：

```text
50 ms
```

即约：

```text
20 Hz
```

当 Stage 发生变化时立即额外发送一次。

---

## 校准成功

最终上报：

```text
Calibration State = SUCCESS
Progress          = 100
Calibration Stage = COMPLETE
Error Code        = 0x0000
```

随后：

```text
Motor State = DISABLED
```

电机需要重新收到 `ENABLE` 才允许进入运行状态。

---

## 校准失败

最终上报：

```text
Calibration State = FAILED
Progress          = 当前进度
Error Code        = 实际错误码
```

同时：

```text
Motor State = FAULT
Fault.CALIBRATION_FAULT = 1
```

之后需要：

```text
CLEAR_FAULT
      ↓
重新 START_ENCODER_CALIBRATION
```

---

# 12. GET_STATUS

请求：

```text
CAN ID = 0x200 + MotorID

Byte0 = 0x10
Byte1 = Sequence
Byte2~7 = 0
```

应答：

```text
CAN ID = 0x380 + MotorID
```

GET_STATUS **不再额外返回 Command ACK**。

`Status Response` 本身就是应答。

---

# 13. Status Response

```text
CAN ID = 0x380 + MotorID
Direction = Motor → Master
DLC = 8
```

## Payload

| Byte | 数据 |
|---|---|
| 0 | Motor State |
| 1~2 | Fault Code `uint16` |
| 3~4 | Bus Voltage `uint16` |
| 5~6 | Motor Temperature `int16` |
| 7 | Warning |

---

## Motor State

```text
0x00  DISABLED
0x01  READY
0x02  RUNNING
0x03  FAULT
0x04  CALIBRATING
```

状态转换：

```text
Power On
   ↓
DISABLED
   ↓ ENABLE
READY
   ↓ Motion Command
RUNNING
```

编码器校准：

```text
DISABLED
   ↓ START_ENCODER_CALIBRATION
CALIBRATING
   ↓ SUCCESS
DISABLED
```

---

# 14. Fault Code

```text
uint16
```

Bit 定义：

```text
bit0   OVER_TEMPERATURE
bit1   OVER_CURRENT
bit2   OVER_VOLTAGE
bit3   UNDER_VOLTAGE

bit4   ENCODER_FAULT
bit5   STALL_OVERLOAD
bit6   DRIVER_FAULT
bit7   CAN_TIMEOUT

bit8   POSITION_ERROR
bit9   MOTOR_PHASE_FAULT
bit10  CALIBRATION_FAULT

bit11~15 Reserved
```

多个故障允许同时置位。

---

# 15. Bus Voltage

```text
uint16
0.01 V / LSB
```

换算：

```text
Voltage(V) = Raw × 0.01
```

---

# 16. Motor Temperature

```text
int16
0.1 °C / LSB
```

换算：

```text
Temperature(°C) = Raw × 0.1
```

---

# 17. Warning

```text
Byte7
```

定义：

```text
bit0  MOTOR_TEMPERATURE_WARNING
bit1  BUS_VOLTAGE_WARNING
bit2  CURRENT_LIMIT_ACTIVE
bit3  TORQUE_LIMIT_ACTIVE
bit4  POSITION_LIMIT_WARNING

bit5~7 Reserved
```

`Warning` 不要求电机进入 Fault。

---

# 18. Command ACK

```text
CAN ID = 0x280 + MotorID
Direction = Motor → Master
DLC = 8
```

格式：

| Byte | 内容 |
|---|---|
| 0 | Command |
| 1 | Sequence |
| 2 | Result |
| 3 | Motor State |
| 4~5 | Fault Code |
| 6~7 | Reserved |

Command：

```text
0x01 ENABLE
0x02 DISABLE
0x03 SET_ZERO
0x04 CLEAR_FAULT
0x05 START_ENCODER_CALIBRATION
0x20 SET_GAINS
```

Result：

```text
0x00  OK
0x01  INVALID_COMMAND
0x02  INVALID_STATE
0x03  PARAMETER_OUT_OF_RANGE
0x04  FAULT_ACTIVE
0x05  BUSY
0x06  INTERNAL_ERROR
```

---

# 19. 应答规则

| 主机发送 | 电机应答 |
|---|---|
| `Motion Command 0x100` | **`Motion Feedback 0x300`** |
| `Control Gains 0x180` | **`Command ACK 0x280`** |
| `ENABLE` | **`Command ACK 0x280`** |
| `DISABLE` | **`Command ACK 0x280`** |
| `SET_ZERO` | **`Command ACK 0x280`** |
| `CLEAR_FAULT` | **`Command ACK 0x280`** |
| `START_ENCODER_CALIBRATION` | **先 `ACK 0x280`，随后主动发送 `Calibration Report 0x3C0`** |
| `GET_STATUS` | **`Status Response 0x380`** |

电机发送以下帧后，主机不需要再次 ACK：

```text
0x280 Command ACK
0x300 Motion Feedback
0x380 Status Response
0x3C0 Encoder Calibration Report
```

---

# 20. Motion Watchdog

只有收到有效的：

```text
0x100 + MotorID
Motion Command
```

才刷新控制 Watchdog。

默认：

```text
Motion Timeout = 100 ms
```

处于 RUNNING 状态时超过 100 ms 未收到 Motion Command：

```text
Torque Command = 0
        ↓
PWM Disable
        ↓
Motor State = FAULT
        ↓
Fault.CAN_TIMEOUT = 1
```

恢复流程：

```text
CLEAR_FAULT
     ↓
ENABLE
     ↓
READY
     ↓
Motion Command
     ↓
RUNNING
```

`GET_STATUS`、`SET_GAINS`、编码器校准等低频报文均**不能刷新 Motion Watchdog**。

---

# 21. 最终 CAN ID 表

```text
0x100 + ID
Motion Command
Master → Motor

0x180 + ID
Control Gains
Master → Motor

0x200 + ID
Management Command
Master → Motor

0x280 + ID
Command ACK
Motor → Master

0x300 + ID
Motion Feedback
Motor → Master

0x380 + ID
Status Response / Fault Event
Motor → Master

0x3C0 + ID
Encoder Calibration Report
Motor → Master
```

## 推荐通信流程

```text
上电
 │
 ▼
DISABLED
 │
 ├──── Set Gains ─────────────►
 │◄──── ACK ───────────────────
 │
 ├──── Encoder Calibration ───►
 │◄──── ACK(Start Accepted) ───
 │
 │◄──── Calibration 5% ────────
 │◄──── Calibration 20% ───────
 │◄──── Calibration 50% ───────
 │◄──── Calibration 80% ───────
 │◄──── Calibration 100% ──────
 │
 ├──── Enable ────────────────►
 │◄──── ACK ───────────────────
 │
 ▼
READY
 │
 ├──── Motion Command ────────►
 │◄──── Motion Feedback ───────
 │
 ├──── Motion Command ────────►
 │◄──── Motion Feedback ───────
 │
 ▼
RUNNING
```