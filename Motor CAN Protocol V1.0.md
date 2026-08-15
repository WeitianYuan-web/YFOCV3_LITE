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
| `0x140 + ID` | 主机 → 电机 | Velocity Mode Command |
| `0x180 + ID` | 主机 → 电机 | Control Gains |
| `0x1C0 + ID` | 主机 → 电机 | Position Mode Command |
| `0x200 + ID` | 主机 → 电机 | Management Command |
| `0x280 + ID` | 电机 → 主机 | Command ACK |
| `0x300 + ID` | 电机 → 主机 | Motion Feedback |
| `0x380 + ID` | 电机 → 主机 | Status Response |
| `0x3C0 + ID` | 电机 → 主机 | Encoder Calibration Report |

### Control Mode

```text
0x00  MOTION_MODE
0x01  VELOCITY_MODE
0x02  POSITION_MODE
```

Control Mode 由 `SET_CONTROL_MODE` 管理命令显式切换。上电默认模式为 `MOTION_MODE`。

---

# 2. 实时控制命令

电机支持以下三种实时控制模式：

| 控制模式 | CAN ID | 控制量 |
|---|---|---|
| 运控模式（Motion） | `0x100 + ID` | 目标位置、目标速度、转矩/电压前馈 |
| 速度模式（Velocity） | `0x140 + ID` | 目标速度 |
| 位置模式（Position） | `0x1C0 + ID` | 目标位置、最大速度 |

Control Mode 必须先通过 `SET_CONTROL_MODE` 选择。驱动器只接受与当前 Control Mode 匹配的实时控制命令：

| 当前 Control Mode | 有效实时控制命令 |
|---|---|
| `MOTION_MODE` | Motion Command |
| `VELOCITY_MODE` | Velocity Mode Command |
| `POSITION_MODE` | Position Mode Command |

收到其他模式的实时控制命令时，驱动器不执行该帧，也不返回 Motion Feedback。三种有效实时控制命令均使用 `Motion Feedback (0x300 + ID)` 作为反馈。

协议不设置实时命令超时保护。进入 RUNNING 后，驱动器持续保持最后一帧有效实时控制命令，直到收到同模式的新命令、DISABLE、进入 FAULT 或掉电。上位机停止周期控制前必须主动发送 DISABLE。

## 2.1 Motion Command

```text
CAN ID = 0x100 + MotorID
Direction = Master → Motor
DLC = 8
```

### Payload

| Byte | 数据 | 类型 | 分辨率 |
|---|---|---|---|
| 0~3 | Position Desired | `int32` | `0.0001 rad/LSB` |
| 4~5 | Velocity Desired | `int16` | `0.1 rad/s/LSB` |
| 6~7 | Torque / Voltage Feedforward | 取决于固件输出模式 | 见下文 |

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
Velocity(rad/s) = VelocityRaw × 0.1
```

Byte6~7 的含义由固件输出模式决定：

| 固件输出模式 | 类型 | 取值/分辨率 | 换算 |
|---|---|---|---|
| 转矩模式 | `int16` | `0.001 Nm/LSB` | `TorqueFF(Nm) = Raw × 0.001` |
| 电压模式 | `uint16` | 归一化电压 `0.0 ~ 1.0` | `VoltageFF = Raw / 65535` |

电压模式中的 `VoltageFF` 是归一化电压指令：

```text
Raw = 0       → VoltageFF = 0.0
Raw = 32768   → VoltageFF ≈ 0.5
Raw = 65535   → VoltageFF = 1.0
```

其实际电压由固件的母线电压、调制方式和电压限幅决定。主机必须根据目标固件的输出模式编码 Byte6~7，不能将转矩模式的 `int16` 数值直接用于电压模式。

转矩模式控制律：

\[
Torque_{cmd}
=
K_p(Position_{des}-Position)
+
K_d(Velocity_{des}-Velocity)
+
Torque_{ff}
\]

电压模式控制律：

\[
Voltage_{cmd}
=
K_p(Position_{des}-Position)
+
K_d(Velocity_{des}-Velocity)
+
Voltage_{ff}
\]

`Voltage_cmd` 最终由固件限制在允许的归一化电压范围内。

### 应答

正常情况下，电机处理有效 Motion Command 后返回一帧 Motion Feedback。

```text
Motion Command
0x100 + ID
      ↓
Motion Feedback
0x300 + ID
```

Motion Command **没有额外 ACK**，`0x300 + ID` 即为它的反馈帧。一次接收处理中存在多帧实时控制命令时，按照第 19 节的实时帧合并规则处理。

---

## 2.2 Velocity Mode Command

速度模式使用电机内部速度环，控制量为 4 Byte 目标速度。

```text
CAN ID = 0x140 + MotorID
Direction = Master → Motor
DLC = 8
```

### Payload

| Byte | 数据 | 类型 | 分辨率 |
|---|---|---|---|
| 0~3 | Velocity Desired | `int32` | `0.1 rad/s/LSB` |
| 4~7 | Reserved = 0 | `uint32` | - |

```text
Byte0  Velocity[7:0]
Byte1  Velocity[15:8]
Byte2  Velocity[23:16]
Byte3  Velocity[31:24]
Byte4~7 = 0
```

换算：

```text
Velocity(rad/s) = VelocityRaw × 0.1
VelocityRaw     = round(Velocity(rad/s) / 0.1)
```

正值和负值分别表示两个旋转方向，具体正方向由电机安装方向和固件配置确定。

### 应答

正常情况下，电机处理有效 Velocity Mode Command 后返回一帧：

```text
Motion Feedback
CAN ID = 0x300 + MotorID
```

Velocity Mode Command 没有额外 ACK。一次接收处理中存在多帧实时控制命令时，按照第 19 节的实时帧合并规则处理。

---

## 2.3 Position Mode Command

位置模式采用位置外环、速度内环。控制量为 4 Byte 目标位置和 4 Byte 最大速度。

```text
CAN ID = 0x1C0 + MotorID
Direction = Master → Motor
DLC = 8
```

### Payload

| Byte | 数据 | 类型 | 分辨率 |
|---|---|---|---|
| 0~3 | Position Desired | `int32` | `0.0001 rad/LSB` |
| 4~7 | Maximum Velocity | `uint32` | `0.1 rad/s/LSB` |

```text
Byte0  Position[7:0]
Byte1  Position[15:8]
Byte2  Position[23:16]
Byte3  Position[31:24]

Byte4  MaxVelocity[7:0]
Byte5  MaxVelocity[15:8]
Byte6  MaxVelocity[23:16]
Byte7  MaxVelocity[31:24]
```

换算：

```text
Position(rad)        = PositionRaw × 0.0001
MaximumVelocity(rad/s) = MaxVelocityRaw × 0.1
```

`Position Desired` 使用与 Motion Command 和 Motion Feedback 相同的连续多圈机械角度。`Maximum Velocity` 是绝对值上限，不表示旋转方向；运动方向由当前位置与目标位置之间的误差决定。

### 应答

正常情况下，电机处理有效 Position Mode Command 后返回一帧：

```text
Motion Feedback
CAN ID = 0x300 + MotorID
```

Position Mode Command 没有额外 ACK。一次接收处理中存在多帧实时控制命令时，按照第 19 节的实时帧合并规则处理。

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
| 4~5 | Velocity Actual | `int16` | `0.1 rad/s/LSB` |
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
Velocity(rad/s) = VelocityRaw × 0.1
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

根据 Control Mode 分别设置运控模式的 `Kp/Kd`、速度模式速度环的 `Kp/Ki`，以及位置模式位置外环的 `Kp/Ki/Kd`。

```text
CAN ID = 0x180 + MotorID
Direction = Master → Motor
DLC = 8
```

## Payload

| Byte | 内容 |
|---|---|
| 0 | Control Mode |
| 1 | Sequence |
| 2~3 | Kp `uint16` |
| 4~5 | Ki `uint16` |
| 6~7 | Kd `uint16` |

Control Mode 使用第 1 节的统一定义：

```text
0x00  MOTION_MODE
0x01  VELOCITY_MODE
0x02  POSITION_MODE
```

所有字节均有固定定义，不再包含 Reserved 字段。对于当前控制算法未使用的增益，主机必须发送 0，驱动器保存为 0：

| Control Mode | Kp | Ki | Kd |
|---|---|---|---|
| `MOTION_MODE` | 使用 | 必须为 0 | 使用 |
| `VELOCITY_MODE` | 使用 | 使用 | 必须为 0 |
| `POSITION_MODE` | 使用 | 使用 | 使用 |

Control Mode 非法，或者要求为 0 的增益不为 0 时，返回 `PARAMETER_OUT_OF_RANGE`，原有参数保持不变。

### Motion Gains

运控模式的 `Kp` 和 `Kd` 原始编码不变，其物理单位由固件输出模式决定：

| 参数 | 类型 | 转矩模式分辨率 | 电压模式分辨率 |
|---|---|---|---|
| Kp | `uint16` | `0.01 Nm/rad/LSB` | `0.01 pu/rad/LSB` |
| Kd | `uint16` | `0.001 Nm·s/rad/LSB` | `0.001 pu·s/rad/LSB` |

### Velocity PI Gains

速度模式使用内部速度 PI，速度误差为：

\[
e_v = Velocity_{des} - Velocity
\]

控制器输出为转矩模式下的转矩指令，或者电压模式下的归一化电压指令：

\[
Output_{cmd} = K_{p,v}e_v + K_{i,v}\int e_vdt
\]

| 参数 | 类型 | 转矩模式分辨率 | 电压模式分辨率 |
|---|---|---|---|
| Velocity Kp | `uint16` | `0.001 Nm/(rad/s)/LSB` | `0.001 pu/(rad/s)/LSB` |
| Velocity Ki | `uint16` | `0.001 Nm/rad/LSB` | `0.001 pu/rad/LSB` |

### Position PID Gains

位置模式采用位置 PID 外环和速度 PI 内环。位置外环根据位置误差生成速度内环的目标速度：

\[
e_p = Position_{des} - Position
\]

\[
Velocity_{ref}
=
K_{p,p}e_p
+
K_{i,p}\int e_pdt
+
K_{d,p}\frac{de_p}{dt}
\]

`Velocity_ref` 最终限制在 Position Mode Command 给定的 `±MaximumVelocity` 范围内，再交给速度 PI 内环控制。

| 参数 | 类型 | 分辨率 |
|---|---|---|
| Position Kp | `uint16` | `0.01 s⁻¹/LSB` |
| Position Ki | `uint16` | `0.001 s⁻²/LSB` |
| Position Kd | `uint16` | `0.001/LSB` |

设置 `POSITION_MODE` Gains 只修改位置外环参数，不修改 `VELOCITY_MODE` 的速度 PI 内环参数。位置模式运行前，主机应分别配置 `VELOCITY_MODE` 和 `POSITION_MODE` Gains。

所有参数对应的原始编码范围均为：

```text
Raw = 0 ~ 65535
```

固件可以根据电机、功率级和控制周期设置更小的有效参数范围。驱动器必须先校验一帧中的全部参数，再一次性更新对应控制器的参数；任一参数越界时返回 `PARAMETER_OUT_OF_RANGE`，且本帧参数全部不生效。

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
Sequence = 请求中的 Sequence
```

ACK 不改变当前 Control Mode。Control Gains 中的 Control Mode 只用于选择要修改的参数组，实际模式切换必须使用 `SET_CONTROL_MODE`。

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
0x06  SET_CONTROL_MODE

0x10  GET_STATUS
```

## 5.1 SET_CONTROL_MODE

显式切换驱动器当前使用的实时控制模式。

请求：

```text
CAN ID = 0x200 + MotorID

Byte0 = 0x06
Byte1 = Sequence
Byte2 = Control Mode
Byte3~7 = 0
```

Control Mode：

```text
0x00  MOTION_MODE
0x01  VELOCITY_MODE
0x02  POSITION_MODE
```

模式切换只允许在以下状态执行：

```text
DISABLED
READY
```

处于 RUNNING、FAULT 或 CALIBRATING 时返回：

```text
Result = INVALID_STATE
```

切换成功后：

```text
保存新的 Control Mode
        ↓
清零新模式的积分器和微分器状态
        ↓
清除上一模式的实时目标有效标志
        ↓
保持当前 Motor State
        ↓
发送 ACK
```

Control Gains 参数不会因为切换模式而被清除。切换后必须收到一帧与新模式匹配的有效实时控制命令，电机才会使用新的实时目标。处于 READY 时，该帧同时使电机进入 RUNNING。

应答：

```text
CAN ID = 0x280 + MotorID

Command  = 0x06
Sequence = 请求中的 Sequence
Result   = OK / INVALID_STATE / PARAMETER_OUT_OF_RANGE
```

Control Mode 非法或 Byte3~7 不为 0 时返回 `PARAMETER_OUT_OF_RANGE`。

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
| 7 | Warning / Control Mode |

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
   ↓ 与当前 Control Mode 匹配的有效实时控制命令
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
bit7   Reserved

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

# 17. Warning / Control Mode

```text
Byte7
```

Byte7 同时包含 Warning 和当前 Control Mode：

```text
bit0  MOTOR_TEMPERATURE_WARNING
bit1  BUS_VOLTAGE_WARNING
bit2  CURRENT_LIMIT_ACTIVE
bit3  TORQUE_LIMIT_ACTIVE
bit4  POSITION_LIMIT_WARNING

bit5~6 CONTROL_MODE
bit7   Reserved
```

Control Mode 解码：

```text
ControlMode = (Byte7 >> 5) & 0x03

0x00  MOTION_MODE
0x01  VELOCITY_MODE
0x02  POSITION_MODE
0x03  Reserved
```

Warning 使用 `Byte7 & 0x1F` 提取，且不要求电机进入 Fault。主机可以通过 GET_STATUS 检查 `SET_CONTROL_MODE` 后的当前模式。

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
0x06 SET_CONTROL_MODE
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
| `Motion Command 0x100` | 当前模式为 MOTION 时返回 **`Motion Feedback 0x300`** |
| `Velocity Mode Command 0x140` | 当前模式为 VELOCITY 时返回 **`Motion Feedback 0x300`** |
| `Control Gains 0x180` | **`Command ACK 0x280`** |
| `Position Mode Command 0x1C0` | 当前模式为 POSITION 时返回 **`Motion Feedback 0x300`** |
| `ENABLE` | **`Command ACK 0x280`** |
| `DISABLE` | **`Command ACK 0x280`** |
| `SET_ZERO` | **`Command ACK 0x280`** |
| `CLEAR_FAULT` | **`Command ACK 0x280`** |
| `START_ENCODER_CALIBRATION` | **先 `ACK 0x280`，随后主动发送 `Calibration Report 0x3C0`** |
| `SET_CONTROL_MODE` | **`Command ACK 0x280`** |
| `GET_STATUS` | **`Status Response 0x380`** |

表中的实时控制命令应答表示正常的单帧处理结果；如果一次接收处理中积累了多帧实时控制命令，不要求逐帧返回 Motion Feedback，按照下述合并规则处理。

电机发送以下帧后，主机不需要再次 ACK：

```text
0x280 Command ACK
0x300 Motion Feedback
0x380 Status Response
0x3C0 Encoder Calibration Report
```

## 19.1 实时控制帧合并规则

一次接收处理是指：驱动器进入 CAN 接收任务后，依次取出当时 RX FIFO 中的待处理帧，直到 FIFO 被取空。

对于以下三种实时控制命令：

```text
0x100 + MotorID  Motion Command
0x140 + MotorID  Velocity Mode Command
0x1C0 + MotorID  Position Mode Command
```

如果一次接收处理中存在多帧，驱动器必须读取并校验所有帧，但只保留最后收到的一帧与当前 Control Mode 匹配的有效实时控制命令：

```text
读取并校验 RX FIFO 中的所有帧
              ↓
管理/参数命令按接收顺序逐帧处理并应答
              ↓
保留最后一帧与当前 Control Mode 匹配的有效命令
              ↓
执行最新有效实时控制命令一次
              ↓
返回一帧最新 Motion Feedback
```

具体规则：

- “最新”以驱动器从 RX FIFO 取出帧的顺序为准。
- 实时控制帧不能隐式切换 Control Mode。与当前模式不匹配的实时帧视为无效帧，不执行且不反馈。
- 无效帧、模式不匹配帧、DLC 错误帧和参数越界帧不能覆盖此前已经收到的有效实时控制命令。
- 驱动器必须取出并检查所有待处理帧，不能因为只执行最新命令而只读取 RX FIFO 的最后一帧。
- 本次没有有效实时控制命令时，不执行新的控制量，也不因这些无效帧发送 Motion Feedback。
- Motion Feedback 表示执行最新控制命令后的电机当前状态，不是对每一帧实时控制命令的独立确认。

当实时控制命令与 Control Gains、Management Command 同时积压时，驱动器先按接收顺序处理参数和管理命令，再根据处理后的 Motor State 和 Control Mode 校验并执行最新实时控制命令。这使 DISABLE、SET_CONTROL_MODE、故障处理等管理动作优先于实时控制输出。

## 19.2 非实时命令处理规则

以下命令不能合并为最新一帧，驱动器必须按接收顺序逐帧处理并给出对应应答：

| 命令类型 | 处理方式 |
|---|---|
| Control Gains | 每帧处理并返回 Command ACK |
| ENABLE / DISABLE | 每帧处理并返回 Command ACK |
| SET_ZERO / CLEAR_FAULT | 每帧处理并返回 Command ACK |
| START_ENCODER_CALIBRATION | 每帧处理并返回 Command ACK；接受后再主动上报进度 |
| SET_CONTROL_MODE | 每帧处理并返回 Command ACK |
| GET_STATUS | 每帧返回 Status Response |

新的非实时请求使用新的 Sequence。主机因应答超时重发同一请求时，必须使用与原请求相同的 Command 和 Sequence。推荐驱动器缓存最近一次非实时请求及其应答；收到相同 Command 和 Sequence 的重复请求时，重新发送缓存的应答，不重复执行命令。

## 19.3 上位机请求建议

### 实时控制请求

- 对同一电机只周期发送当前所选模式的一种实时控制命令，不要在一个控制周期内同时发送 Motion、Velocity 和 Position 命令。
- 使用满足控制性能和总线负载要求的稳定周期发送，不要集中突发发送多帧；常用建议值为 10~20 ms。
- 目标值更新速度高于 CAN 发送速度时，上位机只发送当前发送时刻的最新目标值，不应把已经过时的目标值排队补发。
- 不要求发送一帧后等待对应 Motion Feedback 才能发送下一帧。上位机应把 Motion Feedback 作为最新状态流处理，并记录接收时间用于判断反馈超时。
- 切换实时控制模式时，停止发送旧模式帧并清空上位机的旧实时帧发送队列，然后按照 `DISABLE → SET_CONTROL_MODE → ENABLE` 的顺序操作。收到各步骤 ACK 后，再开始周期发送新模式的实时控制帧。
- 通信停止不会自动清零控制输出或进入 Fault，因此上位机正常停止控制时必须发送 DISABLE，并监控反馈超时作为通信异常提示。
- 多电机系统应错开各电机请求的发送相位，避免所有电机在同一时刻集中发送命令和反馈。

### 非实时请求

- 对同一电机，推荐同一时刻只保留一个等待应答的非实时请求；收到对应 Sequence 的应答或确认超时后，再发送下一条请求。
- 设置 Control Gains 时，在 Byte0 分别指定 `MOTION_MODE`、`VELOCITY_MODE` 或 `POSITION_MODE`，每次收到对应 ACK 后再设置下一组参数。进入速度模式前应确认速度 PI 已配置；进入位置模式前应确认速度 PI 和位置 PID 均已配置。
- 每个新请求的 Sequence 加 1，`0xFF` 后回绕到 `0x00`；重试同一请求时保持原 Sequence 不变。
- 推荐 ACK 和 Status Response 等待超时为 20 ms；超时后最多重试 2 次。具体值可以根据固件任务周期和总线负载调整。
- 收到 Command ACK 时，同时校验 CAN ID、Command 和 Sequence，不能只根据 Sequence 判断请求是否完成。
- SET_ZERO、ENABLE、SET_CONTROL_MODE、START_ENCODER_CALIBRATION 等具有状态影响的命令，在未收到应答前不要改用新的 Sequence 重发，否则驱动器可能将其视为新的操作。
- START_ENCODER_CALIBRATION 收到 `ACK = OK` 后进入进度等待阶段，不要继续重发启动命令；完成状态以 Encoder Calibration Report 为准。
- 管理操作期间避免与实时控制请求混发。推荐先停止实时请求，完成管理命令及应答，再恢复对应模式的周期控制请求。

---

# 20. 最终 CAN ID 表

```text
0x100 + ID
Motion Command
Master → Motor

0x140 + ID
Velocity Mode Command
Master → Motor

0x180 + ID
Control Gains
Master → Motor

0x1C0 + ID
Position Mode Command
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
 ├──── Set Gains by Mode ─────►
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
 ├──── Set Control Mode ──────►
 │◄──── ACK ───────────────────
 │
 ├──── Enable ────────────────►
 │◄──── ACK ───────────────────
 │
 ▼
READY
 │
 ├──── 与当前模式匹配的 ──────►
 │     实时控制命令
 │◄──── Motion Feedback ───────
 │
 ├──── 同类型实时控制命令 ────►
 │◄──── Motion Feedback ───────
 │
 ▼
RUNNING
```
