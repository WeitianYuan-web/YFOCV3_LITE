# YFOCV3 最小电压伺服（LC-ESC）

面向 **LC-ESC / STM32G431CBT6 + FD6288 + MT6701** 的最小电压模式伺服固件。上电校准或加载参数后直接进入运控，用 CAN 收 Motor Protocol V1.0 运控帧。

## 构建

需要 `arm-none-eabi-gcc`（STM32CubeIDE / CubeCLT / Arm GNU Toolchain）与 CMake + Ninja。CMake 会在 PATH 以及常见 Windows 安装路径里查找这些工具，不必手动改环境变量。

产物：`build/lc-voltage/YFOCV3.elf`、`YFOCV3.bin`。

### Windows

CLion：打开本仓库，选择 CMake preset `lc-voltage`，再 Build。

命令行（会自动找 CLion 自带的 cmake）：

```powershell
.\scripts\build.ps1
.\scripts\build.ps1 -Preset lc-voltage-debug
```

若 `cmake` / `ninja` 已在 PATH：

```powershell
cmake --preset lc-voltage
cmake --build --preset lc-voltage
```

找不到编译器时，把工具链 `bin` 目录加入 PATH，或设置 `ARM_NONE_EABI_TOOLCHAIN_PATH`。

### Linux

```bash
cmake --preset lc-voltage
cmake --build --preset lc-voltage
```

### 烧录（OpenOCD）

探针为 **CMSIS-DAP + SWD**（与 YFOCV3_ST 相同：`adapter driver cmsis-dap`）。先整片擦除再写入。

```powershell
.\scripts\flash.ps1
.\scripts\flash.ps1 -BuildBeforeFlash
.\scripts\flash.ps1 -Preset lc-voltage-debug
```

OpenOCD 默认 `D:\OpenOCD-20240916-0.12.0`。换路径时传 `-OpenOcdExe` / `-OpenOcdScripts`。

```bash
openocd -s /path/to/openocd/scripts -f scripts/openocd-stm32g431.cfg \
  -c "init" -c "reset halt" -c "stm32l4x mass_erase 0" \
  -c "program build/lc-voltage/YFOCV3.elf verify" -c "reset run" -c "shutdown"
```

## 硬件

| 功能 | 引脚 |
|------|------|
| TIM1 PWM 高侧 | PA8 / PA9 / PA10 |
| TIM1 PWM 低侧 | PB13 / PB14 / PB15 |
| SPI1 编码器 | PA5/6/7，CS=PA4（`CFG_ENCODER_TYPE`：MT6701 SSI 或 KTH7812 SPI Mode3） |
| 母线电压 | PB0 / ADC1_IN15，`Vbus = Vadc × 31.3` |
| FDCAN1 | PA11 / PA12，Classic 1 Mbps |
| LED | PC13，高电平亮；正常运行按 ID 闪（十位长闪 + 个位短闪） |
| 按键 | PB3 上拉，按下接地。长按 2 s 改 ID，短按 +1，再长按保存并复位 |
| 时钟 | HSE 32 MHz → 170 MHz |

PWM 20 kHz 中心对齐；TIM6 4 kHz 运控。母线电压走 ADC1 规则组（软件触发，约 5 ms）；注入组空着，留给以后的 20 kHz 电流，运行中不换通道、不停 ADC。当前无电流采样。

## 上电校准

首次成功校准后写入 Flash 最后一页（`0x0801F800`，2 KB）。之后上电若记录有效（magic / CRC / 极对数范围合法）则直接加载，电机不再转一圈。

`flash.ps1` 整片擦除会清掉校准，烧录后第一次上电仍会走完整校准。运行中可用 CAN `START_ENCODER_CALIBRATION` 重校准并覆盖 Flash。

无有效记录时走与 CAN `START_ENCODER_CALIBRATION` **同一套** `Cali_RunCommand`：锁定、转圈估极对数/方向、探闭环方向、写 Flash，并按约 50 ms 发 `0x3C0` 进度（上电 Sequence=0）。有有效记录则只加载，不上报校准进度。

失败：LED 快闪，PWM 关闭。成功：LED 按 Motor ID 闪烁（例如 12 = 1 长 + 2 短），进入运控。位置坐标用编码器单圈机械角（不对零）；目标位置设为当时的实际位置，避免上电窜动。MT6701 是单圈，掉电后不保留多圈计数。

`CFG_POLE_PAIRS` 只是校准前的初值；真正使用的极对数以本次测量（或 Flash 记录）为准。锁不住或转角不够会报 `cali: pp bad`。

校准电压/时长也在 `config.h`：过大发热，过小可能锁不住或判失败。拆过电机或编码器后必须重校准，否则会沿用旧零偏。

## 运控

4 kHz 按当前 Control Mode：

```text
MOTION:   t_ref = Kd*(v_set - v_act) + Kp*(p_set - p_act) + Voltage_ff
VELOCITY: t_ref = Kp_v*e_v + Ki_v*∫e_v dt          （积分钳位到 ±V_LIMIT）
POSITION: v_ref = Kp_p*e_p + Ki_p*∫e_p dt - Kd_p*v_act
          v_ref = clamp(v_ref, ±vmax)
          内环同 VELOCITY PI（内环饱和时冻结外环积分）
t_ref = clamp(t_ref, -V_LIMIT, +V_LIMIT)
```

`CFG_V_LIMIT=0.4` 是标幺幅值（1.0 = PWM 满幅），不是伏特。线电压大约 `0.4 × Vbus`。20 kHz 对 Vd/Vq 再限变化率 `CFG_V_SLEW_PU_S`。

20 kHz：`q = closed_loop_dir * t_ref`，`d = 0`，逆 Park + SVPWM。

速度：校准后电角度 20 kHz 差分，再过 500 Hz 二阶 Butterworth（`CFG_VEL_LPF_HZ`），输出机械角速度。CRC 失败的帧丢弃，按上次速度外推电角度。

上电默认 `MOTION_MODE`。校准或加载后 PWM 打开，目标位置设为当前角。`SET_CONTROL_MODE` 切换后积分清零，需再发对应模式的实时命令。反馈在有效实时命令之后回 `0x300+ID`。

## CAN 协议

Motor CAN Protocol V1.0，Classic CAN 1 Mbps，**小端**，Motor ID = 1~63。出厂默认 `CFG_NODE_ID`（1）；成功改 ID 后写入校准同一 Flash 页，上电加载。`SET_NODE_ID`（`0x200+ID`，Byte0=`0x07`，Byte2=新 ID）成功则旧 ID 回 ACK 后复位。上位机收到 ACK 后自动改用新 ID。

| CAN ID | 方向 | 本固件 |
|------|------|------|
| `0x100+ID` | 主机→电机 | Motion Command |
| `0x140+ID` | 主机→电机 | Velocity Mode Command |
| `0x180+ID` | 主机→电机 | Control Gains（MOTION / VELOCITY / POSITION 三组） |
| `0x180+ID` | 电机→主机 | GET_GAINS 成功应答（与 SET 同布局） |
| `0x1C0+ID` | 主机→电机 | Position Mode Command |
| `0x200+ID` | 主机→电机 | Management：SET_ZERO / CLEAR_FAULT / START_CALI / GET_STATUS / SET_CONTROL_MODE / SET_NODE_ID / SAVE_USER_PARAMS / GET_GAINS |
| `0x280+ID` | 电机→主机 | Command ACK |
| `0x300+ID` | 电机→主机 | Motion Feedback |
| `0x380+ID` | 电机→主机 | Status Response（母线电压实测；温度填 0） |
| `0x3C0+ID` | 电机→主机 | Encoder Calibration Report（约 50 ms） |

Motion `0x100+ID`：

| 字节 | 含义 | 编码 |
|------|------|------|
| 0-3 | 目标位置 | `int32`，`0.0001 rad/LSB` |
| 4-5 | 目标速度 | `int16`，`0.1 rad/s/LSB` |
| 6-7 | Voltage FF | `int16`，`raw/32767` → −1~1 |

Gains `0x180+ID`：Byte0=Control Mode，Byte1=Sequence。MOTION：Kp `0.01 pu/rad`，Ki=0，Kd `0.001 pu·s/rad`。VELOCITY：Kp `0.001 pu/(rad/s)`，Ki `0.001 pu/rad`，Kd=0。POSITION：外环 Kp `0.01 s⁻¹`，Ki `0.001 s⁻²`，Kd `0.001`。位置内环使用 VELOCITY 那组 PI。SET 只改 RAM。`GET_GAINS`（`0x200`，Byte0=`0x21`，Byte2=mode）成功时电机在 `0x180` 回同样布局，读的是当前 RAM。`SAVE_USER_PARAMS`（`0x200`，Byte0=`0x08`）把速度 Kp/Ki 和位置 Kp/Ki/Kd 写入校准同一 Flash 记录；不存 Motion 增益、模式、目标或零点。上电若 Flash 有用户增益则加载 Vel/Pos，Motion 保持 0。

`SET_CONTROL_MODE` 可在 RUNNING 下切换三模式；切换后清积分，电压为 0 直到收到新模式的有效实时命令。上电默认 MOTION。

`SET_NODE_ID`：Byte2=1~63。写 Flash 保留校准，ACK Byte6=新 ID，约 20 ms 后复位。也可 PB3 长按进入改 ID。忘了 ID 时看灯闪或用按键。

`START_ENCODER_CALIBRATION`（`0x200+ID`，Byte0=`0x05`）：因本固件无 ENABLE/DISABLE，**RUNNING 下可直接启动**（协议原文要求 DISABLED）。先 ACK `OK` 且 State=`CALIBRATING`，随后 `0x3C0+ID` 约 50 ms 上报进度。成功后回到 **RUNNING**（协议原文是 DISABLED）。失败进入 FAULT + `CALIBRATION_FAULT`，需 `CLEAR_FAULT` 后再发校准。上电无 NV 时同样走这条流程并上报，Sequence=0，没有 Command ACK。校准过程阻塞主循环数秒，期间实时命令会被丢弃。

反馈 `0x300+ID`：

| 字节 | 含义 | 编码 |
|------|------|------|
| 0-3 | 当前位置 | `int32`，`0.0001 rad/LSB`，连续多圈 |
| 4-5 | 当前速度 | `int16`，`0.1 rad/s/LSB` |
| 6-7 | 电压指令 | `int16`，`0.001 pu/LSB`（协议的 Torque 字段；本固件无转矩传感器） |

## 上位机

先进入 `host/` 再建虚拟环境。GUI 按系统列出常用接口，通道可下拉或手填，点「扫描」探测已接适配器。

Windows 常用 PEAK PCAN（需先装 [PCAN-Basic](https://www.peak-system.com/PCAN-Basic.239.0.html)）。Linux 用 SocketCAN。macOS 没有 SocketCAN，一般用 slcan / PCAN。接口与通道也可手填，例如 `slcan` + `COM3` 或 `/dev/ttyACM0`。

Windows（PowerShell）：

```powershell
cd host
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python servo_gui.py
```

GUI（PySide6 + [pyqtgraph](https://github.com/pyqtgraph/pyqtgraph)）可发送协议内全部命令，并在下方示波窗口画实时反馈：位置 / 速度 / 电压。横轴可选时间 t（无新数据也会滚动）或按帧；通道可勾选，窗口长度可调。固件只在有效实时命令后回 `0x300`，要快刷曲线请打开对应模式的循环发送。ENABLE/DISABLE 仍会返回 `INVALID_COMMAND`。校准前请先停止实时循环。

Windows 激活 venv 后请用 `python`，不要用 `python3`：系统里的 `python3` 往往是 Microsoft Store 占位程序，会立刻退出且不报错。

```powershell
python servo_host.py --interface pcan --channel PCAN_USBBUS1 --id 1 --pos 3 --vel 0 --kp 20 --kd 0.2
python servo_host.py --interface pcan --channel PCAN_USBBUS1 --id 1 --listen
```

第二块 USB 适配器一般是 `PCAN_USBBUS2`。

Linux：

```bash
cd host
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 servo_gui.py
python3 servo_host.py --list-can
python3 servo_host.py --interface socketcan --channel can0 --id 1 --pos 0 --vel 10 --kp 0 --kd 0.016
python3 servo_host.py --interface socketcan --channel can0 --id 1 --listen
```

SocketCAN 示例：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

macOS / slcan：`--interface slcan --channel /dev/cu.usbserial` 或 Windows `COM3`。`python servo_host.py --list-can` 可列出当前接口下探测到的通道。

## 调试

SEGGER RTT（SWD）。LC-ESC 的 PB3 是按键（改 CAN ID），不要用 SWO。主循环约 200 ms 打印一次 `p/v/t/kp/kd`（整数毫弧度等）。

## 注意

电压模式没有软件过流保护，依赖功率级硬件。编码器由 `config.h` 的 `CFG_ENCODER_TYPE` 选择：默认 `CFG_ENCODER_MT6701`（SSI 14 bit）；换成 KTH7812 时设为 `CFG_ENCODER_KTH7812`（SPI Mode3，16 bit，无 CRC 的 -N 型号）。换芯片后必须重校准。
