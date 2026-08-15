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
| SPI1 编码器 | PA5/6/7，CS=PA4 |
| FDCAN1 | PA11 / PA12，Classic 1 Mbps |
| LED | PC13，高电平亮 |
| 时钟 | HSE 32 MHz → 170 MHz |

PWM 20 kHz 中心对齐；TIM6 4 kHz 运控；无电流采样。

## 上电校准

首次成功校准后写入 Flash 最后一页（`0x0801F800`，2 KB）。之后上电若记录有效（magic / CRC / 极对数范围合法）则直接加载，电机不再转一圈。

`flash.ps1` 整片擦除会清掉校准，烧录后第一次上电仍会走完整校准。运行中可用 CAN `START_ENCODER_CALIBRATION` 重校准并覆盖 Flash。

无有效记录时走与 CAN `START_ENCODER_CALIBRATION` **同一套** `Cali_RunCommand`：锁定、转圈估极对数/方向、探闭环方向、写 Flash，并按约 50 ms 发 `0x3C0` 进度（上电 Sequence=0）。有有效记录则只加载，不上报校准进度。

失败：LED 快闪，PWM 关闭。成功：LED 常亮，进入运控。位置坐标用编码器单圈机械角（不对零）；目标位置设为当时的实际位置，避免上电窜动。MT6701 是单圈，掉电后不保留多圈计数。

`CFG_POLE_PAIRS` 只是校准前的初值；真正使用的极对数以本次测量（或 Flash 记录）为准。锁不住或转角不够会报 `cali: pp bad`。

校准电压/时长也在 `config.h`：过大发热，过小可能锁不住或判失败。拆过电机或编码器后必须重校准，否则会沿用旧零偏。

## 运控

4 kHz：

```text
t_ref = Kd*(v_set - v_act) + Kp*(p_set - p_act) + Voltage_ff
t_ref = clamp(t_ref, -V_LIMIT, +V_LIMIT)
```

`CFG_V_LIMIT=0.4` 是标幺幅值（1.0 = PWM 满幅），不是伏特。线电压大约 `0.4 × Vbus`。20 kHz 对 Vd/Vq 再限变化率 `CFG_V_SLEW_PU_S`。

20 kHz：`q = closed_loop_dir * t_ref`，`d = 0`，逆 Park + SVPWM。

速度：二阶 Type-2 PLL（`CFG_VEL_PLL_HZ`）。CRC 失败的帧丢弃，PLL 按上次速度外推。

上电校准或加载参数后直接进入运控：PWM 打开，目标位置设为当前角。上位机发 `SET_GAINS` 后即可发 Motion Command。反馈只在有效 Motion Command 之后回 `0x300+ID`。

## CAN 协议

Motor CAN Protocol V1.0，Classic CAN 1 Mbps，**小端**，Motor ID = 1~63（默认 1）。

| CAN ID | 方向 | 本固件 |
|------|------|------|
| `0x100+ID` | 主机→电机 | Motion Command（已接入） |
| `0x180+ID` | 主机→电机 | Control Gains，仅 `MOTION_MODE`（已接入） |
| `0x200+ID` | 主机→电机 | Management：SET_ZERO / CLEAR_FAULT / START_CALI / GET_STATUS / SET_CONTROL_MODE(motion) |
| `0x140+ID` / `0x1C0+ID` | 主机→电机 | 速度/位置模式：收包但不执行 |
| `0x280+ID` | 电机→主机 | Command ACK |
| `0x300+ID` | 电机→主机 | Motion Feedback |
| `0x380+ID` | 电机→主机 | Status Response（母线电压/温度填 0） |
| `0x3C0+ID` | 电机→主机 | Encoder Calibration Report（约 50 ms） |

Motion `0x100+ID`：

| 字节 | 含义 | 编码 |
|------|------|------|
| 0-3 | 目标位置 | `int32`，`0.0001 rad/LSB` |
| 4-5 | 目标速度 | `int16`，`0.1 rad/s/LSB` |
| 6-7 | Voltage FF | `uint16`，`raw/65535` → 0~1 |

Gains `0x180+ID`：Byte0=`0x00`（MOTION），Byte1=Sequence，Byte2-3=Kp（`0.01 pu/rad`），Byte4-5=Ki 必须为 0，Byte6-7=Kd（`0.001 pu·s/rad`）。应答 `0x280`，Command=`0x20`。

`START_ENCODER_CALIBRATION`（`0x200+ID`，Byte0=`0x05`）：因本固件无 ENABLE/DISABLE，**RUNNING 下可直接启动**（协议原文要求 DISABLED）。先 ACK `OK` 且 State=`CALIBRATING`，随后 `0x3C0+ID` 约 50 ms 上报进度。成功后回到 **RUNNING**（协议原文是 DISABLED）。失败进入 FAULT + `CALIBRATION_FAULT`，需 `CLEAR_FAULT` 后再发校准。上电无 NV 时同样走这条流程并上报，Sequence=0，没有 Command ACK。校准过程阻塞主循环数秒，期间 Motion 会被丢弃。

反馈 `0x300+ID`：

| 字节 | 含义 | 编码 |
|------|------|------|
| 0-3 | 当前位置 | `int32`，`0.0001 rad/LSB`，连续多圈 |
| 4-5 | 当前速度 | `int16`，`0.1 rad/s/LSB` |
| 6-7 | 电压指令 | `int16`，`0.001 pu/LSB`（协议的 Torque 字段；本固件无转矩传感器） |

## 上位机

先进入 `host/` 再建虚拟环境。Windows 用 PEAK PCAN（需先装 [PCAN-Basic](https://www.peak-system.com/PCAN-Basic.239.0.html)），Linux 用 SocketCAN。

Windows（PowerShell）：

```powershell
cd host
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python servo_gui.py
```

GUI 可发送协议内全部命令。Motion 支持单次或按设定频率循环发送，并刷新位置/速度/电压反馈。当前固件已接入运控、MOTION Gains、SET_ZERO、GET_STATUS、START_CALI；速度/位置模式与 ENABLE/DISABLE 仍会失败或被忽略。校准前请先停止 Motion 循环。

Windows 激活 venv 后请用 `python`，不要用 `python3`：系统里的 `python3` 往往是 Microsoft Store 占位程序，会立刻退出且不报错。

```powershell
python servo_host.py --interface pcan --channel PCAN_USBBUS1 --id 1 --pos 3 --vel 0 --kp 20 --kd 0.2
python servo_host.py --interface pcan --channel PCAN_USBBUS1 --id 1 --listen
```

第二块 USB 适配器一般是 `PCAN_USBBUS2`。

Linux / macOS：

```bash
cd host
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 servo_gui.py
python3 servo_host.py --interface socketcan --channel can0 --id 1 --pos 0 --vel 10 --kp 0 --kd 0.016
python3 servo_host.py --interface socketcan --channel can0 --id 1 --listen
```

SocketCAN 示例：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

slcan：`--interface slcan --channel /dev/ttyACM0`。

## 调试

SEGGER RTT（SWD）。LC-ESC 的 PB3 是按键，不要用 SWO。主循环约 200 ms 打印一次 `p/v/t/kp/kd`（整数毫弧度等）。

## 注意

电压模式没有软件过流保护，依赖功率级硬件。编码器默认 MT6701 SSI。
