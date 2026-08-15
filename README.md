# YFOCV3 最小电压伺服（LC-ESC）

面向 **LC-ESC / STM32G431CBT6 + FD6288 + MT6701** 的最小电压模式伺服固件。上电校准后进入运控，用 CAN 收 MIT 风格控制帧、回传状态。

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

`flash.ps1` 整片擦除会清掉校准，烧录后第一次上电仍会走完整校准。要重校准：再烧录一次。

无有效记录时自动：

1. D 轴电压锁定
2. 电角度正转若干圈，展开机械角，估计编码器方向和极对数
3. 由锁定位置计算电角度零偏
4. 施加小 `+Vq`，估计闭环方向
5. 关 PWM 输出、停 TIM1/TIM6/CAN 中断后写入 Flash（保留 SysTick）

失败：LED 快闪，PWM 关闭。成功：LED 常亮，进入运控。位置坐标用编码器单圈机械角（不对零）；目标位置设为当时的实际位置，避免上电窜动。MT6701 是单圈，掉电后不保留多圈计数。

`CFG_POLE_PAIRS` 只是校准前的初值；真正使用的极对数以本次测量（或 Flash 记录）为准。锁不住或转角不够会报 `cali: pp bad`。

校准电压/时长也在 `config.h`：过大发热，过小可能锁不住或判失败。拆过电机或编码器后必须重校准（烧录擦除），否则会沿用旧零偏。

## 运控

4 kHz：

```text
t_ref = Kd*(v_set - v_act) + Kp*wrap_pi(p_set - p_act)
t_ref = clamp(t_ref, -V_LIMIT, +V_LIMIT)
```

`CFG_V_LIMIT=0.5` 是标幺幅值（1.0 = PWM 满幅），不是伏特。线电压大约 `0.5 × Vbus`。20 kHz 对 Vd/Vq 再限变化率 `CFG_V_SLEW_PU_S`（默认 40 pu/s，0→0.5 约 12.5 ms）。

20 kHz：`q = closed_loop_dir * t_ref`，`d = 0`，逆 Park + SVPWM。

速度：编码器差分 + 一阶低通（默认 80 Hz）。CRC 失败的帧丢弃，保持上次角度。

上电后 Kp/Kd/setpoint 为 0，电机无力矩，直到上位机发控制帧。反馈帧约 200 Hz。

## CAN 协议

默认节点 ID = 1 → 控制 `0x101`，反馈 `0x201`。8 字节，**大端 u16**，线性映射：

`x = min + raw * (max-min) / 65535`

控制 `0x100+id`：

| 字节 | 含义 | 范围 |
|------|------|------|
| 0-1 | 目标角度 | -4π ~ 4π |
| 2-3 | 目标角速度 | -100 ~ 100 rad/s |
| 4-5 | Kp | 0 ~ 500 |
| 6-7 | Kd | 0 ~ 5 |

反馈 `0x200+id`：

| 字节 | 含义 | 范围 |
|------|------|------|
| 0-1 | 当前角度 | -4π ~ 4π（与指令同一映射，不再折到 ±π） |
| 2-3 | 当前角速度 | -100 ~ 100 rad/s |
| 4-5 | t_ref（电压） | -1 ~ 1 |
| 6-7 | 圈数 | 0 ~ 65535（uint16 回绕） |

## 上位机

先进入 `host/` 再建虚拟环境。Windows 用 PEAK PCAN（需先装 [PCAN-Basic](https://www.peak-system.com/PCAN-Basic.239.0.html)），Linux 用 SocketCAN。

Windows（PowerShell）：

```powershell
cd host
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python servo_host.py --interface pcan --channel PCAN_USBBUS1 --id 1 --pos 3 --vel 0 --kp 0.2 --kd 0
python servo_host.py --interface pcan --channel PCAN_USBBUS1 --id 1 --listen
```

第二块 USB 适配器一般是 `PCAN_USBBUS2`。

Linux / macOS：

```bash
cd host
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 servo_host.py --interface socketcan --channel can0 --id 1 --pos 0 --vel 10 --kp 0 --kd 0.01
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
