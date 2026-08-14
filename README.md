# YFOCV3 最小电压伺服（LC-ESC）

面向 **LC-ESC / STM32G431CBT6 + FD6288 + MT6701** 的最小电压模式伺服固件。上电校准后进入运控，用 CAN 收 MIT 风格控制帧、回传状态。

## 构建

需要 `arm-none-eabi-gcc` 与 Ninja。

```bash
cd YFOCV3
cmake --preset lc-voltage
cmake --build build/lc-voltage
```

产物：`build/lc-voltage/YFOCV3.elf`、`YFOCV3.bin`。

烧录示例（按你的探针改）：

```bash
st-flash write build/lc-voltage/YFOCV3.bin 0x08000000
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

启动后自动：

1. D 轴电压锁定
2. 电角度正转，估计编码器方向
3. 由锁定位置计算电角度零偏
4. 施加小 `+Vq`，估计闭环方向

失败：LED 快闪，PWM 关闭。成功：LED 常亮，进入运控。当前位置定义为位置 0。

默认极对数 `CFG_POLE_PAIRS=7`，在 `App/config.h` 修改。

校准电压/时长也在 `config.h`：过大发热，过小可能锁不住或判失败。

## 运控

4 kHz：

```text
t_ref = Kd*(v_set - v_act) + Kp*(p_set - p_act)    # t_ff = 0
t_ref = clamp(t_ref, -V_LIMIT, +V_LIMIT)
```

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
| 0-1 | 当前角度 | -π ~ π |
| 2-3 | 当前角速度 | -100 ~ 100 rad/s |
| 4-5 | t_ref（电压） | -1 ~ 1 |
| 6-7 | 圈数 | 0 ~ 65535（uint16 回绕） |

## 上位机

```bash
pip install -r host/requirements.txt
python3 host/servo_host.py --channel can0 --id 1 --pos 0 --vel 0 --kp 20 --kd 0.5
python3 host/servo_host.py --channel can0 --id 1 --listen
```

SocketCAN 示例：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

slcan：`--interface slcan --channel /dev/ttyACM0`。

## 调试

SEGGER RTT（SWD）。LC-ESC 的 PB3 是按键，不能用 SWO。主循环约 200 ms 打印一次 `p/v/t/kp/kd`（整数毫弧度等）。

## 注意

电压模式没有软件过流保护，依赖功率级硬件。编码器默认 MT6701 SSI。
