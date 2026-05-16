# UFB - Universal Filament Buffer base on BMCU

这是一个给 BMCU 板子用的独立耗材缓冲器固件。

它不和打印机通信，不跑 AMS 协议，也不要求接到 Bambu 打印机上。这个仓库的目标只有一个：把这块板子改成一个离线的、可独立工作的耗材缓冲器控制板。

## 当前逻辑

这版固件现在的行为是：

- 平时按独立耗材缓冲器工作。
- 第一个微动触发时，启动一次自动进料。
- 自动进料一旦开始，会先完整执行完，再回到普通缓冲器逻辑。
- 正常缓冲器补料和自动进料使用同样的前进速度。
- 高阻力版只提高电机输出，不改变触发灵敏性和整体逻辑。
- 运行状态由主控固件在内部更新时写入缓存，不主动持续上报。
- ESP32-C3 Wi-Fi 桥接固件每 `500ms` 通过 UART 发送一次 `STATE`，收到状态后缓存起来，Web API 只返回 Wi-Fi 侧缓存。

这版不做的事情：

- 不和打印机通信
- 不依赖 AMS 总线
- 不暴露用户侧“手动长按进退料”模式

## Wi-Fi 桥接固件

`wifi/wifi.ino` 是 ESP32-C3 用的 Wi-Fi 桥接固件。它提供一个简单 Web 页面和 HTTP API，通过 UART 控制 UFB 主控。

默认硬件连接：

| ESP32-C3 | 用途 |
|---|---|
| `A3` | 接 UFB 主控 TX |
| `A4` | 接 UFB 主控 RX |
| `GPIO8` | 板载蓝色 LED，平时常亮，执行动作时闪烁 |
| `3.3V/GND` | 由 UFB 供电 |

USB 串口命令：

```text
WIFI ssid/password
STATE
INPUT 0
OUTPUT 0
```

HTTP API：

| 路径 | 方法 | 含义 |
|---|---|---|
| `/` | `GET` | Web 控制页面 |
| `/api/state` | `GET` | 返回 Wi-Fi 侧缓存状态 |
| `/api/action?cmd=input&ch=0` | `POST` | 指定通道进料 |
| `/api/action?cmd=output&ch=0` | `POST` | 指定通道退料 |

`/api/state` 返回紧凑文本，不返回 JSON：

```text
<busy> <stale> <compact-state>
```

`compact-state` 是 57 个十六进制字符：

- 第 1 个字符：当前 loaded 通道，`0..3` 表示通道号，`F` 表示未 loaded
- 后续每个通道 14 个字符，共 4 个通道
- 每个通道包含 7 字节：`flags + status RGB + online RGB`
- `flags`：bit0=`inserted`，bit1..2=`buffer mode`，bit3=`sw1`，bit4=`sw2`

Wi-Fi 固件的 UART 接收缓冲、状态缓存和 API 响应都使用固定缓冲区，避免在 500ms 轮询路径上频繁动态分配。

## LED 状态

每个通道前面有两个 LED：

- `ONLINE`：只反映微动 `KS` 状态
- `STU`：只反映电机动作状态

### `ONLINE` LED

| LED 显示 | `KS` | 含义 |
|---|---:|---|
| 灭 | `0` | 两个微动都没触发 |
| 蓝 | `2` | 只有第一个微动触发 |
| 白 | `1` | 第一个和第二个微动都触发 |
| 红闪 | `3` | 只有第二个微动触发 |

### `KS` 二进制组合

这里把 `KS` 按两个微动的组合来理解。

- `S1`：第一个微动
- `S2`：第二个微动
- 组合规则：`KS = (S1 << 1) | S2`

| `S1` | `S2` | 二进制 | `KS` | 含义 |
|---:|---:|---|---:|---|
| `0` | `0` | `0b00` | `0` | 两个微动都没触发 |
| `0` | `1` | `0b01` | `1` | 第一个和第二个微动都触发 |
| `1` | `0` | `0b10` | `2` | 只有第一个微动触发 |
| `1` | `1` | `0b11` | `3` | 只有第二个微动触发 |

上表里的 `S1/S2` 不是“微动电平是否为高”，而是当前固件里经过阈值映射后的组合语义：

- `KS=0`：空
- `KS=2`：首微动触发，常用于自动吸入起点
- `KS=1`：loaded
- `KS=3`：第二微动单独触发

### `STU` LED

| LED 显示 | 含义 |
|---|---|
| 灭 | 电机空闲/停止 |
| 绿 | 前送、自动吸入、送丝 |
| 青蓝 | on-use 压力控制、手动前送 |
| 紫闪 | 回抽、退料、手动回退 |
| 红 | 卡料、故障、停止锁存 |

## 先看你该用哪个版本

### `filament_buffer_bmcu`

标准版，适合普通 PTFE 长度和阻力。

### `filament_buffer_bmcu_high_force`

高阻力版，适合更长、更弯、摩擦更大的送丝路径。

这个版本现在只提高电机输出，不改灵敏性。和标准版相比，关键差异是：

```cpp
// 标准版
autoload/feed pwm = 850
retract pwm       = 850

// 高阻力版
autoload/feed pwm = 960
retract pwm       = 900
```

## 你需要准备什么

开始前需要：

- `git`
- `python3`
- `platformio`
- 本地可执行的 `wchisp`

安装 PlatformIO：

```bash
python3 -m pip install -U platformio
```

确认安装：

```bash
pio --version
```

## 如何编译

在仓库根目录执行。

编译标准版：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu
```

编译高阻力版：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu_high_force
```

生成产物：

```text
.pio/build/filament_buffer_bmcu/firmware.bin
.pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

## 如何刷写

先确认设备处于 ISP 模式并能被识别：

```bash
./wchisp info
```

刷标准版：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu/firmware.bin
```

刷高阻力版：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

刷写成功后通常会看到：

```text
Verify OK
Device reset
```

刷 ESP32-C3 Wi-Fi 桥接固件：

```bash
arduino-cli compile --upload -p /dev/cu.usbmodem201201 --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc wifi
```

## 仓库结构

```text
src/
  app/
    main.cpp
  control/
    buffer_constants.h
    hall_calibration.cpp
    hall_calibration.h
    motion_control.cpp
    motion_control.h
  hardware/
    adc_dma.cpp
    adc_dma.h
    as5600_multi_soft_i2c.cpp
    as5600_multi_soft_i2c.h
    ws2812.cpp
    ws2812.h
  model/
    filament_state.cpp
    filament_state.h
  platform/
    debug_log.cpp
    debug_log.h
    runtime_api.h
    hal/
      irq_wch.h
      time_hw.c
      time_hw.h
  storage/
    nvm_storage.cpp
    nvm_storage.h
reference/
  original_bmcu/
```

最关键的文件：

- `src/app/main.cpp`：启动流程和主循环入口
- `src/control/motion_control.cpp`：缓冲器主逻辑、电机控制、自动进料
- `src/control/buffer_constants.h`：集中管理当前固件里的行为参数和阈值
- `src/control/hall_calibration.cpp`：上电校准逻辑

## `reference/original_bmcu` 是什么

这是原始上游项目的参考副本，只用于对照原始实现，不参与当前固件编译。

可以把它理解成：

- 当前仓库是你真正要刷进板子的独立版本
- `reference/original_bmcu` 只是查资料用的参考代码
