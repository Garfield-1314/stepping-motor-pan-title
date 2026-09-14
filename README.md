# Pan-Tilt 云台控制系统

基于 **ESP32-C3 + ESP-IDF v5.5.1** 的双轴步进电机云台控制平台，使用 **A4988** 步进电机驱动模块，通过 **LEDC 外设产生 STEP 方波**并配合 GPIO 中断软件计数实现精确运动控制。

## 硬件引脚配置

| 信号 | Pan 轴（水平旋转） | Tilt 轴（俯仰） |
|------|-------------------|----------------|
| STEP | GPIO4 | GPIO0 |
| DIR  | GPIO5 | GPIO1 |
| EN   | GPIO6（两轴共用） | ← 共用 |
| Pan 限位开关 | GPIO3 | — |
| Tilt 限位开关 | — | GPIO7 |
| PCA9546A SCL | GPIO19 | — |
| PCA9546A SDA | GPIO18 | — |
| PCA9546A RESET | GPIO10 | — |
| AS5600 编码器 Pan | PCA9546A CH0（I2C，0x36） | — |
| AS5600 编码器 Tilt | — | PCA9546A CH1（I2C，0x36） |

> **细分**：新 PCB 上 A4988 的 MS1/MS2/MS3 已**硬件上拉到 VCC**，固定 **1/16 细分**，固件不再控制细分引脚。原 MS1/MS2/MS3 引脚（GPIO19/GPIO18/GPIO10）已释放：GPIO19/GPIO18 用于 PCA9546A 的 I2C（SCL/SDA），GPIO10 用于 PCA9546A 的 RESET 复位（低有效）。

### 限位开关接线

常开机械微动开关：
- 开关一端接 GPIO，另一端接 GND
- 新 PCB 在 GPIO 与 VCC(3.3V) 之间已加**外部上拉电阻**（10kΩ），故**禁用 ESP32 内部上拉**（`pullup_enable = false`），避免内外上拉并联改变阈值
- 逻辑：未触发 = 高电平，触发（接通 GND）= 低电平（`active_low = true`）

### UART 串口

- TX = GPIO21，RX = GPIO20，115200 波特率

### PCA9546A I2C 扩展

- 4 通道 I2C 总线切换器（多路选择器），用于后续扩展多路 I2C 从设备
- SCL = GPIO19，SDA = GPIO18，RESET = GPIO10（低有效），地址 A2A1A0=000 → 0x70，100 kHz
- 驱动：`src/main/PCA9546A.c/.h`（基于 ESP-IDF v5.x `i2c_master` 新 API）
- 初始化时对 RESET 引脚执行一次**复位脉冲**（拉低 1ms → 拉高），清空通道选择并释放可能被下游设备阻塞的 I2C 总线；`pca9546a_reset()` 可随时再次复位
- RESET 引脚在 PCB 上已**外部上拉到 VCC**：MCU 不驱动时默认高电平（正常工作状态）；驱动用推挽输出做复位脉冲，外部上拉不影响其高低电平控制
- I2C 线使用外部上拉，`enable_internal_pullup = false`

### AS5600 磁性旋转编码器（双轴）

- 12 位磁性旋转位置传感器（0 ~ 4095，0.088°/LSB），**I2C 接口**（默认地址 0x36）
- **双编码器**：Pan=CH0、Tilt=CH1，两片同址 0x36，靠 PCA9546A 通道隔离避免地址冲突
- **安装在电机轴**（高速侧），经 4.5:1 减速比换算为云台输出轴角度
- 驱动：`src/main/AS5600.c/.h`（单片驱动）+ `src/main/encoder.c/.h`（双轴管理：通道切换 + 互斥锁）
- **互斥安全**：PCA9546A 同一时刻只选通一个通道，encoder 模块用互斥锁保护"通道切换 + 读取"，采样任务与 G-code 层并发安全
- **M114 反馈**：`M114` 返回 `MP: <pan> <tilt> ENC: <pan_deg> <tilt_deg> STEP: <pan_step_deg> <tilt_step_deg> RAW: <pr> <tr> MAG: <pm> <tm> ST: <ps> <ts>`（`ENC` 两轴输出轴**多圈连续**角度可为负；`STEP` 步进角度供融合对比；`RAW`/`MAG`/`ST` 两轴编码器诊断）
- **多圈累计 + 高频采样**：编码器由独立任务以 **5ms 周期**高频采样，通过回绕检测累计圈数输出连续角度（覆盖云台 -90°~90° 行程），采样间隔固定，高速也不易漏计
- **融合方案**：步进软件计数（高速可靠）+ 编码器（绝对校正）——`M114` 同时输出 `ENC`（编码器实际角度）与 `STEP`（步进命令角度），两者偏差可检测丢步/堵转

## 齿轮参数

- 步进电机：1.8° 步距角（200 步/圈）
- 齿轮比：电机 90 圈 = 大齿轮 20 圈（4.5:1）
- 大齿轮每圈 = 900 基础步
- 1/16 细分下大齿轮每圈 = **14400 微步**，每微步 = **0.025°**

## 项目结构

```
stepping-motor-pan-title/
├── .gitignore              # Git 忽略规则
├── .gitmodules             # esp32libraries 子模块引用
├── README.md               # 本文件
├── esp32libraries/         # ESP 库子模块（需手动初始化，见下方）
│   └── esp-idf/            # ESP-IDF 子模块（嵌套在 esp32libraries 中）
└── src/
    ├── CMakeLists.txt      # 项目根构建文件
    └── main/
        ├── CMakeLists.txt          # 组件构建文件
        ├── main.c                  # 应用入口（电机 + 限位开关 + PCA9546A 初始化）
        ├── stepper_ledc.c/.h       # LEDC 步进驱动（方波生成 + 软件计数）
        ├── limit_switch.c/.h       # 限位开关驱动（GPIO 输入轮询）
        ├── gcode_ledc.c/.h         # G-code 串口命令解析器（LEDC 版）
        ├── PCA9546A.c/.h           # PCA9546A 4 通道 I2C 切换器驱动
        ├── AS5600.c/.h             # AS5600 磁性旋转编码器驱动（I2C）
        ├── encoder.c/.h            # 双编码器（Pan/Tilt）管理：通道切换 + 互斥锁
        └── nvs_params.c/.h         # 系统参数存储（NVS 掉电保存）
```

> **注意**：`sdkconfig` 由 `idf.py menuconfig` 或 `idf.py build` 自动生成，已加入 `.gitignore`，不纳入版本控制。

## 获取 ESP-IDF 子模块

本项目以嵌套 Git 子模块方式引用 ESP-IDF，首次克隆后需手动拉取：

```bash
# 1. 拉取 esp32libraries 子模块
git submodule update --init

# 2. 进入 esp32libraries 目录，拉取 ESP-IDF 子模块（递归）
cd esp32libraries
git submodule update --init --recursive
cd ..
```

> **注意**：ESP-IDF 仓库较大（~2GB），`--recursive` 会同时拉取其内部子组件（工具链、示例等），耗时较长，请确保网络稳定。若中途失败可重复执行直到成功。

## 构建与烧录

```bash
# 安装 ESP-IDF 工具链（仅首次）
cd esp32libraries/esp-idf
./install.sh
cd ../..

# 导出环境变量（每次新开终端需执行）
. ./esp32libraries/esp-idf/export.sh

# 构建
idf.py -C src build

# 烧录（根据实际串口设备调整）
idf.py -C src -p /dev/ttyUSB0 flash

# 监视串口输出
idf.py -C src -p /dev/ttyUSB0 monitor
```

## 驱动模块 (stepper_ledc)

### 核心机制

- **LEDC 方波生成**：每电机使用独立 LEDC 定时器/通道（TIMER_0/CH0、TIMER_1/CH1），输出 STEP 方波
- **软件计数**：STEP 引脚配置为 `INPUT_OUTPUT` 模式，同引脚 GPIO 上升沿中断累计步数（ISR 纯 RAM 运算，避免 cache 崩溃）
- **12 位分辨率**：`LEDC_TIMER_12_BIT`（0~4095，50% 占空比 = 2048），在 40MHz XTAL 源钟下覆盖 **50 Hz ~ 9.8 kHz** 实测区间
- **速度安全钳制**：`STEP_SPEED_MIN_HZ=50`、`STEP_SPEED_MAX_HZ=9000`，超限请求显式钳制并打 `ESP_LOGW` 告警
- **双轴并行**：`move_both` / `move_both_accel` 单循环联合轮询，两轴同时启停；加减速在同一 1ms 周期内分别计算两轴梯形速度

### API 接口

```c
stepper_ledc_handle_t stepper_ledc_init(const stepper_ledc_config_t *config);

// 单轴
void stepper_ledc_move_step(stepper_ledc_handle_t motor, int steps, float speed_hz, bool forward);
void stepper_ledc_move_step_accel(stepper_ledc_handle_t motor, int steps,
                                  float speed_hz, float accel, bool forward);

// 双轴并行
void stepper_ledc_move_both(stepper_ledc_handle_t m1, int steps1, bool fwd1,
                            stepper_ledc_handle_t m2, int steps2, bool fwd2,
                            float speed_hz);
void stepper_ledc_move_both_accel(stepper_ledc_handle_t m1, int steps1, bool fwd1,
                                  stepper_ledc_handle_t m2, int steps2, bool fwd2,
                                  float speed_hz, float accel);

// 连续调速（非阻塞，DC 式）
void stepper_ledc_move_speed(stepper_ledc_handle_t motor, float speed_hz);

// 查询/复位/使能
long  stepper_ledc_get_steps(stepper_ledc_handle_t motor);
float stepper_ledc_get_angle(stepper_ledc_handle_t motor);
void  stepper_ledc_reset_position(stepper_ledc_handle_t motor);
void  stepper_ledc_set_enable(stepper_ledc_handle_t motor, bool enable);
```

### 加减速参数（梯形曲线）

- 起始/刹停速度：**200 步/秒**（`STEP_START_HZ`）
- 调频周期：**1 ms**（`dt = 0.001`，依赖 `FREERTOS_HZ=1000`）
- 减速判定：剩余步数 ≤ `cur² / (2·A)` 时开始减速

## 限位开关模块 (limit_switch)

GPIO 输入轮询的机械限位开关驱动。

```c
typedef struct {
    gpio_num_t gpio;       // GPIO 引脚
    bool pullup_enable;    // 是否启用内部上拉
    bool active_low;       // true=触发时读低电平, false=触发时读高电平
} limit_switch_config_t;

limit_switch_handle_t limit_switch_init(const limit_switch_config_t *config);
bool limit_switch_is_triggered(limit_switch_handle_t handle);
```

## 串口 G-code 控制 (gcode_ledc)

架构：`UART 层 → 行读取器 → 命令分发器（前缀匹配注册表）→ 处理器`

| 命令 | 功能 |
|------|------|
| `M17` | 使能所有步进电机 |
| `M18` | 停止脉冲并禁用所有步进电机 |
| `G28 [S<速度>]` | 限位开关回零（Pan 负向、Tilt 正向；15 秒/50000 步超时保护） |
| `M220 P<回退> T<回退>` | 设置 G28 回退步数（可查询，**掉电保存**） |
| `M221 P<最小角> Q<最大角> T<最小角> U<最大角>` | 设置/查询本次运行的编码器角度限位（**仅 RAM，不保存**；步数限位自动换算） |
| `PG1 P<绝对> T<绝对> S<速度>` | 匀速绝对定位（**双轴并行**） |
| `PG2 P<绝对> T<绝对> S<巡航> A<加速度>` | 加减速绝对定位（**双轴并行**） |
| `PGV P<速度> T<速度>` | 连续调速（非阻塞，两轴可独立控制，0=停；位置以编码器为准；成功不回 OK） |
| `M114` | 查询状态 → `MP: <pan> <tilt> ENC: <p_deg> <t_deg> STEP: <p_step> <t_step> RAW: <p_raw> <t_raw> MAG: <p_mag> <t_mag> ST: <p_st> <t_st>`（两轴角度+诊断） |

### 运动范围

| 轴 | 默认换算步数 | 默认编码器角度限制 |
|----|--------------|----------------|
| Pan | -3500 ~ 3500 | -87° ~ 87° |
| Tilt | -2500 ~ 2500 | -62° ~ 62° |

- `PG1/PG2` 的绝对目标必须同时满足步数和对应编码器角度限制；按 40 步/度换算，实际可用目标为 Pan -3480 ~ 3480、Tilt -2480 ~ 2480。
- `PGV` 运行时以编码器角度为准，向边界方向到达限制后自动停止；反向运动仍可启动。
- `M221` 设置后立即对整个固件生效，重启后恢复默认角度限位；返回值同时显示自动换算的步数限位。

### 速度范围（所有命令）

| 边界 | 值 |
|------|----|
| 合法范围 | **50 ~ 9000 步/秒** |
| 超限行为 | 钳制到边界 + `ESP_LOGW` 告警 |
| 加减速起始速度 | 200 步/秒 |
| 加速度默认值（PG2 未给 A） | 50000 步/秒² |

### G28 回零流程

1. 使能电机，停止当前运动
2. Pan 负向、Tilt 正向以 `S`（默认 2000）恒速移动
3. 每 20ms 轮询限位开关，触发即停止对应轴
4. 按 `M220` 回退步数以半速**双轴并行**脱离开关（参数来自 NVS，掉电保持）
5. 复位软件计数与绝对坐标为 0，编码器归零，输出 `OK`

> 若限位开关异常，G28 最多运行 15 秒或 50000 步后自动停止并返回错误。

### 示例

```
M17
M220 P3490 T2940      # 设置 G28 回退步数（掉电保存；首次上电默认即为该值）
G28 S2000
PG2 P2000 T1500 S3000 A50000
PGV P1000 T-800
PGV P0 T0
M114
```

## 系统参数存储 (NVS)

基于 ESP32-C3 片内 Flash 的 NVS（非易失存储）分区（`partitions_singleapp.csv` 已含 24KB nvs 分区），实现参数**掉电保存**。

| 参数 | NVS 键 | 默认值 | 说明 |
|------|--------|--------|------|
| Pan G28 回退步数 | `pan_backoff` | 3490 | `M220 P` |
| Tilt G28 回退步数 | `tilt_backoff` | 2940 | `M220 T` |

- 启动时 `main.c` 调用 `nvs_flash_init()` + `nvs_params_init()` 加载参数
- `M220` 设置后自动提交到 Flash；`gcode_ledc_start()` 从 NVS 加载回退参数
- 首次使用（无命名空间）自动采用默认值 **P3490 T2940**
- 新增参数：在 `nvs_params.c` 增加键名 + 默认值 + get/set 即可，`nvs_params_init/save` 统一读写

## 工程配置说明 (sdkconfig)

`sdkconfig` 由 ESP-IDF 构建系统自动生成（`idf.py build` 或 `idf.py menuconfig`），已加入 `.gitignore`。首次构建时自动创建，关键配置项：

- `CONFIG_FREERTOS_HZ=1000`：FreeRTOS tick = 1ms，保证轮询循环中 `vTaskDelay` 真实延时（原 100Hz 下 `pdMS_TO_TICKS(1)=0` 会忙等并饿死 IDLE 触发 task_wdt）
- `CONFIG_IDF_TARGET="esp32c3"`，CPU 160MHz、XTAL 40MHz
