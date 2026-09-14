/**
 * @file PCA9546A.h
 * @brief PCA9546A 4 通道 I2C 总线切换器驱动（基于 ESP-IDF v5.x i2c_master 新 API）
 *
 * PCA9546A 是一个通过 I2C 控制的 4 通道总线多路选择器（多路复用）：
 *   - 通过写控制寄存器选择挂载在 CH0~CH3 上的某一路 I2C 从设备总线
 *   - 同一时刻只能选中一个通道（或全不选）
 *   - 从机地址由 A2/A1/A0 引脚决定：0x70 ~ 0x77（默认全接地 = 0x70）
 *
 * 控制寄存器（写）：
 *   bit0 ~ bit3  选择 CH0 ~ CH3（写 1 使能对应通道）
 *   bit4         INT 中断状态（只读，写 0）
 *   bit7 ~ bit5  保留（写 0）
 *
 * 读：返回控制寄存器当前值（含 INT 状态位）。
 *
 * 典型用法：
 *   pca9546a_handle_t mux;
 *   pca9546a_init(18, 19, PCA9546A_ADDR(0,0,0), 100000, &mux);
 *   pca9546a_select(&mux, PCA9546A_CH0);
 *   // 通过 pca9546a_get_bus(&mux) 挂载 CH0 上的下游设备...
 *   pca9546a_deselect(&mux);
 */
#ifndef PCA9546A_H
#define PCA9546A_H

#include "driver/i2c_master.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================
// 地址与寄存器定义
// ====================================================================
#define PCA9546A_ADDR_BASE        0x70                        /*!< A2A1A0=000 时的 7 位从机地址 */
#define PCA9546A_ADDR(a2, a1, a0) (PCA9546A_ADDR_BASE + ((a2) << 2) + ((a1) << 1) + (a0)) /*!< 由 A2/A1/A0 组合生成地址 */

#define PCA9546A_CTRL_CH0         (1u << 0)                   /*!< 控制寄存器：选择通道 0 */
#define PCA9546A_CTRL_CH1         (1u << 1)                   /*!< 控制寄存器：选择通道 1 */
#define PCA9546A_CTRL_CH2         (1u << 2)                   /*!< 控制寄存器：选择通道 2 */
#define PCA9546A_CTRL_CH3         (1u << 3)                   /*!< 控制寄存器：选择通道 3 */
#define PCA9546A_CTRL_INT         (1u << 4)                   /*!< 控制寄存器：INT 中断状态（只读） */

// ====================================================================
// 类型定义
// ====================================================================
typedef enum {
    PCA9546A_CH_NONE = -1,   /*!< 未选中任何通道 */
    PCA9546A_CH0     = 0,
    PCA9546A_CH1     = 1,
    PCA9546A_CH2     = 2,
    PCA9546A_CH3     = 3,
} pca9546a_channel_t;

typedef struct {
    i2c_master_bus_handle_t bus;      /*!< I2C master 总线句柄（供下游设备通过 pca9546a_get_bus 挂载） */
    i2c_master_dev_handle_t dev;      /*!< PCA9546A 自身设备句柄 */
    pca9546a_channel_t      current;  /*!< 当前选中通道 */
    int                     reset_io; /*!< RESET 复位引脚（GPIO_NUM_NC 表示未连接） */
} pca9546a_handle_t;

// ====================================================================
// API
// ====================================================================

/**
 * @brief 初始化 I2C master 总线并添加 PCA9546A 设备
 *
 * 若 reset_io 有效（!= GPIO_NUM_NC），先对该引脚做一次**复位脉冲**：
 * 拉低（进入复位）→ 延时 → 拉高（释放复位），从而清空所有通道选择
 * 并释放可能被下游设备阻塞的 I2C 总线。随后创建 I2C 主总线（I2C_NUM_0）
 * 并把 PCA9546A 自身挂到总线上，默认关闭所有通道（deselect）。
 *
 * @param sda_io        SDA 引脚
 * @param scl_io        SCL 引脚
 * @param reset_io      RESET 复位引脚（低有效；GPIO_NUM_NC 表示未连接）
 * @param addr          PCA9546A 从机地址（7 位，如 PCA9546A_ADDR(0,0,0) = 0x70）
 * @param scl_speed_hz  SCL 时钟频率（如 100000 = 100kHz）
 * @param out           输出句柄
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t pca9546a_init(int sda_io, int scl_io, int reset_io, uint8_t addr,
                        uint32_t scl_speed_hz, pca9546a_handle_t *out);

/**
 * @brief 对 PCA9546A 执行复位（拉低 → 延时 → 拉高）
 *
 * 复位会清空所有通道选择并释放被阻塞的 I2C 总线。
 * 仅当 reset_io 有效时可用。
 *
 * @param h 句柄
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t pca9546a_reset(pca9546a_handle_t *h);

/**
 * @brief 选择单个通道（写控制寄存器 1<<ch）
 *
 * 同一时刻只能选中一个通道；先前的通道会被自动取消。
 *
 * @param h  句柄
 * @param ch 目标通道（PCA9546A_CH0 ~ CH3）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t pca9546a_select(pca9546a_handle_t *h, pca9546a_channel_t ch);

/**
 * @brief 关闭所有通道（写控制寄存器 0x00）
 *
 * @param h 句柄
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t pca9546a_deselect(pca9546a_handle_t *h);

/**
 * @brief 读取控制寄存器当前值（含 INT 中断状态位）
 *
 * @param h    句柄
 * @param ctrl 输出寄存器值
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t pca9546a_read_ctrl(pca9546a_handle_t *h, uint8_t *ctrl);

/**
 * @brief 获取当前选中的通道
 *
 * @param h 句柄
 * @return 当前通道；未选中返回 PCA9546A_CH_NONE
 */
pca9546a_channel_t pca9546a_get_channel(const pca9546a_handle_t *h);

/**
 * @brief 获取 I2C master 总线句柄
 *
 * 用于在选定通道后，把下游 I2C 设备（如传感器）挂载到该总线上：
 *   i2c_master_bus_add_device(pca9546a_get_bus(&mux), &dev_cfg, &dev_handle);
 *
 * @param h 句柄
 * @return 总线句柄；句柄无效返回 NULL
 */
i2c_master_bus_handle_t pca9546a_get_bus(pca9546a_handle_t *h);

#ifdef __cplusplus
}
#endif

#endif // PCA9546A_H
