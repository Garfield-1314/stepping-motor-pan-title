/**
 * @file AS5600.h
 * @brief AS5600 12-bit 磁性旋转位置传感器（I2C 接口）驱动
 *
 * 通过 PCA9546A I2C 扩展总线访问。使用 ESP-IDF v5.x i2c_master 新 API。
 * 读取时需先通过 PCA9546A 选中 AS5600 所在通道。
 *
 * 典型用法：
 *   pca9546a_select(&mux, PCA9546A_CH0);          // 选中通道
 *   as5600_init(pca9546a_get_bus(&mux), AS5600_ADDR_DEFAULT, 100000, &enc);
 *   uint16_t raw;
 *   as5600_read_raw(&enc, &raw);                  // 0 ~ 4095
 *   float deg;
 *   as5600_read_angle(&enc, &deg);                // 0 ~ 360°
 */
#ifndef AS5600_H
#define AS5600_H

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!< AS5600 7 位 I2C 地址（A1 引脚为 0 时；A1=1 则为 0x37） */
#define AS5600_ADDR_DEFAULT   0x36
#define AS5600_ADDR_ALT       0x37

/*!< 12 位分辨率（0 ~ 4095） */
#define AS5600_RAW_MAX        4095
#define AS5600_RAW_RES        4096.0f

// ====================================================================
// 寄存器地址（ams AS5600 datasheet）
// ====================================================================
#define AS5600_REG_ZMCO       0x00   /*!< 零位永久编程次数（只读） */
#define AS5600_REG_ZPOS_MSB   0x01   /*!< 零位位置 高字节 */
#define AS5600_REG_ZPOS_LSB   0x02   /*!< 零位位置 低字节 */
#define AS5600_REG_MPOS_MSB   0x03   /*!< 最大位置 高字节 */
#define AS5600_REG_MPOS_LSB   0x04   /*!< 最大位置 低字节 */
#define AS5600_REG_MANG_MSB   0x05   /*!< 最大角度 高字节 */
#define AS5600_REG_MANG_LSB   0x06   /*!< 最大角度 低字节 */
#define AS5600_REG_CONF_MSB   0x07   /*!< 配置 高字节 */
#define AS5600_REG_CONF_LSB   0x08   /*!< 配置 低字节 */
#define AS5600_REG_STATUS     0x0B   /*!< 状态 */
#define AS5600_REG_RAWANGLE_MSB 0x0C /*!< 原始角度 高字节 */
#define AS5600_REG_RAWANGLE_LSB 0x0D /*!< 原始角度 低字节 */
#define AS5600_REG_ANGLE_MSB  0x0E   /*!< 角度 高字节（12 位有效） */
#define AS5600_REG_ANGLE_LSB  0x0F   /*!< 角度 低字节 */
#define AS5600_REG_AGC        0x1A   /*!< 自动增益控制 */
#define AS5600_REG_MAGNITUDE_MSB 0x1B /*!< 磁场强度 高字节 */
#define AS5600_REG_MAGNITUDE_LSB 0x1C /*!< 磁场强度 低字节 */
#define AS5600_REG_BURN       0xFF   /*!< 烧录命令 */

// ====================================================================
// STATUS 状态位（寄存器 0x0B）
// ====================================================================
#define AS5600_STATUS_MH      0x08   /*!< 磁场过强 (magnet too high) */
#define AS5600_STATUS_ML      0x10   /*!< 磁场过弱 (magnet too low) */
#define AS5600_STATUS_MD      0x20   /*!< 检测到有效磁场 (magnet detected) */

// ====================================================================
// 类型定义
// ====================================================================
typedef struct {
    i2c_master_dev_handle_t dev;        /*!< AS5600 设备句柄（挂载在 PCA9546A 总线上） */
    uint16_t                zero_offset; /*!< 软件零位（12 位原始值） */
    float                   gear_ratio;  /*!< 减速比：编码器轴圈数 / 输出轴圈数（电机轴=4.5） */
    int32_t                 position;    /*!< 多圈累计位置（编码器轴，单位 raw LSB，可为负） */
    int16_t                 last_rel_raw;/*!< 上次相对零位 raw（用于回绕检测） */
    bool                    multi_valid; /*!< 累计是否已有有效基准 */
    portMUX_TYPE            mux;         /*!< 保护多圈状态（position/zero_offset/last_rel_raw） */
} as5600_handle_t;

typedef struct {
    bool magnet_detected;   /*!< 磁场有效 */
    bool magnet_too_weak;   /*!< 磁场过弱 */
    bool magnet_too_strong; /*!< 磁场过强 */
} as5600_status_t;

/*!< 诊断读数 */
typedef struct {
    uint16_t     raw;       /*!< 原始 12 位角度（编码器轴，0~4095） */
    uint16_t     magnitude; /*!< 磁场强度（0~4095） */
    float        out_deg;   /*!< 输出轴多圈连续角度（相对零位，含减速比，可为负） */
    as5600_status_t status; /*!< 磁场状态 */
} as5600_diag_t;

// ====================================================================
// API
// ====================================================================

/**
 * @brief 初始化 AS5600 设备（挂载到指定 I2C 总线）
 *
 * 调用前需通过 PCA9546A 选中 AS5600 所在通道，并取得总线句柄：
 *   pca9546a_select(&mux, PCA9546A_CH0);
 *   as5600_init(pca9546a_get_bus(&mux), AS5600_ADDR_DEFAULT, 100000, 4.5f, &enc);
 *
 * @param bus           I2C master 总线句柄（来自 PCA9546A）
 * @param addr          AS5600 7 位从机地址（默认 0x36）
 * @param scl_speed_hz  SCL 频率（如 100000）
 * @param gear_ratio    减速比（编码器轴圈数/输出轴圈数；编码器直连输出轴=1.0，电机轴=4.5）
 * @param out           输出句柄
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_init(i2c_master_bus_handle_t bus, uint8_t addr,
                      uint32_t scl_speed_hz, float gear_ratio, as5600_handle_t *out);

/**
 * @brief 读取 12 位原始角度（编码器轴，0 ~ 4095，对应 0 ~ 360°）
 *
 * @param h   句柄
 * @param raw 输出原始值
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_read_raw(as5600_handle_t *h, uint16_t *raw);

/**
 * @brief 读取云台输出轴**多圈连续**角度（度，相对软件零位，含减速比换算）
 *
 * 每次调用会更新圈数累计（跨 360° 自动回绕计数），输出连续角度，可为负。
 * 输出轴角度 = 累计位置 / gear_ratio。
 * 上电后首次读取以当前位置为 0 基准。
 *
 * @param h   句柄
 * @param deg 输出角度（多圈连续，可为负）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_read_angle(as5600_handle_t *h, float *deg);

/**
 * @brief 读取状态（磁场检测情况）
 *
 * @param h      句柄
 * @param status 输出状态
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_get_status(as5600_handle_t *h, as5600_status_t *status);

/**
 * @brief 组合诊断读取（一次 I2C 传输读取 ANGLE + STATUS + AGC + MAGNITUDE）
 *
 * 用于排查：原始角度是否随轴转动、磁场强度是否合适。
 *
 * @param h    句柄
 * @param diag 输出诊断数据
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_read_diag(as5600_handle_t *h, as5600_diag_t *diag);

/**
 * @brief 将当前位置设置为角度 0（软件零位，不烧录 Flash）
 *
 * 调用后 as5600_read_angle 返回相对该点的角度。
 *
 * @param h 句柄
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_set_zero(as5600_handle_t *h);

/**
 * @brief 将当前位置标定为指定输出轴角度并**重置多圈累计**（软件标定，不烧录 Flash）
 *
 * 接受任意角度并将当前位置标定为指定输出轴角度。
 * 之后 as5600_read_angle / M114 的 ENC 以该点为基准累计连续角度。
 * 软件零位不写入 NVS/Flash，掉电重启后归零。
 *
 * @param h   句柄
 * @param deg 期望当前输出轴角度（度，可为负）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t as5600_set_angle(as5600_handle_t *h, float deg);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H
