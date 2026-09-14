/**
 * @file encoder.h
 * @brief 双编码器（Pan/Tilt）管理模块
 *
 * 通过 PCA9546A 扩展挂载两个 AS5600 磁性编码器（Pan=CH0, Tilt=CH1），
 * 同址 0x36（靠 PCA9546A 通道隔离避免地址冲突）。
 *
 * 由于 PCA9546A 同一时刻只能选中一个通道，所有"通道切换 + 读取/标定"
 * 操作都由内部互斥锁保护，保证采样任务与 G-code 层并发安全。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "PCA9546A.h"
#include "AS5600.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_PAN = 0,
    ENCODER_TILT = 1,
} encoder_axis_t;

typedef struct {
    pca9546a_handle_t *mux;        /*!< PCA9546A 句柄 */
    as5600_handle_t    enc[2];     /*!< 0=Pan, 1=Tilt */
    pca9546a_channel_t ch[2];      /*!< 0=Pan 通道, 1=Tilt 通道 */
    int                 direction_sign[2]; /*!< 机械正方向相对 AS5600 正方向：+1/-1 */
    SemaphoreHandle_t  lock;       /*!< I2C 共享总线互斥（保护通道切换+读写） */
} encoder_t;

/**
 * @brief 初始化两个编码器（PAN/TILT）
 *
 * @param e            编码器管理器句柄
 * @param mux          PCA9546A 句柄（须已初始化）
 * @param pan_ch       Pan 编码器所在通道（如 CH0）
 * @param tilt_ch      Tilt 编码器所在通道（如 CH1）
 * @param gear_ratio   减速比（编码器轴圈数/输出轴圈数，电机轴=4.5）
 * @param scl_speed_hz I2C SCL 频率
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t encoder_init(encoder_t *e, pca9546a_handle_t *mux,
                       pca9546a_channel_t pan_ch, pca9546a_channel_t tilt_ch,
                       float gear_ratio, uint32_t scl_speed_hz);

/**
 * @brief 读取指定轴编码器输出轴多圈连续角度（线程安全）
 *
 * @param e    句柄
 * @param axis ENCODER_PAN / ENCODER_TILT
 * @param deg  输出角度（度，含减速比，可为负）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t encoder_read_angle(encoder_t *e, encoder_axis_t axis, float *deg);

/**
 * @brief 设置指定轴的坐标方向符号
 *
 * @param e    编码器管理器
 * @param axis ENCODER_PAN / ENCODER_TILT
 * @param sign +1 或 -1；用于统一编码器与步进坐标方向
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t encoder_set_direction_sign(encoder_t *e, encoder_axis_t axis, int sign);

/**
 * @brief 读取指定轴编码器诊断（RAW/MAG/状态，线程安全）
 *
 * @param e    句柄
 * @param axis ENCODER_PAN / ENCODER_TILT
 * @param diag 输出诊断数据
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t encoder_read_diag(encoder_t *e, encoder_axis_t axis, as5600_diag_t *diag);

/**
 * @brief 两轴编码器同时标零为 0°（G28 完成后调用）
 *
 * @param e 句柄
 * @return ESP_OK 成功；否则返回 I2C/设备错误
 */
esp_err_t encoder_zero_all(encoder_t *e);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
