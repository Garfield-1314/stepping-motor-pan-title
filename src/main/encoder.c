/**
 * @file encoder.c
 * @brief 双编码器（Pan/Tilt）管理模块实现
 *
 * 通过 PCA9546A 通道隔离 + 互斥锁，安全访问两个同址(0x36)的 AS5600。
 */
#include "encoder.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "ENCODER";

esp_err_t encoder_init(encoder_t *e, pca9546a_handle_t *mux,
                       pca9546a_channel_t pan_ch, pca9546a_channel_t tilt_ch,
                       float gear_ratio, uint32_t scl_speed_hz)
{
    if (!e || !mux || gear_ratio <= 0.0f) return ESP_ERR_INVALID_ARG;

    e->mux = mux;
    e->ch[ENCODER_PAN]  = pan_ch;
    e->ch[ENCODER_TILT] = tilt_ch;
    e->direction_sign[ENCODER_PAN]  = 1;
    e->direction_sign[ENCODER_TILT] = 1;
    e->lock = xSemaphoreCreateMutex();
    if (!e->lock) {
        ESP_LOGE(TAG, "create mutex failed");
        return ESP_ERR_NO_MEM;
    }

    // ---- 初始化 Pan 编码器 ----
    ESP_RETURN_ON_ERROR(pca9546a_select(mux, pan_ch), TAG, "select pan ch failed");
    ESP_RETURN_ON_ERROR(as5600_init(pca9546a_get_bus(mux), AS5600_ADDR_DEFAULT,
                                    scl_speed_hz, gear_ratio, &e->enc[ENCODER_PAN]),
                        TAG, "init pan encoder failed");

    // ---- 初始化 Tilt 编码器 ----
    ESP_RETURN_ON_ERROR(pca9546a_select(mux, tilt_ch), TAG, "select tilt ch failed");
    ESP_RETURN_ON_ERROR(as5600_init(pca9546a_get_bus(mux), AS5600_ADDR_DEFAULT,
                                    scl_speed_hz, gear_ratio, &e->enc[ENCODER_TILT]),
                        TAG, "init tilt encoder failed");

    // 默认回到 Pan 通道
    ESP_RETURN_ON_ERROR(pca9546a_select(mux, pan_ch), TAG, "restore pan ch failed");

    ESP_LOGI(TAG, "init ok: pan=CH%d tilt=CH%d gear=%.2f",
             pan_ch, tilt_ch, gear_ratio);
    return ESP_OK;
}

esp_err_t encoder_read_angle(encoder_t *e, encoder_axis_t axis, float *deg)
{
    if (!e || !e->lock || !deg) return ESP_ERR_INVALID_ARG;
    if (axis != ENCODER_PAN && axis != ENCODER_TILT) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(e->lock, portMAX_DELAY);
    esp_err_t err = pca9546a_select(e->mux, e->ch[axis]);
    if (err != ESP_OK) {
        xSemaphoreGive(e->lock);
        return err;
    }
    float as5600_deg;
    err = as5600_read_angle(&e->enc[axis], &as5600_deg);
    if (err == ESP_OK) {
        *deg = as5600_deg * e->direction_sign[axis];
    }
    xSemaphoreGive(e->lock);
    return err;
}

esp_err_t encoder_set_direction_sign(encoder_t *e, encoder_axis_t axis, int sign)
{
    if (!e || !e->lock) return ESP_ERR_INVALID_ARG;
    if (axis != ENCODER_PAN && axis != ENCODER_TILT) return ESP_ERR_INVALID_ARG;
    if (sign != 1 && sign != -1) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(e->lock, portMAX_DELAY);
    e->direction_sign[axis] = sign;
    xSemaphoreGive(e->lock);
    return ESP_OK;
}

esp_err_t encoder_read_diag(encoder_t *e, encoder_axis_t axis, as5600_diag_t *diag)
{
    if (!e || !e->lock || !diag) return ESP_ERR_INVALID_ARG;
    if (axis != ENCODER_PAN && axis != ENCODER_TILT) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(e->lock, portMAX_DELAY);
    esp_err_t err = pca9546a_select(e->mux, e->ch[axis]);
    if (err != ESP_OK) {
        xSemaphoreGive(e->lock);
        return err;
    }
    err = as5600_read_diag(&e->enc[axis], diag);
    if (err == ESP_OK) {
        // RAW/MAG/ST 保持硬件诊断值，只有对外角度应用坐标方向修正。
        diag->out_deg *= e->direction_sign[axis];
    }
    xSemaphoreGive(e->lock);
    return err;
}

esp_err_t encoder_zero_all(encoder_t *e)
{
    if (!e || !e->lock) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(e->lock, portMAX_DELAY);
    esp_err_t err = pca9546a_select(e->mux, e->ch[ENCODER_PAN]);
    if (err == ESP_OK) err = as5600_set_angle(&e->enc[ENCODER_PAN], 0.0f);
    if (err == ESP_OK) err = pca9546a_select(e->mux, e->ch[ENCODER_TILT]);
    if (err == ESP_OK) err = as5600_set_angle(&e->enc[ENCODER_TILT], 0.0f);
    if (err == ESP_OK) err = pca9546a_select(e->mux, e->ch[ENCODER_PAN]);
    xSemaphoreGive(e->lock);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "both encoders zeroed");
    }
    return err;
}
