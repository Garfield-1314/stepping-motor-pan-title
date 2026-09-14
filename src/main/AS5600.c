/**
 * @file AS5600.c
 * @brief AS5600 12-bit 磁性旋转位置传感器驱动实现（I2C）
 *
 * 基于 ESP-IDF v5.x i2c_master 新 API，通过 PCA9546A 扩展总线访问。
 * 读取操作：先写寄存器地址，再连续读数据字节。
 */
#include "AS5600.h"
#include "esp_check.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "AS5600";

#define AS5600_XFER_TIMEOUT_MS  100   /*!< 单次 I2C 传输超时（ms） */

// ====================================================================
// 内部：读寄存器（写地址后读 n 字节）
// ====================================================================
static esp_err_t as5600_read_regs(as5600_handle_t *h, uint8_t reg,
                                  uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, buf, len,
                                       AS5600_XFER_TIMEOUT_MS);
}

// ====================================================================
// 初始化
// ====================================================================
esp_err_t as5600_init(i2c_master_bus_handle_t bus, uint8_t addr,
                      uint32_t scl_speed_hz, float gear_ratio, as5600_handle_t *out)
{
    if (!bus || !out || gear_ratio <= 0.0f) return ESP_ERR_INVALID_ARG;

    as5600_handle_t *h = out;
    h->dev = NULL;
    h->zero_offset = 0;
    h->gear_ratio  = gear_ratio;
    h->position    = 0;
    h->last_rel_raw = 0;
    h->multi_valid = false;
    spinlock_initialize(&h->mux);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = scl_speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &h->dev), TAG,
                        "add AS5600 device failed (addr=0x%02X)", addr);

    // 探测确认设备在线（读状态寄存器）
    uint8_t st;
    esp_err_t err = as5600_read_regs(h, AS5600_REG_STATUS, &st, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 not responding (addr=0x%02X): %s",
                 addr, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "init ok: addr=0x%02X speed=%lu Hz gear=%.2f",
             addr, (unsigned long)scl_speed_hz, gear_ratio);
    return ESP_OK;
}

// ====================================================================
// 内部：更新多圈累计（回绕检测 + 位置累加，单位 raw LSB）
// 相邻两次采样间隔内转动超过 180°（编码器轴）会漏计，云台低速下安全。
// 临界区保护：可与采样任务/G28 标零并发安全访问。
// ====================================================================
static void as5600_update_multi(as5600_handle_t *h, uint16_t raw)
{
    portENTER_CRITICAL(&h->mux);
    int16_t rel = (int16_t)((raw - h->zero_offset) & AS5600_RAW_MAX);

    if (h->multi_valid) {
        int32_t delta = (int32_t)rel - h->last_rel_raw;   // -4095..4095
        if (delta >  2048) delta -= 4096;                  // 正向跨过 4095→0
        else if (delta < -2048) delta += 4096;             // 反向跨过 0→4095
        h->position += delta;
    } else {
        h->multi_valid = true;
        h->position = 0;   // 首次以上电后当前位置为 0 基准
    }
    h->last_rel_raw = rel;
    portEXIT_CRITICAL(&h->mux);
}

// ====================================================================
// 读取原始角度（12 位，编码器轴）
// ====================================================================
esp_err_t as5600_read_raw(as5600_handle_t *h, uint16_t *raw)
{
    if (!h || !h->dev || !raw) return ESP_ERR_INVALID_ARG;

    uint8_t buf[2];
    ESP_RETURN_ON_ERROR(as5600_read_regs(h, AS5600_REG_ANGLE_MSB, buf, 2), TAG,
                        "read ANGLE failed");
    *raw = (uint16_t)(((uint16_t)(buf[0] & 0x0F) << 8) | buf[1]);  // 高字节仅低 4 位有效
    return ESP_OK;
}

// ====================================================================
// 读取云台输出轴多圈连续角度（度，相对零位，含减速比换算）
// 输出轴角度 = 多圈累计位置 / gear_ratio
// ====================================================================
esp_err_t as5600_read_angle(as5600_handle_t *h, float *deg)
{
    if (!h || !h->dev || !deg) return ESP_ERR_INVALID_ARG;

    uint16_t raw;
    ESP_RETURN_ON_ERROR(as5600_read_raw(h, &raw), TAG, "read raw angle failed");
    as5600_update_multi(h, raw);

    float pos;
    portENTER_CRITICAL(&h->mux);
    pos = (float)h->position;
    portEXIT_CRITICAL(&h->mux);
    *deg = pos * 360.0f / AS5600_RAW_RES / h->gear_ratio;
    return ESP_OK;
}

// ====================================================================
// 读取状态（磁场检测）
// ====================================================================
esp_err_t as5600_get_status(as5600_handle_t *h, as5600_status_t *status)
{
    if (!h || !h->dev || !status) return ESP_ERR_INVALID_ARG;

    uint8_t st;
    ESP_RETURN_ON_ERROR(as5600_read_regs(h, AS5600_REG_STATUS, &st, 1), TAG,
                        "read STATUS failed");

    status->magnet_detected   = (st & AS5600_STATUS_MD) != 0;
    status->magnet_too_weak   = (st & AS5600_STATUS_ML) != 0;
    status->magnet_too_strong = (st & AS5600_STATUS_MH) != 0;
    return ESP_OK;
}

// ====================================================================
// 组合诊断读取：分别读 STATUS(0x0B)、ANGLE(0x0E/0x0F)、MAGNITUDE(0x1B/0x1C)
// ====================================================================
esp_err_t as5600_read_diag(as5600_handle_t *h, as5600_diag_t *diag)
{
    if (!h || !h->dev || !diag) return ESP_ERR_INVALID_ARG;

    uint8_t st;
    ESP_RETURN_ON_ERROR(as5600_read_regs(h, AS5600_REG_STATUS, &st, 1), TAG,
                        "read STATUS failed");

    uint8_t angle[2];
    ESP_RETURN_ON_ERROR(as5600_read_regs(h, AS5600_REG_ANGLE_MSB, angle, 2), TAG,
                        "read ANGLE failed");

    uint8_t mag[2];
    ESP_RETURN_ON_ERROR(as5600_read_regs(h, AS5600_REG_MAGNITUDE_MSB, mag, 2), TAG,
                        "read MAGNITUDE failed");

    diag->raw       = (uint16_t)(((uint16_t)(angle[0] & 0x0F) << 8) | angle[1]);
    diag->status.magnet_detected   = (st & AS5600_STATUS_MD) != 0;
    diag->status.magnet_too_weak   = (st & AS5600_STATUS_ML) != 0;
    diag->status.magnet_too_strong = (st & AS5600_STATUS_MH) != 0;
    diag->magnitude = (uint16_t)(((uint16_t)(mag[0] & 0x0F) << 8) | mag[1]);

    // 更新多圈累计，输出连续输出轴角度
    as5600_update_multi(h, diag->raw);
    float pos;
    portENTER_CRITICAL(&h->mux);
    pos = (float)h->position;
    portEXIT_CRITICAL(&h->mux);
    diag->out_deg = pos * 360.0f / AS5600_RAW_RES / h->gear_ratio;
    return ESP_OK;
}

// ====================================================================
// 软件标定：把当前位置标定为指定输出轴角度并重置多圈累计
// ====================================================================
esp_err_t as5600_set_angle(as5600_handle_t *h, float deg)
{
    if (!h || !h->dev) return ESP_ERR_INVALID_ARG;

    uint16_t raw;
    ESP_RETURN_ON_ERROR(as5600_read_raw(h, &raw), TAG, "read raw for set_angle failed");

    // 当前位置设为编码器相对角 0；累计位置 = 目标输出角 deg 对应 LSB（编码器轴）
    portENTER_CRITICAL(&h->mux);
    h->zero_offset  = raw;
    h->position     = (int32_t)llroundf(deg * h->gear_ratio * AS5600_RAW_RES / 360.0f);
    h->last_rel_raw = 0;
    h->multi_valid  = true;
    portEXIT_CRITICAL(&h->mux);

    ESP_LOGI(TAG, "set_angle: out=%.1f deg (raw=%u, pos=%ld)", deg, raw, (long)h->position);
    return ESP_OK;
}

esp_err_t as5600_set_zero(as5600_handle_t *h)
{
    return as5600_set_angle(h, 0.0f);
}
