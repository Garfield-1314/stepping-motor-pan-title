/**
 * @file PCA9546A.c
 * @brief PCA9546A 4 通道 I2C 总线切换器驱动实现
 *
 * 基于 ESP-IDF v5.x 新版 i2c_master 驱动（driver/i2c_master.h）。
 * 新 PCB 的 I2C 线（SCL/SDA）已有外部上拉电阻，故不启用内部上拉。
 */
#include "PCA9546A.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "driver/gpio.h"

static const char *TAG = "PCA9546A";

#define PCA9546A_XFER_TIMEOUT_MS  100   /*!< 单次 I2C 传输超时（ms） */
#define PCA9546A_RESET_HOLD_US    1000  /*!< 复位低电平保持时间（µs） */
#define PCA9546A_RESET_RELEASE_US 1000  /*!< 复位释放后等待就绪时间（µs） */

// ====================================================================
// 复位（RESET 低有效：拉低复位，拉高正常工作）
// ====================================================================
esp_err_t pca9546a_reset(pca9546a_handle_t *h)
{
    if (!h || h->reset_io == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;

    // 配置为推挽输出
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << h->reset_io),
    };
    gpio_config(&io);

    gpio_set_level(h->reset_io, 0);          // 拉低 → 进入复位
    esp_rom_delay_us(PCA9546A_RESET_HOLD_US);
    gpio_set_level(h->reset_io, 1);          // 拉高 → 释放复位
    esp_rom_delay_us(PCA9546A_RESET_RELEASE_US);

    // 硬件复位会清空芯片的通道寄存器，软件缓存也必须失效。
    h->current = PCA9546A_CH_NONE;

    ESP_LOGI(TAG, "reset done (reset_io=%d)", h->reset_io);
    return ESP_OK;
}

// ====================================================================
// 初始化
// ====================================================================
esp_err_t pca9546a_init(int sda_io, int scl_io, int reset_io, uint8_t addr,
                        uint32_t scl_speed_hz, pca9546a_handle_t *out)
{
    if (!out || sda_io < 0 || scl_io < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    pca9546a_handle_t *h = out;
    h->bus      = NULL;
    h->dev      = NULL;
    h->current  = PCA9546A_CH_NONE;
    h->reset_io = reset_io;

    // ---- 0. 复位脉冲：清空通道选择、释放可能被阻塞的 I2C 总线 ----
    if (reset_io != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(pca9546a_reset(h), TAG, "reset failed");
    }

    // ---- 1. 创建 I2C master 总线 ----
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port           = I2C_NUM_0,          // ESP32-C3 仅一个 I2C
        .sda_io_num         = sda_io,
        .scl_io_num         = scl_io,
        .clk_source         = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt  = 7,
        .intr_priority      = 0,
        .trans_queue_depth  = 0,
        .flags.enable_internal_pullup = false,   // 新 PCB 已加外部上拉；若无外部上拉可改为 true
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &h->bus), TAG,
                        "create I2C master bus failed (SDA=%d SCL=%d)", sda_io, scl_io);

    // ---- 2. 把 PCA9546A 自身挂到总线上 ----
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = scl_speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(h->bus, &dev_cfg, &h->dev), TAG,
                        "add PCA9546A device failed (addr=0x%02X)", addr);

    // ---- 3. 初始关闭所有通道 ----
    ESP_RETURN_ON_ERROR(pca9546a_deselect(h), TAG, "initial deselect failed");

    ESP_LOGI(TAG, "init ok: SDA=%d SCL=%d addr=0x%02X speed=%lu Hz",
             sda_io, scl_io, addr, (unsigned long)scl_speed_hz);
    return ESP_OK;
}

// ====================================================================
// 通道选择
// ====================================================================
esp_err_t pca9546a_select(pca9546a_handle_t *h, pca9546a_channel_t ch)
{
    if (!h || !h->dev) return ESP_ERR_INVALID_STATE;
    if (ch < PCA9546A_CH0 || ch > PCA9546A_CH3) return ESP_ERR_INVALID_ARG;

    // 幂等：已在目标通道则跳过（避免高频采样重复写寄存器/刷日志）
    if (h->current == ch) return ESP_OK;

    uint8_t ctrl = (uint8_t)(1u << ch);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(h->dev, &ctrl, 1, PCA9546A_XFER_TIMEOUT_MS), TAG,
                        "select CH%d failed", ch);

    h->current = ch;
    ESP_LOGD(TAG, "selected CH%d (ctrl=0x%02X)", ch, ctrl);
    return ESP_OK;
}

esp_err_t pca9546a_deselect(pca9546a_handle_t *h)
{
    if (!h || !h->dev) return ESP_ERR_INVALID_STATE;

    uint8_t ctrl = 0x00;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(h->dev, &ctrl, 1, PCA9546A_XFER_TIMEOUT_MS), TAG,
                        "deselect failed");

    h->current = PCA9546A_CH_NONE;
    ESP_LOGD(TAG, "all channels deselected");
    return ESP_OK;
}

// ====================================================================
// 寄存器读取
// ====================================================================
esp_err_t pca9546a_read_ctrl(pca9546a_handle_t *h, uint8_t *ctrl)
{
    if (!h || !h->dev || !ctrl) return ESP_ERR_INVALID_ARG;

    return i2c_master_receive(h->dev, ctrl, 1, PCA9546A_XFER_TIMEOUT_MS);
}

// ====================================================================
// 查询
// ====================================================================
pca9546a_channel_t pca9546a_get_channel(const pca9546a_handle_t *h)
{
    if (!h) return PCA9546A_CH_NONE;
    return (h->current >= PCA9546A_CH0 && h->current <= PCA9546A_CH3)
               ? h->current : PCA9546A_CH_NONE;
}

i2c_master_bus_handle_t pca9546a_get_bus(pca9546a_handle_t *h)
{
    return h ? h->bus : NULL;
}
