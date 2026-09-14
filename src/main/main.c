#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs_params.h"
#include "stepper_ledc.h"
#include "limit_switch.h"
#include "gcode_ledc.h"
#include "PCA9546A.h"
#include "AS5600.h"
#include "encoder.h"

static const char *TAG = "MAIN";

// 电机 1（Pan）
#define STEP1_GPIO 4
#define DIR1_GPIO  5

// 电机 2（Tilt）
#define STEP2_GPIO 0
#define DIR2_GPIO  1

// 共享引脚
#define EN_GPIO    6
// 细分：新 PCB 上 A4988 的 MS1/MS2/MS3 已硬件上拉到 VCC，固定 1/16 细分，
//       固件不再控制细分引脚（IO10/IO18/IO19 释放，其中 IO18/IO19 用于 I2C）。

// 限位开关引脚
#define PAN_LIMIT_GPIO  3
#define TILT_LIMIT_GPIO 7

// PCA9546A I2C 扩展芯片（新 PCB：IO19=SCL, IO18=SDA, IO10=RESET）
#define PCA9546A_SCL_GPIO   19
#define PCA9546A_SDA_GPIO   18
#define PCA9546A_RESET_GPIO 10                 // RESET 复位引脚（低有效，外部已上拉到 VCC）
#define PCA9546A_I2C_ADDR   PCA9546A_ADDR(0, 0, 0)   // A2A1A0=000 → 0x70
#define PCA9546A_SPEED_HZ   100000             // 100 kHz

// AS5600 磁性编码器（Pan=CH0, Tilt=CH1，安装在电机轴，4.5:1 减速）
#define AS5600_GEAR_RATIO   4.5f               // 电机 90 圈 = 大齿轮 20 圈（4.5:1）
#define ENCODER_PAN_CH      PCA9546A_CH0       // Pan 编码器通道
#define ENCODER_TILT_CH     PCA9546A_CH1       // Tilt 编码器通道
// 编码器安装方向与步进坐标方向相反，统一对外坐标后 STEP/ENC 同号。
#define ENCODER_PAN_SIGN    (-1)
#define ENCODER_TILT_SIGN   (-1)

static pca9546a_handle_t s_pca9546a;
static encoder_t         s_encoder;

// ---- 编码器高频采样任务 ----
// 周期读取两轴编码器并更新多圈累计，使 position 始终最新；
// 采样间隔固定（5ms），避免 M114 手动查询间隔过大导致高速漏计。
#define ENCODER_SAMPLE_MS  5
static void encoder_sample_task(void *arg)
{
    float pan_deg = 0.0f;
    float tilt_deg = 0.0f;
    while (1) {
        // 内部互斥锁保护 PCA9546A 通道切换 + 读取，与 G-code 层并发安全
        bool pan_ok = (encoder_read_angle(&s_encoder, ENCODER_PAN, &pan_deg) == ESP_OK);
        if (!pan_ok) {
            ESP_LOGW(TAG, "pan encoder sample failed");
        }
        bool tilt_ok = (encoder_read_angle(&s_encoder, ENCODER_TILT, &tilt_deg) == ESP_OK);
        if (!tilt_ok) {
            ESP_LOGW(TAG, "tilt encoder sample failed");
        }
        gcode_ledc_update_encoder_position(pan_deg, pan_ok, tilt_deg, tilt_ok);
        vTaskDelay(pdMS_TO_TICKS(ENCODER_SAMPLE_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Pan-Tilt LEDC + G-code start");

    // ---- NVS 初始化（G28 归零参数等掉电保存）----
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    ESP_ERROR_CHECK(nvs_params_init());

    stepper_ledc_config_t pan_cfg = {
        .step_pin = STEP1_GPIO,
        .dir_pin  = DIR1_GPIO,
        .en_pin   = EN_GPIO,
        .ms1_pin  = GPIO_NUM_NC,   // 细分由硬件固定 1/16，不控制
        .ms2_pin  = GPIO_NUM_NC,
        .ms3_pin  = GPIO_NUM_NC,
    };
    stepper_ledc_handle_t pan = stepper_ledc_init(&pan_cfg);
    if (pan < 0) return;

    stepper_ledc_config_t tilt_cfg = {
        .step_pin = STEP2_GPIO,
        .dir_pin  = DIR2_GPIO,
        .en_pin   = EN_GPIO,
        .ms1_pin  = GPIO_NUM_NC,   // 细分由硬件固定 1/16，不控制
        .ms2_pin  = GPIO_NUM_NC,
        .ms3_pin  = GPIO_NUM_NC,
    };
    stepper_ledc_handle_t tilt = stepper_ledc_init(&tilt_cfg);
    if (tilt < 0) return;

    // ---- 限位开关初始化（GPIO 输入轮询，外部上拉、低有效）----
    // 新 PCB 已在开关端到 VCC 之间加外部上拉电阻，故禁用内部上拉
    // （避免内部 ~45kΩ 上拉与外部上拉并联改变触发阈值）。
    limit_switch_config_t pan_limit_cfg = {
        .gpio = PAN_LIMIT_GPIO,
        .pullup_enable = false,
        .active_low = true,
    };
    limit_switch_handle_t pan_limit = limit_switch_init(&pan_limit_cfg);
    if (pan_limit < 0) return;

    limit_switch_config_t tilt_limit_cfg = {
        .gpio = TILT_LIMIT_GPIO,
        .pullup_enable = false,
        .active_low = true,
    };
    limit_switch_handle_t tilt_limit = limit_switch_init(&tilt_limit_cfg);
    if (tilt_limit < 0) return;

    // ---- PCA9546A I2C 扩展芯片初始化 ----
    // IO18/IO19 原为 stepper 的 MS2/MS1 细分引脚，新 PCB 上细分已由硬件
    // 固定为 1/16（MS1/2/3 上拉到 VCC）；IO10 原为 MS3，现作 PCA9546A 的
    // RESET 复位引脚（低有效），初始化时自动执行复位脉冲。
    ESP_ERROR_CHECK(pca9546a_init(PCA9546A_SDA_GPIO, PCA9546A_SCL_GPIO,
                                  PCA9546A_RESET_GPIO, PCA9546A_I2C_ADDR,
                                  PCA9546A_SPEED_HZ, &s_pca9546a));

    // ---- AS5600 双编码器（PCA9546A Pan=CH0, Tilt=CH1，电机轴，4.5:1 减速）----
    // 两编码器同址 0x36，靠 PCA9546A 通道隔离避免地址冲突；
    // encoder 模块内部用互斥锁保护通道切换与读取。
    ESP_ERROR_CHECK(encoder_init(&s_encoder, &s_pca9546a,
                                 ENCODER_PAN_CH, ENCODER_TILT_CH,
                                 AS5600_GEAR_RATIO, PCA9546A_SPEED_HZ));
    ESP_ERROR_CHECK(encoder_set_direction_sign(&s_encoder, ENCODER_PAN,
                                               ENCODER_PAN_SIGN));
    ESP_ERROR_CHECK(encoder_set_direction_sign(&s_encoder, ENCODER_TILT,
                                               ENCODER_TILT_SIGN));

    // 高频采样任务：周期读取两轴编码器维护多圈累计（高速下防漏计）
    xTaskCreate(encoder_sample_task, "enc_samp", 2048, NULL, 10, NULL);

    // 启动 UART G-code 命令层（不返回；保持主循环）
    // 传入双编码器句柄，M114 查询时可反馈两轴编码器角度
    gcode_ledc_start(pan, tilt, pan_limit, tilt_limit, &s_encoder);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
