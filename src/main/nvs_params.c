/**
 * @file nvs_params.c
 * @brief 系统参数存储模块实现（基于 NVS 掉电保存）
 */
#include "nvs_params.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "NVS_PARAMS";

// ====================================================================
// 键名与默认值
// 新增参数：在此增加键名宏 + 默认值，并在 load/save 中处理
// ====================================================================
#define NVS_KEY_PAN_BACKOFF    "pan_backoff"     /*!< Pan G28 回退步数 */
#define NVS_KEY_TILT_BACKOFF   "tilt_backoff"    /*!< Tilt G28 回退步数 */

#define DEFAULT_PAN_BACKOFF    3490              /*!< 默认 Pan 回退（M220 P） */
#define DEFAULT_TILT_BACKOFF   2940              /*!< 默认 Tilt 回退（M220 T） */

// ====================================================================
// 内存中的参数镜像（掉电前由 save 提交到 Flash）
// ====================================================================
static int s_pan_backoff  = DEFAULT_PAN_BACKOFF;
static int s_tilt_backoff = DEFAULT_TILT_BACKOFF;

// ====================================================================
// 初始化：加载参数到内存
// ====================================================================
esp_err_t nvs_params_init(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_PARAMS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // 首次使用：无命名空间，保留默认值
        ESP_LOGW(TAG, "namespace '%s' not found, using defaults (P=%d T=%d)",
                 NVS_PARAMS_NAMESPACE, DEFAULT_PAN_BACKOFF, DEFAULT_TILT_BACKOFF);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(RO) failed: %s", esp_err_to_name(err));
        return err;
    }

    // ---- Pan 回退 ----
    int32_t v = DEFAULT_PAN_BACKOFF;
    if (nvs_get_i32(h, NVS_KEY_PAN_BACKOFF, &v) != ESP_OK) {
        v = DEFAULT_PAN_BACKOFF;
        ESP_LOGW(TAG, "'%s' missing, default=%d", NVS_KEY_PAN_BACKOFF, v);
    }
    s_pan_backoff = (v >= 0 && v <= NVS_PARAMS_BACKOFF_MAX)
                        ? (int)v : DEFAULT_PAN_BACKOFF;

    // ---- Tilt 回退 ----
    v = DEFAULT_TILT_BACKOFF;
    if (nvs_get_i32(h, NVS_KEY_TILT_BACKOFF, &v) != ESP_OK) {
        v = DEFAULT_TILT_BACKOFF;
        ESP_LOGW(TAG, "'%s' missing, default=%d", NVS_KEY_TILT_BACKOFF, v);
    }
    s_tilt_backoff = (v >= 0 && v <= NVS_PARAMS_BACKOFF_MAX)
                         ? (int)v : DEFAULT_TILT_BACKOFF;

    nvs_close(h);

    ESP_LOGI(TAG, "params loaded: pan_backoff=%d tilt_backoff=%d",
             s_pan_backoff, s_tilt_backoff);
    return ESP_OK;
}

// ====================================================================
// 保存：提交全部参数到 Flash
// ====================================================================
esp_err_t nvs_params_save(void)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_PARAMS_NAMESPACE, NVS_READWRITE, &h), TAG,
                        "nvs_open(RW) failed");

    esp_err_t err = nvs_set_i32(h, NVS_KEY_PAN_BACKOFF, s_pan_backoff);
    if (err == ESP_OK) err = nvs_set_i32(h, NVS_KEY_TILT_BACKOFF, s_tilt_backoff);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "params saved: pan_backoff=%d tilt_backoff=%d",
             s_pan_backoff, s_tilt_backoff);
    return ESP_OK;
}

// ====================================================================
// 访问函数
// ====================================================================
int nvs_params_get_pan_backoff(void)  { return s_pan_backoff; }
int nvs_params_get_tilt_backoff(void) { return s_tilt_backoff; }

void nvs_params_set_pan_backoff(int v)  { s_pan_backoff  = v; }
void nvs_params_set_tilt_backoff(int v) { s_tilt_backoff = v; }
