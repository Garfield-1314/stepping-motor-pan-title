/**
 * @file nvs_params.h
 * @brief 系统参数存储模块（基于 NVS 掉电保存）
 *
 * 集中管理需要在掉电后保持的参数。当前包含：
 *   - G28 归零回退步数（对应 M220 P/T，默认 P3490 T2940）
 *
 * 后续新增参数：
 *   1. 在 nvs_params.c 增加对应键名宏 + 默认值
 *   2. 增加 get/set 访问函数
 *   3. 在 nvs_params_load/save 中读写
 */
#ifndef NVS_PARAMS_H
#define NVS_PARAMS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!< NVS 命名空间（避免与其它模块键冲突） */
#define NVS_PARAMS_NAMESPACE "params"
#define NVS_PARAMS_BACKOFF_MAX 100000

/**
 * @brief 初始化参数模块
 *
 * 打开 NVS 命名空间并加载全部参数到内存。
 * 注意：调用前须已完成 nvs_flash_init()（main 中统一处理）。
 * 首次使用（命名空间不存在）时返回 ESP_OK 并使用默认值。
 *
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t nvs_params_init(void);

/**
 * @brief 将内存中的全部参数提交到 Flash（掉电保持）
 *
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t nvs_params_save(void);

// ====================================================================
// G28 归零回退参数（M220 P/T，默认 3490 / 2940）
// ====================================================================
int  nvs_params_get_pan_backoff(void);
int  nvs_params_get_tilt_backoff(void);
void nvs_params_set_pan_backoff(int v);
void nvs_params_set_tilt_backoff(int v);

#ifdef __cplusplus
}
#endif

#endif // NVS_PARAMS_H
