#include "limit_switch.h"
#include "esp_log.h"

static const char *TAG = "LIMIT_SWITCH";

#define MAX_LIMIT_SWITCHES 4

typedef struct {
    limit_switch_config_t config;
    bool used;
} limit_switch_instance_t;

static limit_switch_instance_t s_switches[MAX_LIMIT_SWITCHES] = {0};

static bool is_valid_handle(limit_switch_handle_t handle)
{
    return handle >= 0 && handle < MAX_LIMIT_SWITCHES && s_switches[handle].used;
}

limit_switch_handle_t limit_switch_init(const limit_switch_config_t *config)
{
    if (!config) return -1;

    limit_switch_handle_t handle = -1;
    for (int i = 0; i < MAX_LIMIT_SWITCHES; i++) {
        if (!s_switches[i].used) { handle = i; break; }
    }
    if (handle < 0) {
        ESP_LOGE(TAG, "No free limit switch slot");
        return -1;
    }

    limit_switch_instance_t *sw = &s_switches[handle];
    sw->config = *config;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << sw->config.gpio),
        .pull_down_en = 0,
        .pull_up_en = sw->config.pullup_enable ? 1 : 0,
    };
    gpio_config(&io_conf);

    sw->used = true;

    ESP_LOGI(TAG, "Limit switch[%d] init (GPIO=%d, active_low=%d, pullup=%d)",
             handle, sw->config.gpio, sw->config.active_low, sw->config.pullup_enable);
    return handle;
}

bool limit_switch_is_triggered(limit_switch_handle_t handle)
{
    if (!is_valid_handle(handle)) return false;

    limit_switch_instance_t *sw = &s_switches[handle];
    int level = gpio_get_level(sw->config.gpio);
    return sw->config.active_low ? (level == 0) : (level == 1);
}