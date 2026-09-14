#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "driver/gpio.h"
#include <stdbool.h>

typedef struct {
    gpio_num_t gpio;       // GPIO pin
    bool pullup_enable;    // Enable internal pull-up
    bool active_low;       // true = triggered when reads low, false = triggered when reads high
} limit_switch_config_t;

typedef int limit_switch_handle_t;

/**
 * @brief Initialize a limit switch (GPIO input polling)
 *
 * @param config Pin configuration
 * @return Handle, or -1 on failure
 */
limit_switch_handle_t limit_switch_init(const limit_switch_config_t *config);

/**
 * @brief Check whether the limit switch is triggered
 *
 * @param handle Limit switch handle
 * @return true = triggered, false = not triggered
 */
bool limit_switch_is_triggered(limit_switch_handle_t handle);

#endif // LIMIT_SWITCH_H