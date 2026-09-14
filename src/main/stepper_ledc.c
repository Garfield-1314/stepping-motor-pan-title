#include "stepper_ledc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

static const char *TAG = "STEPPER_LEDC";

#define MAX_LEDC_MOTORS   2
// 12 位分辨率（ESP32-C3 可选，定时器位宽上限 14）。
// LEDC 最低频率 = 源时钟 / (0x3FFFF * 2^duty_res)，最高频率 = 源时钟 / 2^duty_res。
// 10 位：APB(80M) 下最低 76 Hz，无法输出 50 Hz 及以下的低速指令；
// 12 位：XTAL(40M) 下 50 Hz ~ 9.8kHz，APB(80M) 下 19 Hz ~ 19.5kHz，
//        同时覆盖 PGV 低速（≥50 Hz）与 PG1/PG2 高速（5000 Hz），互不影响。
#define LEDC_DUTY_RES     LEDC_TIMER_12_BIT   // 12 位分辨率：0~4095
#define LEDC_DUTY_HALF    (1 << (12 - 1))     // 2048 → 50% 占空比
// 速度安全区间 STEP_SPEED_MIN_HZ / STEP_SPEED_MAX_HZ 定义于 stepper_ledc.h

// 1/16 细分下：200 步/圈 × 4.5 齿轮比 × 16 细分 = 14400 微步/圈
#define MICROSTEPS_PER_REV 14400
#define DEG_PER_STEP       (360.0f / MICROSTEPS_PER_REV)  // 0.025°

// 加减速定位（move_step_accel）的起始速度（步/秒）
#define STEP_START_HZ      200

typedef struct {
    stepper_ledc_config_t config;
    ledc_timer_t   timer;     // 每电机独立定时器（避免双电机互扰频率）
    ledc_channel_t channel;   // CH0 / CH1
    volatile long  position;  // 软件计数：正转 +1/步，反转 -1/步
    volatile signed char dir_sign;  // 方向符号 +1/-1（ISR 直接使用，避免中断中调用 flash 函数）
    bool used;
} stepper_ledc_instance_t;

static stepper_ledc_instance_t s_motors[MAX_LEDC_MOTORS];
static bool s_isr_installed = false;

static bool is_valid(stepper_ledc_handle_t motor)
{
    return motor >= 0 && motor < MAX_LEDC_MOTORS && s_motors[motor].used;
}

// ====================================================================
// STEP 上升沿中断 → 软件计数
// 当 LEDC 输出方波翻转时，同引脚输入通路捕获上升沿进入此处。
// 注意：ISR 中不能调用任何 flash 中的 API（gpio_get_level 等位于 flash，
//       中断执行时 cache 未命中会触发 panic 0xdeadc0de）。
//       这里使用 RAM 内存中预置的 dir_sign 直接累加，纯 RAM 运算。
// ====================================================================
static void IRAM_ATTR step_isr(void *arg)
{
    stepper_ledc_instance_t *m = (stepper_ledc_instance_t *)arg;
    m->position += m->dir_sign;
}

stepper_ledc_handle_t stepper_ledc_init(const stepper_ledc_config_t *config)
{
    if (!config) return -1;

    // 查找空闲槽位
    stepper_ledc_handle_t handle = -1;
    for (int i = 0; i < MAX_LEDC_MOTORS; i++) {
        if (!s_motors[i].used) { handle = i; break; }
    }
    if (handle < 0) {
        ESP_LOGE(TAG, "No free motor slot");
        return -1;
    }

    stepper_ledc_instance_t *m = &s_motors[handle];
    m->config = *config;
    m->position = 0;
    m->dir_sign = 1;

    // ---- GPIO 输出：DIR / EN / MS ----
    uint64_t out_mask = (1ULL << m->config.dir_pin);
    if (m->config.en_pin  != GPIO_NUM_NC) out_mask |= (1ULL << m->config.en_pin);
    if (m->config.ms1_pin != GPIO_NUM_NC) out_mask |= (1ULL << m->config.ms1_pin);
    if (m->config.ms2_pin != GPIO_NUM_NC) out_mask |= (1ULL << m->config.ms2_pin);
    if (m->config.ms3_pin != GPIO_NUM_NC) out_mask |= (1ULL << m->config.ms3_pin);

    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = out_mask,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGE(TAG, "Motor[%d] DIR/EN GPIO config failed", handle);
        return -1;
    }

    if (m->config.en_pin != GPIO_NUM_NC) gpio_set_level(m->config.en_pin, 1); // 默认禁能
    gpio_set_level(m->config.dir_pin, 0);

    // 固定 1/16 细分：MS1=MS2=MS3=1
    if (m->config.ms1_pin != GPIO_NUM_NC) gpio_set_level(m->config.ms1_pin, 1);
    if (m->config.ms2_pin != GPIO_NUM_NC) gpio_set_level(m->config.ms2_pin, 1);
    if (m->config.ms3_pin != GPIO_NUM_NC) gpio_set_level(m->config.ms3_pin, 1);

    // ---- STEP 引脚：输出+输入同时使能，上升沿中断 ----
    gpio_config_t step_io = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT_OUTPUT,     // 关键：LEDC 输出 + 同引脚读回
        .pin_bit_mask = (1ULL << m->config.step_pin),
        .pull_up_en = 0,
        .pull_down_en = 0,
    };
    if (gpio_config(&step_io) != ESP_OK) {
        ESP_LOGE(TAG, "Motor[%d] STEP GPIO config failed", handle);
        return -1;
    }

    if (!s_isr_installed) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "install GPIO ISR service failed: %s", esp_err_to_name(err));
            return -1;
        }
        s_isr_installed = true;
    }
    if (gpio_isr_handler_add(m->config.step_pin, step_isr, m) != ESP_OK) {
        ESP_LOGE(TAG, "Motor[%d] STEP ISR install failed", handle);
        return -1;
    }

    // ---- LEDC：方波输出（每电机独立定时器）----
    m->timer = (handle == 0) ? LEDC_TIMER_0 : LEDC_TIMER_1;
    m->channel = (handle == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = m->timer,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = 1000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Motor[%d] LEDC timer config failed", handle);
        return -1;
    }

    ledc_channel_config_t chan_cfg = {
        .gpio_num   = m->config.step_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = m->channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = m->timer,
        .duty       = LEDC_DUTY_HALF,   // 50% 占空比
        .hpoint     = 0,
    };
    if (ledc_channel_config(&chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Motor[%d] LEDC channel config failed", handle);
        return -1;
    }

    // 初始停止：duty=0（输出低电平，不发脉冲）并暂停定时器
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->channel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->channel);
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, m->timer);

    m->used = true;
    ESP_LOGI(TAG, "Motor[%d] LEDC init (STEP=%d DIR=%d chan=%d timer=%d)",
             handle, m->config.step_pin, m->config.dir_pin,
             m->channel, m->timer);
    return handle;
}

void stepper_ledc_move_step(stepper_ledc_handle_t motor, int steps, float speed_hz, bool forward)
{
    if (!is_valid(motor) || steps <= 0 || speed_hz <= 0) return;

    stepper_ledc_instance_t *m = &s_motors[motor];

    // 1. 目标步数（根据方向 ±steps）
    long target = m->position + (forward ? steps : -steps);

    // 2. 套用底层原语：直接以目标速度启动
    stepper_ledc_move_speed(motor, forward ? speed_hz : -speed_hz);

    // 3. 轮询软件计数到目标 → 自动停止
    //    注意：不能用 pdMS_TO_TICKS(1)，当 FREERTOS_HZ<1000 时整型截断为 0
    //    （vTaskDelay(0) = 空操作忙等，会饿死 IDLE 任务触发 task_wdt）。
    //    直接延时 ≥1 tick 作为兜底。
    while (1) {
        bool reached = forward ? (m->position >= target)
                               : (m->position <= target);
        if (reached) break;
        vTaskDelay(1);
    }

    // 4. 停止
    stepper_ledc_move_speed(motor, 0.0f);
}

void stepper_ledc_move_step_accel(stepper_ledc_handle_t motor, int steps,
                                  float speed_hz, float accel, bool forward)
{
    if (!is_valid(motor) || steps <= 0 || speed_hz <= 0 || accel <= 0) return;

    stepper_ledc_instance_t *m = &s_motors[motor];

    // 1. 目标步数
    long target = m->position + (forward ? steps : -steps);

    // 2. 以起始速度起步（套用底层原语）
    float cur = STEP_START_HZ;
    stepper_ledc_move_speed(motor, forward ? cur : -cur);

    const float dt = 0.001f;   // 每 1ms 调整一次频率

    // 3. 梯形加减速循环：加速 → 匀速 → 减速，精确停在目标
    while (1) {
        bool reached = forward ? (m->position >= target)
                               : (m->position <= target);
        if (reached) break;

        long remaining = forward ? (target - m->position)
                                 : (m->position - target);
        if (remaining < 0) remaining = 0;

        // 减速所需距离：v² = 2as → s = v² / (2a)
        float s_decel = (cur * cur) / (2.0f * accel);

        if ((float)remaining <= s_decel) {
            // 减速段
            cur -= accel * dt;
            if (cur < STEP_START_HZ) cur = STEP_START_HZ;
        } else if (cur < speed_hz) {
            // 加速段
            cur += accel * dt;
            if (cur > speed_hz) cur = speed_hz;
        } else {
            // 匀速段
            cur = speed_hz;
        }

        // 套用底层原语调速（同上，避免 pdMS_TO_TICKS(1) 整型截断为忙等）
        if (cur > STEP_SPEED_MAX_HZ) cur = STEP_SPEED_MAX_HZ;
        stepper_ledc_move_speed(motor, forward ? cur : -cur);
        vTaskDelay(1);
    }

    // 4. 停止
    stepper_ledc_move_speed(motor, 0.0f);
}

void stepper_ledc_move_both(stepper_ledc_handle_t m1, int steps1, bool fwd1,
                            stepper_ledc_handle_t m2, int steps2, bool fwd2,
                            float speed_hz)
{
    if (speed_hz <= 0) return;
    if (!is_valid(m1) && !is_valid(m2)) return;
    if (steps1 <= 0 && steps2 <= 0) return;

    stepper_ledc_instance_t *a = is_valid(m1) ? &s_motors[m1] : NULL;
    stepper_ledc_instance_t *b = is_valid(m2) ? &s_motors[m2] : NULL;

    // 1. 计算各自目标步数
    long tgt1 = (a && steps1 > 0) ? a->position + (fwd1 ? steps1 : -steps1) : 0;
    long tgt2 = (b && steps2 > 0) ? b->position + (fwd2 ? steps2 : -steps2) : 0;

    // 2. 同时启动两轴（非阻塞 DC 式原语）
    if (a && steps1 > 0) stepper_ledc_move_speed(m1, fwd1 ? speed_hz : -speed_hz);
    if (b && steps2 > 0) stepper_ledc_move_speed(m2, fwd2 ? speed_hz : -speed_hz);

    // 3. 单循环联合轮询：每轴到位立即停止，两轴都停才退出
    //    修复：原实现轴到位后未立即停，会继续转到另一轴完成（多走步数）
    bool running1 = (a && steps1 > 0);
    bool running2 = (b && steps2 > 0);
    while (running1 || running2) {
        if (running1) {
            bool done1 = fwd1 ? (a->position >= tgt1) : (a->position <= tgt1);
            if (done1) {
                stepper_ledc_move_speed(m1, 0.0f);   // 轴 1 到位立即停
                running1 = false;
            }
        }
        if (running2) {
            bool done2 = fwd2 ? (b->position >= tgt2) : (b->position <= tgt2);
            if (done2) {
                stepper_ledc_move_speed(m2, 0.0f);   // 轴 2 到位立即停
                running2 = false;
            }
        }
        vTaskDelay(1);
    }
}

void stepper_ledc_move_both_accel(stepper_ledc_handle_t m1, int steps1, bool fwd1,
                                  stepper_ledc_handle_t m2, int steps2, bool fwd2,
                                  float speed_hz, float accel)
{
    if (speed_hz <= 0 || accel <= 0) return;
    if (!is_valid(m1) && !is_valid(m2)) return;
    if (steps1 <= 0 && steps2 <= 0) return;

    stepper_ledc_instance_t *a = is_valid(m1) ? &s_motors[m1] : NULL;
    stepper_ledc_instance_t *b = is_valid(m2) ? &s_motors[m2] : NULL;

    // 1. 各自目标步数
    long tgt1 = (a && steps1 > 0) ? a->position + (fwd1 ? steps1 : -steps1) : 0;
    long tgt2 = (b && steps2 > 0) ? b->position + (fwd2 ? steps2 : -steps2) : 0;

    // 2. 各自以起始速度同时起步
    float cur1 = STEP_START_HZ, cur2 = STEP_START_HZ;
    if (a && steps1 > 0) stepper_ledc_move_speed(m1, fwd1 ? cur1 : -cur1);
    if (b && steps2 > 0) stepper_ledc_move_speed(m2, fwd2 ? cur2 : -cur2);

    const float dt = 0.001f;   // 每 1ms 调整一次频率

    // 3. 单循环联合梯形加减速：每轴到位立即停止，两轴都停才退出
    //    修复：原实现轴到位后未立即停，会继续转到另一轴完成（多走步数）
    bool running1 = (a && steps1 > 0);
    bool running2 = (b && steps2 > 0);
    while (running1 || running2) {
        // ---- 轴 1 ----
        if (running1) {
            bool done1 = fwd1 ? (a->position >= tgt1) : (a->position <= tgt1);
            if (done1) {
                stepper_ledc_move_speed(m1, 0.0f);   // 轴 1 到位立即停
                running1 = false;
            } else {
                long rem1 = fwd1 ? (tgt1 - a->position) : (a->position - tgt1);
                if (rem1 < 0) rem1 = 0;
                float s_dec1 = (cur1 * cur1) / (2.0f * accel);
                if ((float)rem1 <= s_dec1) {
                    cur1 -= accel * dt;
                    if (cur1 < STEP_START_HZ) cur1 = STEP_START_HZ;
                } else if (cur1 < speed_hz) {
                    cur1 += accel * dt;
                    if (cur1 > speed_hz) cur1 = speed_hz;
                } else {
                    cur1 = speed_hz;
                }
                if (cur1 > STEP_SPEED_MAX_HZ) cur1 = STEP_SPEED_MAX_HZ;
                stepper_ledc_move_speed(m1, fwd1 ? cur1 : -cur1);
            }
        }

        // ---- 轴 2 ----
        if (running2) {
            bool done2 = fwd2 ? (b->position >= tgt2) : (b->position <= tgt2);
            if (done2) {
                stepper_ledc_move_speed(m2, 0.0f);   // 轴 2 到位立即停
                running2 = false;
            } else {
                long rem2 = fwd2 ? (tgt2 - b->position) : (b->position - tgt2);
                if (rem2 < 0) rem2 = 0;
                float s_dec2 = (cur2 * cur2) / (2.0f * accel);
                if ((float)rem2 <= s_dec2) {
                    cur2 -= accel * dt;
                    if (cur2 < STEP_START_HZ) cur2 = STEP_START_HZ;
                } else if (cur2 < speed_hz) {
                    cur2 += accel * dt;
                    if (cur2 > speed_hz) cur2 = speed_hz;
                } else {
                    cur2 = speed_hz;
                }
                if (cur2 > STEP_SPEED_MAX_HZ) cur2 = STEP_SPEED_MAX_HZ;
                stepper_ledc_move_speed(m2, fwd2 ? cur2 : -cur2);
            }
        }

        vTaskDelay(1);
    }
}

void stepper_ledc_move_speed(stepper_ledc_handle_t motor, float speed_hz)
{
    if (!is_valid(motor)) return;

    stepper_ledc_instance_t *m = &s_motors[motor];

    int freq = (int)(speed_hz > 0 ? speed_hz : -speed_hz);

    if (freq == 0) {
        // 停止：duty=0 输出低电平 + 暂停定时器
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m->channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, m->channel);
        ledc_timer_pause(LEDC_LOW_SPEED_MODE, m->timer);
        return;
    }

    // 速度安全钳制：区间外显式钳制并告警，杜绝静默跑飞/静默丢弃
    if (freq < STEP_SPEED_MIN_HZ) {
        ESP_LOGW(TAG, "Speed %d Hz below min %.0f Hz, clamped", freq, STEP_SPEED_MIN_HZ);
        freq = (int)STEP_SPEED_MIN_HZ;
    }
    if (freq > STEP_SPEED_MAX_HZ) {
        ESP_LOGW(TAG, "Speed %d Hz over max %.0f Hz, clamped", freq, STEP_SPEED_MAX_HZ);
        freq = (int)STEP_SPEED_MAX_HZ;
    }

    // 方向：正速度 = 正转，负速度 = 反转（先更新 dir_sign 再翻转 GPIO）
    m->dir_sign = (speed_hz > 0) ? 1 : -1;
    gpio_set_level(m->config.dir_pin, speed_hz > 0 ? 1 : 0);

    // 调速并启动（DC 式，非阻塞）
    ledc_set_freq(LEDC_LOW_SPEED_MODE, m->timer, (uint32_t)freq);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->channel, LEDC_DUTY_HALF);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->channel);
    ledc_timer_resume(LEDC_LOW_SPEED_MODE, m->timer);
}

long stepper_ledc_get_steps(stepper_ledc_handle_t motor)
{
    if (!is_valid(motor)) return 0;
    return s_motors[motor].position;
}

void stepper_ledc_set_position(stepper_ledc_handle_t motor, long position)
{
    if (!is_valid(motor)) return;
    s_motors[motor].position = position;
}

float stepper_ledc_get_angle(stepper_ledc_handle_t motor)
{
    if (!is_valid(motor)) return 0.0f;
    return s_motors[motor].position * DEG_PER_STEP;
}

void stepper_ledc_reset_position(stepper_ledc_handle_t motor)
{
    if (!is_valid(motor)) return;
    s_motors[motor].position = 0;
}

void stepper_ledc_set_enable(stepper_ledc_handle_t motor, bool enable)
{
    if (!is_valid(motor)) return;

    stepper_ledc_instance_t *m = &s_motors[motor];
    if (m->config.en_pin != GPIO_NUM_NC) {
        gpio_set_level(m->config.en_pin, enable ? 0 : 1);  // EN 低有效
    }
}
