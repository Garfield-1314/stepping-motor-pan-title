/**
 * @file gcode_ledc.c
 * @brief G-code 解析器（映射到 LEDC 步进驱动）
 * 
 * 架构与旧版 gcode_parser 一致：
 *   UART 层  →  行读取器  →  命令分发器  →  处理器
 *                                             ├─ PG（家族：PG1, PG2, PGV）
 *                                             ├─ M17 / M18 / M114
 *                                             └─（未来命令家族）
 * 
 * 添加新命令家族：
 *   1. 编写处理器：static void cmd_xxx(const char *args)
 *   2. 注册到 s_commands[]：{ "XX", cmd_xx }
 *   3. 输入 "XX..." 将分发到 cmd_xx("...")
 * 
 * 给 PG 家族添加子命令：
 *   1. 在 cmd_pg() 的 switch(sub_cmd) 中添加 case
 */

#include "gcode_ledc.h"
#include "nvs_params.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

// ====================================================================
// 配置
// ====================================================================
static const char *TAG = "GCODE_LEDC";

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      256
#define UART_RX_TIMEOUT_MS 10
#define UART_TX_GPIO       21
#define UART_RX_GPIO       20

#define OUTPUT_STEPS_PER_DEG  40.0f   // 14400 微步/圈 ÷ 360°
#define PAN_MIN_DEG           (-87.0f)
#define PAN_MAX_DEG           (87.0f)
#define TILT_MIN_DEG          (-62.0f)
#define TILT_MAX_DEG          (62.0f)
#define G28_POLL_MS           20
#define G28_TIMEOUT_MS        15000
#define G28_MAX_SEEK_STEPS    50000

typedef struct {
    float pan_min_deg;
    float pan_max_deg;
    float tilt_min_deg;
    float tilt_max_deg;
} motion_limits_t;

// ====================================================================
// 上下文（电机句柄、共享位置状态）
// 注：位置计数由 LEDC 驱动的 GPIO 中断软件计数提供，
//     此处的 pan_position / tilt_position 是 PG 命令的绝对坐标记录。
// ====================================================================
typedef struct {
    stepper_ledc_handle_t pan;
    stepper_ledc_handle_t tilt;
    limit_switch_handle_t pan_limit;
    limit_switch_handle_t tilt_limit;
    encoder_t            *encoder;   // 双编码器管理（M114 反馈两轴角度；可 NULL）
    volatile long pan_position;   // Pan 绝对坐标（输出轴步数）
    volatile long tilt_position;  // Tilt 绝对坐标（输出轴步数）
    volatile bool pan_velocity_mode;       // PGV 时由编码器更新位置
    volatile bool tilt_velocity_mode;
    volatile signed char pan_velocity_sign;   // PGV 方向：+1/-1，供边界保护使用
    volatile signed char tilt_velocity_sign;
    SemaphoreHandle_t motion_lock;      // 串行化 UART 命令与编码器采样的运动操作
    int pan_backoff;     // G28 回零后 Pan 回退步数（M220 设置）
    int tilt_backoff;    // G28 回零后 Tilt 回退步数（M220 设置）
} gcode_ctx_t;

static gcode_ctx_t s_ctx;
static motion_limits_t s_motion_limits = {
    .pan_min_deg = PAN_MIN_DEG,
    .pan_max_deg = PAN_MAX_DEG,
    .tilt_min_deg = TILT_MIN_DEG,
    .tilt_max_deg = TILT_MAX_DEG,
};

// 编码器高频采样任务调用：PGV 期间以编码器输出轴角度作为位置真值。
void gcode_ledc_update_encoder_position(float pan_deg, bool pan_valid,
                                        float tilt_deg, bool tilt_valid)
{
    SemaphoreHandle_t lock = s_ctx.motion_lock;
    if (lock) xSemaphoreTake(lock, portMAX_DELAY);

    if (!pan_valid && s_ctx.pan_velocity_mode) {
        // PGV 运行依赖编码器闭环；采样失败时不能继续盲转。
        // 保留最后一次有效位置，等待下一条 PGV 指令重新同步编码器。
        stepper_ledc_move_speed(s_ctx.pan, 0.0f);
        s_ctx.pan_velocity_mode = false;
        s_ctx.pan_velocity_sign = 0;
        ESP_LOGE(TAG, "PGV: Pan encoder sample failed, stopped for safety");
    } else if (pan_valid && s_ctx.pan_velocity_mode) {
        s_ctx.pan_position = lroundf(pan_deg * OUTPUT_STEPS_PER_DEG);
        if ((pan_deg >= s_motion_limits.pan_max_deg && s_ctx.pan_velocity_sign > 0) ||
            (pan_deg <= s_motion_limits.pan_min_deg && s_ctx.pan_velocity_sign < 0)) {
            stepper_ledc_move_speed(s_ctx.pan, 0.0f);
            s_ctx.pan_velocity_mode = false;
            s_ctx.pan_velocity_sign = 0;
            stepper_ledc_set_position(s_ctx.pan,
                                      lroundf(pan_deg * OUTPUT_STEPS_PER_DEG));
            ESP_LOGW(TAG, "PGV: Pan encoder limit reached at %.2f deg, stopped", pan_deg);
        }
    }
    if (!tilt_valid && s_ctx.tilt_velocity_mode) {
        // 同上：Tilt 编码器失效时立即停机，禁止无反馈运行。
        stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
        s_ctx.tilt_velocity_mode = false;
        s_ctx.tilt_velocity_sign = 0;
        ESP_LOGE(TAG, "PGV: Tilt encoder sample failed, stopped for safety");
    } else if (tilt_valid && s_ctx.tilt_velocity_mode) {
        s_ctx.tilt_position = lroundf(tilt_deg * OUTPUT_STEPS_PER_DEG);
        if ((tilt_deg >= s_motion_limits.tilt_max_deg && s_ctx.tilt_velocity_sign > 0) ||
            (tilt_deg <= s_motion_limits.tilt_min_deg && s_ctx.tilt_velocity_sign < 0)) {
            stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
            s_ctx.tilt_velocity_mode = false;
            s_ctx.tilt_velocity_sign = 0;
            stepper_ledc_set_position(s_ctx.tilt,
                                      lroundf(tilt_deg * OUTPUT_STEPS_PER_DEG));
            ESP_LOGW(TAG, "PGV: Tilt encoder limit reached at %.2f deg, stopped", tilt_deg);
        }
    }

    if (lock) xSemaphoreGive(lock);
}

static bool get_axis_limits_from(const motion_limits_t *limits, encoder_axis_t axis,
                                 long *min_steps, long *max_steps,
                                 float *min_deg, float *max_deg)
{
    if (!limits) return false;

    if (axis == ENCODER_PAN) {
        *min_deg = limits->pan_min_deg;
        *max_deg = limits->pan_max_deg;
        *min_steps = (long)ceilf(*min_deg * OUTPUT_STEPS_PER_DEG);
        *max_steps = (long)floorf(*max_deg * OUTPUT_STEPS_PER_DEG);
        return true;
    }
    if (axis == ENCODER_TILT) {
        *min_deg = limits->tilt_min_deg;
        *max_deg = limits->tilt_max_deg;
        *min_steps = (long)ceilf(*min_deg * OUTPUT_STEPS_PER_DEG);
        *max_steps = (long)floorf(*max_deg * OUTPUT_STEPS_PER_DEG);
        return true;
    }
    return false;
}

static bool get_axis_limits(encoder_axis_t axis, long *min_steps, long *max_steps,
                            float *min_deg, float *max_deg)
{
    return get_axis_limits_from(&s_motion_limits, axis,
                                min_steps, max_steps, min_deg, max_deg);
}

static bool position_target_valid(encoder_axis_t axis, long steps)
{
    long min_steps, max_steps;
    float min_deg, max_deg;
    if (!get_axis_limits(axis, &min_steps, &max_steps, &min_deg, &max_deg)) {
        return false;
    }

    float deg = (float)steps / OUTPUT_STEPS_PER_DEG;
    return steps >= min_steps && steps <= max_steps &&
           deg >= min_deg && deg <= max_deg;
}

static bool velocity_allowed(encoder_axis_t axis, float deg, float speed)
{
    long min_steps, max_steps;
    float min_deg, max_deg;
    if (!get_axis_limits(axis, &min_steps, &max_steps, &min_deg, &max_deg)) {
        return false;
    }

    if (speed > 0.0f) return deg < max_deg;
    if (speed < 0.0f) return deg > min_deg;
    return true;
}

static bool validate_motion_limits(const motion_limits_t *limits)
{
    long min_steps, max_steps;
    float min_deg, max_deg;

    if (!limits ||
        limits->pan_min_deg < PAN_MIN_DEG || limits->pan_max_deg > PAN_MAX_DEG ||
        limits->tilt_min_deg < TILT_MIN_DEG || limits->tilt_max_deg > TILT_MAX_DEG ||
        limits->pan_min_deg >= limits->pan_max_deg ||
        limits->tilt_min_deg >= limits->tilt_max_deg) {
        return false;
    }

    if (!get_axis_limits_from(limits, ENCODER_PAN,
                              &min_steps, &max_steps, &min_deg, &max_deg) ||
        min_steps > max_steps) {
        return false;
    }
    if (!get_axis_limits_from(limits, ENCODER_TILT,
                              &min_steps, &max_steps, &min_deg, &max_deg) ||
        min_steps > max_steps) {
        return false;
    }
    return true;
}

static bool position_within_limits(const motion_limits_t *limits,
                                   encoder_axis_t axis, long position)
{
    long min_steps, max_steps;
    float min_deg, max_deg;
    if (!get_axis_limits_from(limits, axis, &min_steps, &max_steps,
                              &min_deg, &max_deg)) {
        return false;
    }

    float deg = (float)position / OUTPUT_STEPS_PER_DEG;
    return position >= min_steps && position <= max_steps &&
           deg >= min_deg && deg <= max_deg;
}

static float normalize_pgv_speed(float speed)
{
    float magnitude = fabsf(speed);
    if (magnitude < 0.5f) return 0.0f;
    if (magnitude < STEP_SPEED_MIN_HZ) magnitude = STEP_SPEED_MIN_HZ;
    if (magnitude > STEP_SPEED_MAX_HZ) magnitude = STEP_SPEED_MAX_HZ;
    return copysignf(magnitude, speed);
}

static bool read_encoder_axis(encoder_axis_t axis, bool align_stepper, float *deg_out)
{
    if (!s_ctx.encoder) return false;

    float deg;
    if (encoder_read_angle(s_ctx.encoder, axis, &deg) != ESP_OK) {
        return false;
    }

    long position = lroundf(deg * OUTPUT_STEPS_PER_DEG);
    if (axis == ENCODER_PAN) {
        s_ctx.pan_position = position;
        if (align_stepper) stepper_ledc_set_position(s_ctx.pan, position);
    } else {
        s_ctx.tilt_position = position;
        if (align_stepper) stepper_ledc_set_position(s_ctx.tilt, position);
    }
    if (deg_out) *deg_out = deg;
    return true;
}

static bool sync_encoder_axis(encoder_axis_t axis, bool align_stepper)
{
    return read_encoder_axis(axis, align_stepper, NULL);
}

static bool sync_encoder_positions(bool pan, bool tilt, bool align_stepper)
{
    bool ok = true;
    if (pan && !sync_encoder_axis(ENCODER_PAN, align_stepper)) ok = false;
    if (tilt && !sync_encoder_axis(ENCODER_TILT, align_stepper)) ok = false;
    return ok;
}

// 从 PGV 切换到定位模式前，停止连续脉冲并把步进软件计数对齐到编码器。
static bool prepare_position_move(void)
{
    bool pan_active = s_ctx.pan_velocity_mode;
    bool tilt_active = s_ctx.tilt_velocity_mode;
    if (!pan_active && !tilt_active) return true;

    if (pan_active) stepper_ledc_move_speed(s_ctx.pan, 0.0f);
    if (tilt_active) stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
    vTaskDelay(1);

    bool ok = sync_encoder_positions(pan_active, tilt_active, true);
    s_ctx.pan_velocity_mode = false;
    s_ctx.tilt_velocity_mode = false;
    s_ctx.pan_velocity_sign = 0;
    s_ctx.tilt_velocity_sign = 0;
    return ok;
}

// ====================================================================
// UART 层
// ====================================================================
static void uart_send(const char *str)
{
    uart_write_bytes(UART_PORT_NUM, str, strlen(str));
}

static void uart_send_ok(void)
{
    uart_send("OK\r\n");
}

static void uart_send_error(const char *msg)
{
    char buf[UART_BUF_SIZE];
    snprintf(buf, sizeof(buf), "ER: %s\r\n", msg);
    uart_send(buf);
}

// ====================================================================
// 参数提取器
// 从字符串中提取带字母前缀的参数。
// 示例："P3000 T2000 S1800" → pan=3000, tilt=2000, speed=1800
// ====================================================================
typedef struct {
    char key;       // 参数字母（P, T, S, A, ...）
    int  ival;      // 整数值
    float fval;     // 浮点值
    bool found;     // 参数是否存在
} param_t;

/**
 * @brief 扫描字符串提取单字母前缀参数
 * 
 * @param line     输入字符串（如 "P3000 T2000 S1800"）
 * @param expected 期望的字母字符串（如 "PTS"）
 * @param params   输出参数数组，对应每个期望字母
 * @param count    期望参数数量
 * @return int     成功返回 0，解析失败返回 -1
 */
static int parse_params(const char *line, const char *expected, param_t *params, int count)
{
    for (int i = 0; i < count; i++) {
        params[i].key   = expected[i];
        params[i].ival  = 0;
        params[i].fval  = 0.0f;
        params[i].found = false;
    }

    while (*line) {
        while (*line && isspace((unsigned char)*line)) line++;
        if (!*line) break;

        if (!isalpha((unsigned char)*line)) return -1;
        char c = toupper((unsigned char)*line);
        line++;

        if (*line == '=') line++;

        int idx = -1;
        for (int i = 0; i < count; i++) {
            if (params[i].key == c) { idx = i; break; }
        }
        if (idx < 0) {
            return -1;
        }

        if (params[idx].found) return -1;

        char *end;
        params[idx].fval = strtof(line, &end);
        if (end == line || !isfinite(params[idx].fval)) return -1;
        params[idx].ival = (int)params[idx].fval;
        params[idx].found = true;
        line = end;

        // 兼容部分串口工具把 CRLF 作为两个字面量字符发送："\\r\\n"。
        // 正常的实际 CR/LF 已由 isspace() 处理，这里只放行行尾的字面量转义。
        if (*line == '\\' &&
            ((line[1] == 'r') || (line[1] == 'n'))) {
            const char *tail = line;
            while (tail[0] == '\\' && (tail[1] == 'r' || tail[1] == 'n')) {
                tail += 2;
            }
            while (*tail && isspace((unsigned char)*tail)) tail++;
            if (!*tail) {
                line = tail;
                break;
            }
        }
    }
    return 0;
}

// ====================================================================
// PG 子命令分发器
// ====================================================================

/**
 * PG1：匀速绝对定位
 * 格式：PG1 P<绝对坐标> T<绝对坐标> S<步/秒>
 * 
 * P/T 为绝对目标位置（相对于 G28 原点）。
 * 计算 delta = 目标 - 当前记录，调用 LEDC move_step 匀速执行。
 */
static void cmd_pg1(const char *args)
{
    param_t params[] = {
        { .key = 'P' },
        { .key = 'T' },
        { .key = 'S' },
    };
    const int N = sizeof(params) / sizeof(params[0]);

    if (parse_params(args, "PTS", params, N) != 0) {
        uart_send_error("Bad PG1 params");
        return;
    }
    if (!params[0].found && !params[1].found) {
        uart_send_error("Missing P or T");
        return;
    }
    if (!params[2].found) {
        uart_send_error("Missing S");
        return;
    }

    float speed   = params[2].fval;
    if (speed <= 0.0f) {
        uart_send_error("Speed must be positive");
        return;
    }

    if (!prepare_position_move()) {
        uart_send_error("Encoder sync failed");
        return;
    }

    int pan_cmd   = params[0].found ? params[0].ival : s_ctx.pan_position;
    int tilt_cmd  = params[1].found ? params[1].ival : s_ctx.tilt_position;

    // 绝对定位：计算相对当前位置的位移
    long dPan  = pan_cmd  - s_ctx.pan_position;
    long dTilt = tilt_cmd - s_ctx.tilt_position;

    if (!position_target_valid(ENCODER_PAN, pan_cmd) ||
        !position_target_valid(ENCODER_TILT, tilt_cmd)) {
        uart_send_error("Target outside axis limits");
        return;
    }

    ESP_LOGI(TAG, "PG1: target P=%d T=%d S=%.0f",
             pan_cmd, tilt_cmd, speed);

    if (dPan == 0 && dTilt == 0) {
        uart_send_ok();   // 已在目标位置
        return;
    }

    // 匀速移动（双轴并行，同时启停）
    stepper_ledc_move_both(s_ctx.pan, (int)(dPan > 0 ? dPan : -dPan), dPan > 0,
                           s_ctx.tilt, (int)(dTilt > 0 ? dTilt : -dTilt), dTilt > 0,
                           speed);

    // 更新位置记录
    s_ctx.pan_position  = pan_cmd;
    s_ctx.tilt_position = tilt_cmd;

    uart_send_ok();
}

/**
 * PG2：加减速绝对定位
 * 格式：PG2 P<绝对坐标> T<绝对坐标> S<最大速度> A<加速度>
 * 
 * P/T 为绝对目标位置。调用 LEDC move_step_accel 梯形加减速平滑移动。
 */
static void cmd_pg2(const char *args)
{
    param_t params[] = {
        { .key = 'P' },
        { .key = 'T' },
        { .key = 'S' },
        { .key = 'A' },
    };
    const int N = sizeof(params) / sizeof(params[0]);

    if (parse_params(args, "PTSA", params, N) != 0) {
        uart_send_error("Bad PG2 params");
        return;
    }
    if (!params[0].found && !params[1].found) {
        uart_send_error("Missing P or T");
        return;
    }
    if (!params[2].found) {
        uart_send_error("Missing S (max speed)");
        return;
    }

    float speed  = params[2].fval;
    float accel  = params[3].found ? params[3].fval : 50000.0f;
    if (speed <= 0.0f || accel <= 0.0f) {
        uart_send_error("Speed and accel must be positive");
        return;
    }

    if (!prepare_position_move()) {
        uart_send_error("Encoder sync failed");
        return;
    }

    int pan_cmd  = params[0].found ? params[0].ival : s_ctx.pan_position;
    int tilt_cmd = params[1].found ? params[1].ival : s_ctx.tilt_position;

    // 绝对定位：计算相对当前位置的位移
    long dPan  = pan_cmd  - s_ctx.pan_position;
    long dTilt = tilt_cmd - s_ctx.tilt_position;

    if (!position_target_valid(ENCODER_PAN, pan_cmd) ||
        !position_target_valid(ENCODER_TILT, tilt_cmd)) {
        uart_send_error("Target outside axis limits");
        return;
    }

    ESP_LOGI(TAG, "PG2: target P=%d T=%d S=%.0f A=%.0f",
             pan_cmd, tilt_cmd, speed, accel);

    if (dPan == 0 && dTilt == 0) {
        uart_send_ok();   // 已在目标位置
        return;
    }

    // 加减速移动（双轴并行，同时启停）
    stepper_ledc_move_both_accel(s_ctx.pan, (int)(dPan > 0 ? dPan : -dPan), dPan > 0,
                                 s_ctx.tilt, (int)(dTilt > 0 ? dTilt : -dTilt), dTilt > 0,
                                 speed, accel);

    // 更新位置记录
    s_ctx.pan_position  = pan_cmd;
    s_ctx.tilt_position = tilt_cmd;

    uart_send_ok();
}

// PGV 速度命令（定义在文件后部，此处前向声明供 cmd_pg 分发）
static void cmd_pgv(const char *args);

/**
 * PG 家族分发器
 * 数字子命令："<数字> <参数>"，例如 "1 P3000 T2000 S1800" → cmd_pg1
 * 字母子命令："<字母><参数>"，例如 "V T1500" → cmd_pgv
 */
static void cmd_pg(const char *args)
{
    while (*args && isspace((unsigned char)*args)) args++;
    if (!*args) {
        uart_send_error("Missing PG sub-command");
        return;
    }

    char c = toupper((unsigned char)*args);

    if (isdigit((unsigned char)c)) {
        // 数字子命令：PG1 / PG2 ...
        char *end;
        long sub_cmd = strtol(args, &end, 10);
        if (end == args) {
            uart_send_error("Bad PG sub-command");
            return;
        }
        const char *params = end;

        switch (sub_cmd) {
            case 1: cmd_pg1(params); break;
            case 2: cmd_pg2(params); break;
            default:
                ESP_LOGW(TAG, "Unknown PG sub-command: %ld", sub_cmd);
                uart_send_error("Unknown PG cmd");
                break;
        }
    } else {
        // 字母子命令：PGV ...
        args++;
        switch (c) {
            case 'V': cmd_pgv(args); break;
            default:
                ESP_LOGW(TAG, "Unknown PG letter cmd: %c", c);
                uart_send_error("Unknown PG cmd");
                break;
        }
    }
}

// ====================================================================
// 状态字母 helper（D=有效 L=弱 H=强）
// ====================================================================
static void status_to_str(const as5600_status_t *st, char *buf, size_t len)
{
    int k = 0;
    if (st->magnet_detected)   buf[k++] = 'D';
    if (st->magnet_too_weak)   buf[k++] = 'L';
    if (st->magnet_too_strong) buf[k++] = 'H';
    if (k == 0) { buf[0] = '-'; buf[1] = '-'; k = 2; }
    buf[k] = '\0';
}

// ====================================================================
// M114：状态查询（通用，含编码器角度 + 诊断）
// 格式：M114
// 响应：MP: <pan_steps> <tilt_steps>\r\n                    （未挂编码器）
//       MP: <pan> <tilt> ENC: <pan_deg> <tilt_deg> STEP: <pan_step_deg> <tilt_step_deg>
//           RAW: <pan_raw> <tilt_raw> MAG: <pan_mag> <tilt_mag> ST: <pan_st> <tilt_st>\r\n
//       ENC  两轴编码器输出轴多圈连续角度（含减速比，可为负）
//       STEP 两轴步进输出轴角度（融合对比用，无丢步时应接近 ENC）
//       RAW  两轴编码器原始 12 位角度；MAG 磁场强度；ST 磁场状态(D/L/H)
// 后接 OK\r\n
// ====================================================================
static void cmd_m114(const char *args)
{
    char buf[UART_BUF_SIZE];
    // PGV 期间先取一次最新编码器位置，保证 MP 与实际机械位置一致。
    sync_encoder_positions(s_ctx.pan_velocity_mode, s_ctx.tilt_velocity_mode, false);
    if (s_ctx.encoder) {
        as5600_diag_t dp, dt;
        bool op = (encoder_read_diag(s_ctx.encoder, ENCODER_PAN, &dp) == ESP_OK);
        bool ot = (encoder_read_diag(s_ctx.encoder, ENCODER_TILT, &dt) == ESP_OK);
        if (op && ot) {
            char sp[4], st[4];
            status_to_str(&dp.status, sp, sizeof(sp));
            status_to_str(&dt.status, st, sizeof(st));

            snprintf(buf, sizeof(buf),
                     "MP: %ld %ld ENC: %.1f %.1f STEP: %.1f %.1f "
                     "RAW: %u %u MAG: %u %u ST: %s %s\r\n",
                     s_ctx.pan_position, s_ctx.tilt_position,
                     dp.out_deg, dt.out_deg,
                     stepper_ledc_get_angle(s_ctx.pan),
                     stepper_ledc_get_angle(s_ctx.tilt),
                     dp.raw, dt.raw, dp.magnitude, dt.magnitude,
                     sp, st);
        } else {
            snprintf(buf, sizeof(buf), "MP: %ld %ld ENC: ERR\r\n",
                     s_ctx.pan_position, s_ctx.tilt_position);
        }
    } else {
        snprintf(buf, sizeof(buf), "MP: %ld %ld\r\n",
                 s_ctx.pan_position, s_ctx.tilt_position);
    }
    uart_send(buf);
    uart_send_ok();
}

/**
 * M221：设置/查询本次运行有效的编码器角度限位（不写 NVS）
 * 格式：M221 [P<Pan最小角>] [Q<Pan最大角>] [T<Tilt最小角>] [U<Tilt最大角>]
 * 步数限位由角度按 OUTPUT_STEPS_PER_DEG 自动换算。
 */
static void cmd_m221(const char *args)
{
    param_t params[] = {
        { .key = 'P' },
        { .key = 'Q' },
        { .key = 'T' },
        { .key = 'U' },
    };
    const int N = sizeof(params) / sizeof(params[0]);

    if (parse_params(args, "PQTU", params, N) != 0) {
        uart_send_error("Bad M221 params");
        return;
    }

    motion_limits_t candidate = s_motion_limits;
    if (params[0].found) candidate.pan_min_deg = params[0].fval;
    if (params[1].found) candidate.pan_max_deg = params[1].fval;
    if (params[2].found) candidate.tilt_min_deg = params[2].fval;
    if (params[3].found) candidate.tilt_max_deg = params[3].fval;

    bool updating = params[0].found || params[1].found ||
                    params[2].found || params[3].found;
    if (updating && !validate_motion_limits(&candidate)) {
        uart_send_error("Motion limits out of range");
        return;
    }

    if (updating) {
        // 修改限位前停止 PGV，并用编码器刷新当前位置。
        if (!prepare_position_move()) {
            uart_send_error("Encoder sync failed");
            return;
        }
        if (s_ctx.encoder && !sync_encoder_positions(true, true, false)) {
            uart_send_error("Encoder sync failed");
            return;
        }
        if (!position_within_limits(&candidate, ENCODER_PAN, s_ctx.pan_position) ||
            !position_within_limits(&candidate, ENCODER_TILT, s_ctx.tilt_position)) {
            uart_send_error("Current position outside limits");
            return;
        }
        s_motion_limits = candidate;
    }

    long pan_min_steps, pan_max_steps, tilt_min_steps, tilt_max_steps;
    float pan_min_deg, pan_max_deg, tilt_min_deg, tilt_max_deg;
    get_axis_limits(ENCODER_PAN, &pan_min_steps, &pan_max_steps,
                    &pan_min_deg, &pan_max_deg);
    get_axis_limits(ENCODER_TILT, &tilt_min_steps, &tilt_max_steps,
                    &tilt_min_deg, &tilt_max_deg);

    char buf[UART_BUF_SIZE];
    snprintf(buf, sizeof(buf),
             "M221 ANGLE P%.2f Q%.2f T%.2f U%.2f "
             "STEPS P%ld Q%ld T%ld U%ld\r\n",
             pan_min_deg, pan_max_deg, tilt_min_deg, tilt_max_deg,
             pan_min_steps, pan_max_steps, tilt_min_steps, tilt_max_steps);
    uart_send(buf);
    uart_send_ok();
}

/**
 * M17：使能所有步进电机
 * 格式：M17
 */
static void cmd_m17(const char *args)
{
    stepper_ledc_set_enable(s_ctx.pan, true);
    stepper_ledc_set_enable(s_ctx.tilt, true);
    uart_send_ok();
}

/**
 * M18：禁用所有步进电机
 * 格式：M18
 */
static void cmd_m18(const char *args)
{
    stepper_ledc_move_speed(s_ctx.pan, 0.0f);
    stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
    vTaskDelay(1);

    // 禁能前保存 PGV 的最终编码器位置，重新 M17 后不会从旧坐标继续运行。
    bool sync_ok = sync_encoder_positions(s_ctx.pan_velocity_mode,
                                          s_ctx.tilt_velocity_mode, true);
    s_ctx.pan_velocity_mode = false;
    s_ctx.tilt_velocity_mode = false;
    s_ctx.pan_velocity_sign = 0;
    s_ctx.tilt_velocity_sign = 0;
    stepper_ledc_set_enable(s_ctx.pan, false);
    stepper_ledc_set_enable(s_ctx.tilt, false);
    if (!sync_ok) {
        uart_send_error("Encoder sync failed");
        return;
    }
    uart_send_ok();
}

/**
 * G28：回归原点（使用限位开关找零）
 * 格式：G28 [S<速度>]
 *
 * 流程：
 *   1. 向负方向低速移动 Pan、向正方向移动 Tilt，直到限位开关触发
 *   2. 触发后立即停止对应电机
 *   3. 两轴都找到限位后，按 M220 回退步数以半速**并行**脱离开关
 *   4. 重置位置计数器为 0
 *   5. 以回退后位置为编码器零点（M114 的 ENC 归零）
 */
static void cmd_g28(const char *args)
{
    // 默认速度（步/秒）
    float speed = 2000.0f;

    param_t params[] = {
        { .key = 'S' },
    };
    const int N = sizeof(params) / sizeof(params[0]);

    int parse_result = parse_params(args, "S", params, N);
    if (parse_result != 0) {
        uart_send_error("Bad G28 params");
        return;
    }
    if (params[0].found) {
        speed = params[0].fval;
        if (speed < STEP_SPEED_MIN_HZ) speed = STEP_SPEED_MIN_HZ;
    }

    ESP_LOGI(TAG, "G28: Homing at S=%.0f", speed);

    // ---- 步骤 0: 使能电机 ----
    stepper_ledc_set_enable(s_ctx.pan, true);
    stepper_ledc_set_enable(s_ctx.tilt, true);

    // ---- 步骤 1: 停止任何当前运动 ----
    stepper_ledc_move_speed(s_ctx.pan, 0.0f);
    stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
    s_ctx.pan_velocity_mode = false;
    s_ctx.tilt_velocity_mode = false;
    s_ctx.pan_velocity_sign = 0;
    s_ctx.tilt_velocity_sign = 0;

    // ---- 步骤 2: Pan 负向、Tilt 正向移动，轮询限位开关 ----
    bool pan_homed = limit_switch_is_triggered(s_ctx.pan_limit);
    bool tilt_homed = limit_switch_is_triggered(s_ctx.tilt_limit);

    long pan_start_steps = stepper_ledc_get_steps(s_ctx.pan);
    long tilt_start_steps = stepper_ledc_get_steps(s_ctx.tilt);
    int elapsed_ms = 0;

    if (!pan_homed) stepper_ledc_move_speed(s_ctx.pan, -speed);
    if (!tilt_homed) stepper_ledc_move_speed(s_ctx.tilt, speed);

    while (!pan_homed || !tilt_homed) {
        vTaskDelay(pdMS_TO_TICKS(G28_POLL_MS));
        elapsed_ms += G28_POLL_MS;

        if (!pan_homed && limit_switch_is_triggered(s_ctx.pan_limit)) {
            ESP_LOGI(TAG, "G28: Pan limit triggered");
            stepper_ledc_move_speed(s_ctx.pan, 0.0f);
            pan_homed = true;
        }
        if (!tilt_homed && limit_switch_is_triggered(s_ctx.tilt_limit)) {
            ESP_LOGI(TAG, "G28: Tilt limit triggered");
            stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
            tilt_homed = true;
        }

        long pan_moved = labs(stepper_ledc_get_steps(s_ctx.pan) - pan_start_steps);
        long tilt_moved = labs(stepper_ledc_get_steps(s_ctx.tilt) - tilt_start_steps);
        if (elapsed_ms >= G28_TIMEOUT_MS ||
            pan_moved >= G28_MAX_SEEK_STEPS || tilt_moved >= G28_MAX_SEEK_STEPS) {
            stepper_ledc_move_speed(s_ctx.pan, 0.0f);
            stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
            uart_send_error("G28 timeout or travel limit");
            ESP_LOGE(TAG, "G28 aborted: elapsed=%dms pan=%ld tilt=%ld",
                     elapsed_ms, pan_moved, tilt_moved);
            return;
        }
    }

    // ---- 步骤 3: 回退脱离开关（半速，步数由 M220 设置，双轴并行）----
    const float backoff_speed = speed / 2.0f;

    // 双轴同时回退脱离开关：Pan 正向、Tilt 负向；
    // move_both 对 steps<=0 的轴自动跳过（兼容某轴 backoff=0）。
    stepper_ledc_move_both(s_ctx.pan, s_ctx.pan_backoff, true,
                           s_ctx.tilt, s_ctx.tilt_backoff, false,
                           backoff_speed);

    // ---- 步骤 4: 重置位置计数器 ----
    stepper_ledc_set_position(s_ctx.pan, 0);
    stepper_ledc_set_position(s_ctx.tilt, 0);
    s_ctx.pan_position  = 0;
    s_ctx.tilt_position = 0;

    // ---- 步骤 5: 以 M220 回退后的位置为两轴编码器零点 ----
    // G28 完成（机械回退到位）后，两轴编码器当前位置标定为 0°，
    // 此后 M114 的 ENC 即相对云台回退零点的角度。
    if (s_ctx.encoder) {
        if (encoder_zero_all(s_ctx.encoder) != ESP_OK) {
            uart_send_error("Encoder zero failed");
            return;
        }
        ESP_LOGI(TAG, "G28: encoders zeroed at homed position");
    }

    uart_send_ok();
    ESP_LOGI(TAG, "G28: Homing complete");
}

/**
 * M220：设置 G28 回零后各轴回退步数（掉电保存到 NVS）
 * 格式：M220 P<pan步数> T<tilt步数>（省略参数仅查询）
 */
static void cmd_m220(const char *args)
{
    param_t params[] = {
        { .key = 'P' },
        { .key = 'T' },
    };
    const int N = sizeof(params) / sizeof(params[0]);

    if (parse_params(args, "PT", params, N) != 0) {
        uart_send_error("Bad M220 params");
        return;
    }

    {
        int old_pan_backoff = s_ctx.pan_backoff;
        int old_tilt_backoff = s_ctx.tilt_backoff;
        int new_pan_backoff = s_ctx.pan_backoff;
        int new_tilt_backoff = s_ctx.tilt_backoff;
        if (params[0].found) {
            new_pan_backoff = params[0].ival;
            if (new_pan_backoff < 0 || new_pan_backoff > NVS_PARAMS_BACKOFF_MAX) {
                uart_send_error("Pan backoff out of range");
                return;
            }
        }
        if (params[1].found) {
            new_tilt_backoff = params[1].ival;
            if (new_tilt_backoff < 0 || new_tilt_backoff > NVS_PARAMS_BACKOFF_MAX) {
                uart_send_error("Tilt backoff out of range");
                return;
            }
        }

        s_ctx.pan_backoff = new_pan_backoff;
        s_ctx.tilt_backoff = new_tilt_backoff;

        // 同步到参数存储并提交 Flash（掉电保持）
        nvs_params_set_pan_backoff(s_ctx.pan_backoff);
        nvs_params_set_tilt_backoff(s_ctx.tilt_backoff);
        if (nvs_params_save() != ESP_OK) {
            ESP_LOGE(TAG, "M220: failed to save params to NVS");
            s_ctx.pan_backoff = old_pan_backoff;
            s_ctx.tilt_backoff = old_tilt_backoff;
            nvs_params_set_pan_backoff(old_pan_backoff);
            nvs_params_set_tilt_backoff(old_tilt_backoff);
            uart_send_error("NVS save failed");
            return;
        }
    }

    char buf[UART_BUF_SIZE];
    snprintf(buf, sizeof(buf), "M220 P%d T%d\r\n", s_ctx.pan_backoff, s_ctx.tilt_backoff);
    uart_send(buf);
    uart_send_ok();
    ESP_LOGI(TAG, "M220: P=%d T=%d", s_ctx.pan_backoff, s_ctx.tilt_backoff);
}

/**
 * PGV：速度模式指令（PG 家族子命令）
 * 格式：PGV P<速度> T<速度>
 * 
 * 同时控制 Pan / Tilt 电机速度（Hz），正数 = 正转，负数 = 反转，0 = 停止。
 * 两个参数可单独或同时提供；不提供的轴保持原速不变。
 * 映射到 LEDC move_speed（连续调速，类直流电机，非阻塞 → 两轴并行）。
 */
static void cmd_pgv(const char *args)
{
    param_t params[] = {
        { .key = 'P' },
        { .key = 'T' },
    };
    const int N = sizeof(params) / sizeof(params[0]);

    if (parse_params(args, "PT", params, N) != 0) {
        uart_send_error("Bad PGV params");
        return;
    }
    if (!params[0].found && !params[1].found) {
        uart_send_error("Missing P or T");
        return;
    }

    bool ok = true;
    bool limit_error = false;
    if (params[0].found) {
        float speed = normalize_pgv_speed(params[0].fval);
        ESP_LOGD(TAG, "PGV: P=%.0f", speed);
        if (fabsf(speed) < 0.5f) {
            bool was_active = s_ctx.pan_velocity_mode;
            stepper_ledc_move_speed(s_ctx.pan, 0.0f);
            if (was_active) {
                vTaskDelay(1);
                ok = sync_encoder_axis(ENCODER_PAN, true) && ok;
            }
            s_ctx.pan_velocity_mode = false;
            s_ctx.pan_velocity_sign = 0;
        } else {
            if (!s_ctx.pan_velocity_mode ||
                s_ctx.pan_velocity_sign != (speed > 0.0f ? 1 : -1)) {
                if (s_ctx.pan_velocity_mode) {
                    stepper_ledc_move_speed(s_ctx.pan, 0.0f);
                    vTaskDelay(1);
                }
                float deg = 0.0f;
                bool synced = read_encoder_axis(ENCODER_PAN, true, &deg);
                ok = synced && ok;
                if (synced && !velocity_allowed(ENCODER_PAN, deg, speed)) {
                    stepper_ledc_move_speed(s_ctx.pan, 0.0f);
                    s_ctx.pan_velocity_mode = false;
                    s_ctx.pan_velocity_sign = 0;
                    limit_error = true;
                    ok = false;
                } else if (synced) {
                    s_ctx.pan_velocity_sign = speed > 0.0f ? 1 : -1;
                    s_ctx.pan_velocity_mode = true;
                }
            }
            if (s_ctx.pan_velocity_mode) stepper_ledc_move_speed(s_ctx.pan, speed);
        }
    }
    if (params[1].found) {
        float speed = normalize_pgv_speed(params[1].fval);
        ESP_LOGD(TAG, "PGV: T=%.0f", speed);
        if (fabsf(speed) < 0.5f) {
            bool was_active = s_ctx.tilt_velocity_mode;
            stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
            if (was_active) {
                vTaskDelay(1);
                ok = sync_encoder_axis(ENCODER_TILT, true) && ok;
            }
            s_ctx.tilt_velocity_mode = false;
            s_ctx.tilt_velocity_sign = 0;
        } else {
            if (!s_ctx.tilt_velocity_mode ||
                s_ctx.tilt_velocity_sign != (speed > 0.0f ? 1 : -1)) {
                if (s_ctx.tilt_velocity_mode) {
                    stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
                    vTaskDelay(1);
                }
                float deg = 0.0f;
                bool synced = read_encoder_axis(ENCODER_TILT, true, &deg);
                ok = synced && ok;
                if (synced && !velocity_allowed(ENCODER_TILT, deg, speed)) {
                    stepper_ledc_move_speed(s_ctx.tilt, 0.0f);
                    s_ctx.tilt_velocity_mode = false;
                    s_ctx.tilt_velocity_sign = 0;
                    limit_error = true;
                    ok = false;
                } else if (synced) {
                    s_ctx.tilt_velocity_sign = speed > 0.0f ? 1 : -1;
                    s_ctx.tilt_velocity_mode = true;
                }
            }
            if (s_ctx.tilt_velocity_mode) stepper_ledc_move_speed(s_ctx.tilt, speed);
        }
    }
    if (!ok) {
        uart_send_error(limit_error ? "Encoder limit" : "Encoder sync failed");
        return;
    }
    // PGV 允许高频发送，成功时不回 OK，避免上位机 UART 接收缓存持续堆积。
}

// ====================================================================
// 命令注册表
// 在此添加新的命令家族。
// ====================================================================
typedef struct {
    const char *name;
    void (*handler)(const char *args);
} command_entry_t;

static const command_entry_t s_commands[] = {
    { "G28",  cmd_g28 },
    { "M220", cmd_m220 },
    { "M221", cmd_m221 },
    { "PG",   cmd_pg },
    { "M114", cmd_m114 },
    { "M17",  cmd_m17 },
    { "M18",  cmd_m18 },
    // --- 在此添加新命令家族 ---
    // { "XX", cmd_xx },
};

static const int s_num_commands = sizeof(s_commands) / sizeof(s_commands[0]);

// ====================================================================
// 命令分发器
// 使用前缀匹配："PG1" 匹配注册的 "PG"
// ====================================================================
static void trim_command_suffix(char *line)
{
    size_t len = strlen(line);

    // 去掉实际的行尾空白。
    while (len > 0 && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }

    // 兼容串口工具发送的字面量 "\\r\\n"。
    while (len >= 2 && line[len - 2] == '\\' &&
           (line[len - 1] == 'r' || line[len - 1] == 'n')) {
        len -= 2;
        while (len > 0 && isspace((unsigned char)line[len - 1])) {
            line[--len] = '\0';
        }
        line[len] = '\0';
    }
}

static void dispatch(char *line)
{
    trim_command_suffix(line);

    const char *p = line;
    while (*p && !isspace((unsigned char)*p)) p++;
    int name_len = p - line;
    if (name_len == 0) {
        uart_send_error("Empty command");
        return;
    }

    for (int i = 0; i < s_num_commands; i++) {
        size_t reg_len = strlen(s_commands[i].name);
        bool is_pg_family = (strcmp(s_commands[i].name, "PG") == 0);
        bool name_matches = ((size_t)name_len == reg_len) ||
                            (is_pg_family && (size_t)name_len > reg_len);
        if (name_matches &&
            strncasecmp(line, s_commands[i].name, reg_len) == 0) {
            if (s_ctx.motion_lock) xSemaphoreTake(s_ctx.motion_lock, portMAX_DELAY);
            s_commands[i].handler(line + reg_len);
            if (s_ctx.motion_lock) xSemaphoreGive(s_ctx.motion_lock);
            return;
        }
    }

    ESP_LOGW(TAG, "Unknown command: '%.*s'", name_len, line);
    uart_send_error("Unknown command");
}

// ====================================================================
// UART 任务（行读取器）
// ====================================================================
static void uart_task(void *arg)
{
    char line_buf[UART_BUF_SIZE];
    int line_pos = 0;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_send("Pan-Tilt Ready\r\n");
    ESP_LOGI(TAG, "UART ready (TX=%d, RX=%d, %d baud)", UART_TX_GPIO, UART_RX_GPIO, UART_BAUD_RATE);

    while (1) {
        uint8_t byte;
        int len = uart_read_bytes(UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(UART_RX_TIMEOUT_MS));
        if (len <= 0) continue;

        char c = (char)byte;

        if (c == '\n') {
            line_buf[line_pos] = '\0';
            if (line_pos > 0 && line_buf[line_pos - 1] == '\r') {
                line_buf[line_pos - 1] = '\0';
            }
            if (line_pos > 0) {
                ESP_LOGI(TAG, "RX: '%s'", line_buf);
                dispatch(line_buf);
            }
            line_pos = 0;

        } else if (c == '\r') {
            // 忽略

        } else if (line_pos < (UART_BUF_SIZE - 1)) {
            line_buf[line_pos++] = c;

        } else {
            line_pos = 0;
            uart_send_error("Line too long");
        }
    }
}

// ====================================================================
// 公开 API
// ====================================================================
void gcode_ledc_start(stepper_ledc_handle_t pan_handle,
                      stepper_ledc_handle_t tilt_handle,
                      limit_switch_handle_t pan_limit_handle,
                      limit_switch_handle_t tilt_limit_handle,
                      encoder_t *encoder)
{
    s_ctx.pan          = pan_handle;
    s_ctx.tilt         = tilt_handle;
    s_ctx.pan_limit    = pan_limit_handle;
    s_ctx.tilt_limit   = tilt_limit_handle;
    s_ctx.encoder      = encoder;
    s_ctx.pan_position = 0;
    s_ctx.tilt_position = 0;
    s_ctx.pan_velocity_mode = false;
    s_ctx.tilt_velocity_mode = false;
    s_ctx.pan_velocity_sign = 0;
    s_ctx.tilt_velocity_sign = 0;
    s_motion_limits = (motion_limits_t){
        .pan_min_deg = PAN_MIN_DEG,
        .pan_max_deg = PAN_MAX_DEG,
        .tilt_min_deg = TILT_MIN_DEG,
        .tilt_max_deg = TILT_MAX_DEG,
    };
    s_ctx.motion_lock = xSemaphoreCreateMutex();
    if (!s_ctx.motion_lock) {
        ESP_LOGE(TAG, "failed to create motion mutex");
        return;
    }
    // G28 回退参数从 NVS 加载（掉电保持，默认 P3490 T2940）
    s_ctx.pan_backoff  = nvs_params_get_pan_backoff();
    s_ctx.tilt_backoff = nvs_params_get_tilt_backoff();

    xTaskCreate(uart_task, "gcode_ledc", 4096, NULL, 5, NULL);
}
