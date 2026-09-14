#ifndef STEPPER_LEDC_H
#define STEPPER_LEDC_H

#include "driver/gpio.h"
#include <stdbool.h>

// 速度安全区间（12 位分辨率硬件可达 ~9.8kHz，留余量取 9000；下限 50 Hz）
#define STEP_SPEED_MIN_HZ  50.0f
#define STEP_SPEED_MAX_HZ  9000.0f

typedef struct {
    gpio_num_t step_pin;   // STEP 引脚（LEDC 输出方波 + 同引脚 GPIO 中断计数）
    gpio_num_t dir_pin;    // DIR 引脚
    gpio_num_t en_pin;     // EN 使能引脚（低有效），GPIO_NUM_NC 表示未连接
    gpio_num_t ms1_pin;    // 细分引脚 MS1（可选）
    gpio_num_t ms2_pin;    // 细分引脚 MS2（可选）
    gpio_num_t ms3_pin;    // 细分引脚 MS3（可选）
} stepper_ledc_config_t;

typedef int stepper_ledc_handle_t;

/**
 * @brief 初始化 A4988 步进电机实例（LEDC 驱动）
 *
 * LEDC 输出方波到 STEP 引脚，同一引脚配置 GPIO 上升沿中断用于软件计数。
 * MS 引脚（若提供）固定 1/16 细分。
 *
 * @param config 引脚配置
 * @return 电机句柄，失败返回 -1
 */
stepper_ledc_handle_t stepper_ledc_init(const stepper_ledc_config_t *config);

/**
 * @brief 精确定位：以指定速度阻塞移动指定步数
 *
 * 基于软件计数，走到目标步数自动停止（LEDC pause）。
 *
 * @param motor    电机句柄
 * @param steps    步数（必须 > 0）
 * @param speed_hz 脉冲频率（步/秒，必须 > 0）
 * @param forward  方向（true=正向, false=反向）
 */
void stepper_ledc_move_step(stepper_ledc_handle_t motor, int steps, float speed_hz, bool forward);

/**
 * @brief 加减速定位：以指定速度和加速度，梯形曲线移动到目标步数（阻塞）
 *
 * 从起始速度（STEP_START_HZ）起步，按 accel 加速至 speed_hz，
 * 接近目标时按同加速度平滑减速，精确停在目标步数。
 * 全程通过 move_speed 底层原语控制 LEDC 频率。
 *
 * @param motor    电机句柄
 * @param steps    步数（必须 > 0）
 * @param speed_hz 目标巡航速度（步/秒，必须 > 0）
 * @param accel    加速度（步/秒²，必须 > 0）
 * @param forward  方向（true=正向, false=反向）
 */
void stepper_ledc_move_step_accel(stepper_ledc_handle_t motor, int steps,
                                  float speed_hz, float accel, bool forward);

/**
 * @brief 双轴并行匀速定位（同一循环联合轮询，两轴同时启停）
 *
 * 两电机使用独立 LEDC 定时器/通道，可同时输出脉冲。
 * 此函数同时启动两轴，在单循环中轮询两轴的软件计数，
 * 两轴都到达目标步数后同时停止。
 * 传入 steps<=0 的轴不参与移动。
 *
 * @param m1       电机 1 句柄
 * @param steps1   电机 1 步数（<=0 表示不移动）
 * @param fwd1     电机 1 方向（true=正向）
 * @param m2       电机 2 句柄
 * @param steps2   电机 2 步数（<=0 表示不移动）
 * @param fwd2     电机 2 方向（true=正向）
 * @param speed_hz 脉冲频率（步/秒，必须 > 0）
 */
void stepper_ledc_move_both(stepper_ledc_handle_t m1, int steps1, bool fwd1,
                            stepper_ledc_handle_t m2, int steps2, bool fwd2,
                            float speed_hz);

/**
 * @brief 双轴并行加减速定位（同一 1ms 周期各自梯形调速，同时启停）
 *
 * 两轴各自从 STEP_START_HZ 起步，在同一个循环中按各自剩余距离
 * 独立计算梯形速度并输出，两轴都到达目标后同时停止。
 *
 * @param m1       电机 1 句柄
 * @param steps1   电机 1 步数（<=0 表示不移动）
 * @param fwd1     电机 1 方向（true=正向）
 * @param m2       电机 2 句柄
 * @param steps2   电机 2 步数（<=0 表示不移动）
 * @param fwd2     电机 2 方向（true=正向）
 * @param speed_hz 目标巡航速度（步/秒，必须 > 0）
 * @param accel    加速度（步/秒²，必须 > 0）
 */
void stepper_ledc_move_both_accel(stepper_ledc_handle_t m1, int steps1, bool fwd1,
                                  stepper_ledc_handle_t m2, int steps2, bool fwd2,
                                  float speed_hz, float accel);

/**
 * @brief 连续调速（类直流电机，非阻塞）
 *
 * @param motor    电机句柄
 * @param speed_hz 速度（Hz）：正=正转，负=反转，0=停止
 */
void stepper_ledc_move_speed(stepper_ledc_handle_t motor, float speed_hz);

/**
 * @brief 获取累计步数（软件计数）
 *
 * @param motor 电机句柄
 * @return 步数（正向为 +，反向为 -）
 */
long stepper_ledc_get_steps(stepper_ledc_handle_t motor);

/**
 * @brief 设置累计步数（用于编码器校准后的软件位置对齐）
 *
 * 调用前应先停止该电机的脉冲输出。
 */
void stepper_ledc_set_position(stepper_ledc_handle_t motor, long position);

/**
 * @brief 获取角度（1/16 细分下每微步 0.025°）
 *
 * @param motor 电机句柄
 * @return 角度（度）
 */
float stepper_ledc_get_angle(stepper_ledc_handle_t motor);

/**
 * @brief 位置计数归零
 *
 * @param motor 电机句柄
 */
void stepper_ledc_reset_position(stepper_ledc_handle_t motor);

/**
 * @brief 使能/禁用电机（EN 低电平有效）
 *
 * @param motor  电机句柄
 * @param enable true=使能, false=禁用
 */
void stepper_ledc_set_enable(stepper_ledc_handle_t motor, bool enable);

#endif // STEPPER_LEDC_H
