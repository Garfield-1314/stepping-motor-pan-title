#ifndef GCODE_LEDC_H
#define GCODE_LEDC_H

#include "stepper_ledc.h"
#include "limit_switch.h"
#include "encoder.h"

/**
 * @brief Start the UART G-code command layer (blocking task)
 *
 * Commands over UART0:
 *   M17                        Enable both motors
 *   M18                        Stop pulses and disable both motors
 *   G28 [S<speed>]             Home to limit switches (Pan backward, Tilt forward);
 *                              back off by M220 steps and zero the encoder
 *                              (M114 ENC becomes 0 at homed position)
 *   M220 P<backoff> T<backoff> Set G28 backoff steps (also queries current value)
 *   M221 P<pan_min_deg> Q<pan_max_deg> T<tilt_min_deg> U<tilt_max_deg>
 *                               Set/query runtime encoder angle limits (RAM only);
 *                               step limits are derived automatically
 *   PG1 P<abs> T<abs> S<speed>    Constant-speed absolute move (both axes parallel)
 *   PG2 P<abs> T<abs> S<speed> A<accel>  Accel/decel absolute move (both axes parallel)
 *   PGV P<speed> T<speed>      Continuous speed mode; position follows encoder
 *                               Pan steps -3500..3500, Tilt steps -2500..2500;
 *                               encoder limits Pan ±87°, Tilt ±62°
 *   M114                       Query state -> "MP: <pan_steps> <tilt_steps> [ENC: <pan_deg> <tilt_deg>]"
 *                               (ENC appended when encoder is attached)
 *
 * @param pan        Pan motor handle
 * @param tilt       Tilt motor handle
 * @param pan_limit  Pan limit switch handle (G28 homing)
 * @param tilt_limit Tilt limit switch handle (G28 homing)
 * @param encoder    Dual encoder manager (may be NULL if not attached)
 */
void gcode_ledc_start(stepper_ledc_handle_t pan, stepper_ledc_handle_t tilt,
                      limit_switch_handle_t pan_limit, limit_switch_handle_t tilt_limit,
                      encoder_t *encoder);

/**
 * @brief 更新 PGV 期间的编码器位置
 *
 * 由编码器采样任务调用；只有处于 PGV 的轴会被更新。
 */
void gcode_ledc_update_encoder_position(float pan_deg, bool pan_valid,
                                        float tilt_deg, bool tilt_valid);

#endif // GCODE_LEDC_H
