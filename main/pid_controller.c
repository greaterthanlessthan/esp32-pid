#include "freertos/FreeRTOS.h"
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "pid_controller.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

//#include "esp_log.h"
//#include "esp_system.h"

/**
 * @brief Default values for PID struct. Override if used.
 * Recommended to use mutex when changing during runtime
 *
 */
static struct pid_c_advanced pid_advanced_default = {
  .sample_period = PID_SAMPLE_PERIOD,
  .wndup_mode = NORMAL,
  .wndup_pos_lim = 0.0,
  .wndup_neg_lim = 0.0,
  .dis_pos_wndup = false,
  .dis_neg_wndup = false,
};

static struct pid_c_limits pid_limits_default = {
  .en_cv_lim = false,
  .en_cv_roc_lim = false,
  .en_sp_lim = false,
  .en_sp_roc_lim = false,
  .cv_lim_hi = FLT_MAX,
  .cv_lim_lo = FLT_MIN,
  .pv_lim_hi = FLT_MAX,
  .pv_lim_lo = FLT_MIN,
  .sp_lim_hi = FLT_MAX,
  .sp_lim_lo = FLT_MIN,
};

static struct pid_c_controls
pid_control_default ()
{
  struct pid_c_controls controls = {
    .init = true,
    .init_value = 0.0,
    .mutex = xSemaphoreCreateMutex (),

  };
  return controls;
}

/**
 * @brief find the integral for the timeslice using the trapezoidal rule
 *
 * @param err current calculated at this time
 * @param err_prev previous error
 * @param pariod time elapsted between err calculations, in ms. Converted to s
 * in formula
 *
 * @return 0.5h(e1+e2)
 */
static float
calc_trapezoid_integral (float err, float err_prev, uint16_t period)
{
  return 0.5 * (period / 1000.0) * (err + err_prev);
}

/**
 * @brief takes error, checks limits, and returns the mode to use when
 * integrating
 *
 * @param mode integral windup
 * @param err
 * @param pos_limit
 * @param neg_limit
 * @param disable_pos
 * @param disable_neg
 *
 * @return
 */
static enum integral_windup_mode
integral_mode_used (enum integral_windup_mode mode, float err, float pos_limit,
                    float neg_limit, bool disable_pos, bool disable_neg)
{
  disable_pos = disable_pos && (err > 0.0);
  disable_neg = disable_neg && (err < 0.0);
  bool at_pos_limit = err > pos_limit;
  bool at_neg_limit = err < neg_limit;

  // Limits not used in NORMAL
  if (mode == NORMAL && !(disable_pos || disable_neg))
    {
      return NORMAL;
    }

  // Clamped and cleared mode only triggered by limits, and have priority over
  // disable commands
  if (mode == CLAMPED && (at_pos_limit || at_neg_limit))
    {
      return at_pos_limit ? _CLAMPED_POS : _CLAMPED_NEG;
    }

  if (mode == CLEARED && (at_pos_limit || at_neg_limit))
    {
      return CLEARED;
    }

  // Limit triggers or disable commands can cause hold mode
  if (disable_pos || disable_neg)
    {
      return HELD;
    }

  if (mode == HELD && (at_pos_limit || at_neg_limit))
    {
      return HELD;
    }

  // Handle fallthrough
  return NORMAL;
}

/**
 * @brief Calculates integral, using windup control and trapezoidal rule
 *
 */
static float
calc_i_out (float err, float err_prev, uint16_t period, float integral,
            enum integral_windup_mode mode, float pos_limit, float neg_limit)
{
  switch (mode)
    {
    case NORMAL:
      // Normal integration
      return integral + calc_trapezoid_integral (err, err_prev, period);

    case _CLAMPED_POS:
      // integrator behaves as though err was equal to limit
      return integral + calc_trapezoid_integral (pos_limit, err_prev, period);
    case _CLAMPED_NEG:
      return integral + calc_trapezoid_integral (neg_limit, err_prev, period);

    case HELD:
      // Integrator pauses
      return integral;

    case CLEARED:
      // Integrator is cleared
      return 0.0;

    default:
      return integral + calc_trapezoid_integral (err, err_prev, period);
    }
}

/**
 * @brief Takes a float value, clamps it between two values if enabled, and
 * sets status bools
 *
 */
static float
hi_lo_limits_w_status (float value, float hi, float lo, bool limit_en,
                       bool *hi_status, bool *lo_status, bool *lim_status)
{
  *hi_status = value > hi;
  *lo_status = value < lo;
  *lim_status = (*hi_status || *lo_status) && limit_en;

  // Guard if not limited
  if (!*lim_status)
    {
      return value;
    }

  return *hi_status ? hi : lo;
}

/**
 * @brief Takes a float value, compares it to its old value and clamps it to an
 * acceptable rate of change, and sets status bools
 *
 */
static float
roc_limits_w_status (float new, float old, uint16_t period_ms, float hi_roc,
                     float lo_roc, bool lim_roc_en, bool *hi_roc_status,
                     bool *lo_roc_status, bool *roc_lim_status)
{
  // Calculate rate of change in %/s
  float roc_perc_s = ((new - old) / old) * 100.0 * (1000.0 / period_ms);

  // Set status
  *hi_roc_status = roc_perc_s > hi_roc;
  *lo_roc_status = roc_perc_s < lo_roc;
  *roc_lim_status = (*hi_roc_status || *lo_roc_status) && lim_roc_en;

  // Guard if not limited
  if (!*roc_lim_status)
    {
      return new;
    }

  // Calculate limited value
  roc_perc_s = *hi_roc_status ? hi_roc : lo_roc;
  return (roc_perc_s * old) / (100.0 * (1000.0 / period_ms)) + old;
}

void
app_main ()
{
  false;
  // struct pid_controller_struct pid
  //    = create_pid_control_struct (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
  // pid.advanced.windup_control_mode;
}

pid_controller_struct
create_pid_control_struct (float sp, float kp, float ki, float kd,
                           bool control_action)
{
  pid_controller_struct pid = {
    .pv = xQueueCreate (1, sizeof (float)),
    .sp = sp,
    .cv = xQueueCreate (1, sizeof (float)),

    .kp = kp,
    .ki = ki,
    .kd = kd,

    .control_action = control_action,

    .control = pid_control_default (),
    .adv = pid_advanced_default,
    .lim = pid_limits_default,
    // everything else set when PID task is started. Undefined behavior occurs
    // before then.
  };

  return pid;
}

void
pid_controller (pid_controller_struct *pid)
{
  bool init;
  pid_controller_struct pid_t; // pid temp

  float sp_t;            // sp temp
  float last_sp_t = 0.0; // sp temp old

  float integral = 0.0;
  enum integral_windup_mode mode;
  float last_err;

  float cv_t;

  float pv;
  float cv;

  // Dummy pointer. Value is meaningless
  bool dummy_ptr_bool_val = false;
  bool *dum_ptr = &dummy_ptr_bool_val;

  TickType_t xLastWakeTime = xTaskGetTickCount ();

  for (;;)
    {
      // Create working copy of pid struct. This allows values to be changed
      // with mutex. Continue using pid for pv and outputs, as nothing else
      // should modify those
      if (pid->control.mutex)
        {
          if (xSemaphoreTake (pid->control.mutex, 0) == pdTRUE)
            {
              pid_t = *pid;
              xSemaphoreGive (pid->control.mutex);
            }
        }

      // Delay to get sample period
      xTaskDelayUntil (&xLastWakeTime,
                       pdMS_TO_TICKS (pid_t.adv.sample_period));
      xLastWakeTime = xTaskGetTickCount ();

      // Receive cv
      xQueueReceive (pid->pv, &pv, 0);

      // Check for NaN errors. Init if they exist to avoid undefined behavior
      pid->sts.pv_NaN = isnan (pv) != 0 || isinf (pv) != 0;
      pid->sts.sp_NaN = isnan (pid->sp) != 0 || isinf (pid->sp) != 0;

      // Determine when to init
      init = pid->sts.pv_NaN || pid->sts.sp_NaN || pid_t.control.init;

      /* Process the setpoint
      Example:
      First sample: 100 to 200, limited to 4%/sec, period 1000ms. sp_wrk is
      200, and is then set to 104. sp_wrk_old is set to 104
      Second sample: 104 to 200. sp_wrk is set to 200, then 108.16, then
      sp_wrk_old is set to 108.16 */
      sp_t = hi_lo_limits_w_status (pid_t.sp, pid_t.lim.sp_lim_hi,
                                    pid_t.lim.sp_lim_lo, pid_t.lim.en_sp_lim,
                                    &pid->sts.sp_hi, &pid->sts.sp_lo,
                                    &pid->sts.sp_limited);

      sp_t = roc_limits_w_status (
          sp_t, last_sp_t, pid_t.adv.sample_period, pid_t.lim.sp_roc_lim_hi,
          pid_t.lim.sp_roc_lim_lo, pid_t.lim.en_sp_roc_lim && !init, dum_ptr,
          dum_ptr, &pid->sts.sp_roc_limited);

      last_sp_t = sp_t;

      // Process the PV. Sets status only
      hi_lo_limits_w_status (pv, pid_t.lim.pv_lim_hi, pid_t.lim.pv_lim_lo,
                             false, &pid->sts.pv_hi, &pid->sts.pv_lo, dum_ptr);

      // Calculate the error
      last_err = pid->err;
      pid->err = pid_t.control_action ? pv - sp_t : sp_t - pv;

      // Calculate the components
      pid->p_out = pid->err * pid_t.kp;

      mode = integral_mode_used (
          pid_t.adv.wndup_mode, pid->err, pid_t.adv.wndup_pos_lim,
          pid_t.adv.wndup_neg_lim, pid_t.adv.dis_pos_wndup,
          pid_t.adv.dis_neg_wndup);
      integral = init
                     ? 0.0
                     : calc_i_out (pid->err, last_err, pid_t.adv.sample_period,
                                   integral, mode, pid_t.adv.wndup_pos_lim,
                                   pid_t.adv.wndup_neg_lim);
      pid->i_out = integral * pid_t.ki;

      pid->d_out = (pid->err - last_err) * pid_t.kd;

      // Calculate the total CV, then limit it
      cv_t = init ? pid_t.control.init_value
                  : pid->p_out + pid->i_out + pid->d_out;

      cv_t = hi_lo_limits_w_status (
          cv_t, pid_t.lim.cv_lim_hi, pid_t.lim.cv_lim_lo, pid_t.lim.en_cv_lim,
          &pid->sts.cv_hi, &pid->sts.cv_lo, &pid->sts.cv_limited);

      cv_t = roc_limits_w_status (
          cv_t, cv, pid_t.adv.sample_period, pid_t.lim.cv_roc_lim_hi,
          pid_t.lim.cv_roc_lim_lo, pid_t.lim.en_cv_roc_lim,
          &pid->sts.cv_roc_hi, &pid->sts.cv_roc_lo, &pid->sts.cv_roc_limited);

      // One final check of NaN/inf before setting cv
      pid->sts.cv_NaN = isnan (cv_t) != 0 || isinf (cv_t) != 0;
      cv = pid->sts.cv_NaN ? pid_t.control.init_value : cv_t;

      xQueueOverwrite (pid->cv, &cv);
    }
}
