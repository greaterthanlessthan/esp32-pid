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

void
app_main ()
{
  false;
  // struct pid_controller_struct pid
  //    = create_pid_control_struct (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
  // pid.advanced.windup_control_mode;
}

/**
 * @brief find the integral for the timeslice using the trapezoidal rule
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
 * @brief Takes a float value, compares it to its old value and clamps it to an
 * acceptable rate of change, and sets status bools
 *
 */
static float
check_limits (struct pid_limit lim, float new, float old, uint16_t period_ms,
              struct pid_status *status)
{
  // Calculate rate of change in %/s
  float roc_perc_s = ((new - old) / old) * 100.0 * (1000.0 / period_ms);
  float value;

  // Set status
  status->roc_hi = roc_perc_s > lim.roc_lim_hi;
  status->roc_lo = roc_perc_s < lim.roc_lim_lo;
  status->rate_limited = (status->roc_hi || status->roc_lo) && lim.en_roc_lim;

  // Guard if not limited
  if (!status->rate_limited)
    {
      value = new;
    }

  // Calculate limited value
  roc_perc_s = status->roc_hi ? lim.roc_lim_hi : lim.roc_lim_lo;
  value = (roc_perc_s * old) / (100.0 * (1000.0 / period_ms)) + old;

  // Set status
  status->hi = value > lim.lim_hi;
  status->lo = value < lim.lim_lo;
  status->limited = (status->hi || status->lo) && lim.en_lim;

  // Guard if not limited
  if (!status->limited)
    {
      return value;
    }

  return status->hi ? lim.lim_hi : lim.lim_lo;
}

pid_controller_struct
create_pid_control_struct (float sp, float kp, float ki, float kd,
                           bool control_action)
{
  struct pid_c_controls pid_control_default = {
    .init = true,
    .init_value = 0.0,
    .mutex = xSemaphoreCreateMutex (),
  };

  struct pid_c_advanced pid_advanced_default = {
    .period = PID_SAMPLE_PERIOD,
    .wndup_mode = NORMAL,
    .wndup_pos_lim = FLT_MAX,
    .wndup_neg_lim = FLT_MIN,
    .dis_pos_wndup = false,
    .dis_neg_wndup = false,
  };

  struct pid_c_limits pid_limits_default = {
    .cv = { .en_lim = false,
            .lim_hi = FLT_MAX,
            .lim_lo = FLT_MIN,
            .en_roc_lim = false,
            .roc_lim_hi = FLT_MAX,
            .roc_lim_lo = FLT_MIN },

    .pv = { .en_lim = false,
            .lim_hi = FLT_MAX,
            .lim_lo = FLT_MIN,
            .en_roc_lim = false,
            .roc_lim_hi = FLT_MAX,
            .roc_lim_lo = FLT_MIN },

    .sp = { .en_lim = false,
            .lim_hi = FLT_MAX,
            .lim_lo = FLT_MIN,
            .en_roc_lim = false,
            .roc_lim_hi = FLT_MAX,
            .roc_lim_lo = FLT_MIN },
  };

  pid_controller_struct pid = {
    .pv = xQueueCreate (1, sizeof (float)),
    .sp = sp,
    .cv = xQueueCreate (1, sizeof (float)),

    .kp = kp,
    .ki = ki,
    .kd = kd,

    .control_action = control_action,

    .control = pid_control_default,
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
  float last_pv_t = 0.0;

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
      xTaskDelayUntil (&xLastWakeTime, pdMS_TO_TICKS (pid_t.adv.period));
      xLastWakeTime = xTaskGetTickCount ();

      // Receive cv
      xQueueReceive (pid->pv, &pv, 0);

      // Check for NaN errors. Init if they exist to avoid undefined behavior
      pid->sts.pv.NaN = isnan (pv) != 0 || isinf (pv) != 0;
      pid->sts.sp.NaN = isnan (pid->sp) != 0 || isinf (pid->sp) != 0;

      // Determine when to init
      init = pid->sts.pv.NaN || pid->sts.sp.NaN || pid_t.control.init;

      // Process the setpoint and process value
      sp_t = check_limits (pid->lim.sp, pid_t.sp, last_sp_t, pid_t.adv.period,
                           &pid->sts.sp);

      pv = check_limits (pid->lim.pv, pv, last_pv_t, pid_t.adv.period,
                         &pid->sts.pv);
      last_sp_t = sp_t;
      last_pv_t = pv;

      // Calculate the error
      last_err = pid->err;
      pid->err = pid_t.control_action ? pv - sp_t : sp_t - pv;

      // Calculate the components
      pid->p_out = pid->err * pid_t.kp;

      mode = integral_mode_used (
          pid_t.adv.wndup_mode, pid->err, pid_t.adv.wndup_pos_lim,
          pid_t.adv.wndup_neg_lim, pid_t.adv.dis_pos_wndup,
          pid_t.adv.dis_neg_wndup);
      integral = init ? 0.0
                      : calc_i_out (pid->err, last_err, pid_t.adv.period,
                                    integral, mode, pid_t.adv.wndup_pos_lim,
                                    pid_t.adv.wndup_neg_lim);
      pid->i_out = integral * pid_t.ki;

      pid->d_out = (pid->err - last_err) * pid_t.kd;

      // Calculate the total CV, then limit it
      cv_t = pid->p_out + pid->i_out + pid->d_out;
      cv_t = init ? pid_t.control.init_value : cv_t;

      cv_t = check_limits (pid->lim.cv, cv_t, cv, pid_t.adv.period,
                           &pid->sts.cv);

      // One final check of NaN/inf before setting cv
      pid->sts.cv.NaN = isnan (cv_t) != 0 || isinf (cv_t) != 0;
      cv = pid->sts.cv.NaN ? pid_t.control.init_value : cv_t;

      xQueueOverwrite (pid->cv, &cv);
    }
}
