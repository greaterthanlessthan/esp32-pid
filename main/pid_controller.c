/*
Creates a PID control loop that runs a in a task.
Gains, initializing, etc. can be controlled by other taskes.


*/

#include "pid_controller.h"

#include <float.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

// Dummy pointer. Value is meaningless
bool dummy_ptr_bool_val = false;
bool *dummy_ptr_bool = &dummy_ptr_bool_val;

void
app_main (void)
{
  // struct pid_controller_struct pid
  //    = create_pid_control_struct (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
  // pid.advanced.windup_control_mode;
}

struct pid_controller_struct
create_pid_control_struct (float *pv, float sp, float *cv, float kp, float ki,
                           float kd, bool control_action)
{
  struct pid_controller_struct pid = {
      .pv = pv,
      .sp = sp,
      .cv = cv,

      .kp = kp,
      .ki = ki,
      .kd = kd,

      .control_action = control_action,

      .limits =
          {
              .enable_cv_limits = false,
              .enable_cv_roc_limits = false,
              .enable_sp_limits = false,
              .enable_sp_roc_limits = false,
              .cv_limit_hi = FLT_MAX,
              .cv_limit_lo = FLT_MIN,
              .pv_limit_hi = FLT_MAX,
              .pv_limit_lo = FLT_MIN,
              .sp_limit_hi = FLT_MAX,
              .sp_limit_lo = FLT_MIN,
          },

      .control =
          {
              .init = true,
              .init_value = 0.0,
              .mutex = xSemaphoreCreateMutex(),
          },

      .advanced =
          {
              .sample_period = PID_SAMPLE_PERIOD,
              .windup_control_mode = 0,
              .windup_pos_limit = 0.0,
              .windup_neg_limit = 0.0,
              .disable_pos_windup = false,
              .disable_neg_windup = false,
          },
      // everything else set when PID task is started. Undefined behavior occurs
      // before then.
  };

  return pid;
};

// PID Controller
void
pid_controller (struct pid_controller_struct *pid)
{
  bool init;
  struct pid_controller_struct pid_wrk;

  float sp_wrk;
  float sp_wrk_old = 0.0;

  float integral;
  float last_err;

  float cv_wrk;
  float cv_roc_ms;

  TickType_t xLastWakeTime = xTaskGetTickCount ();

  for (;;)
    {
      // Create copy of pid struct. This allows values to be changed with mutex
      // continue using pid for pv and outputs, as nothing else should modify
      // those
      if (pid->control.mutex)
        {
          if (xSemaphoreTake (pid->control.mutex,
                              pdMS_TO_TICKS (pid->advanced.sample_period / 2))
              == pdTRUE)
            {
              pid_wrk = *pid;
              xSemaphoreGive (pid->control.mutex);
            };
        };

      // Delay to get sample period
      xTaskDelayUntil (&xLastWakeTime,
                       pdMS_TO_TICKS (pid->advanced.sample_period));
      xLastWakeTime = xTaskGetTickCount ();

      // Check for NaN errors. Init if they exist to avoid undefined behavior
      pid->status.pv_NaN = isnan (pid->pv) != 0 || isinf (pid->pv) != 0;
      pid->status.sp_NaN = isnan (pid->sp) != 0 || isinf (pid->sp) != 0;

      // Determine when to init
      init = pid->status.pv_NaN || pid->status.sp_NaN || pid_wrk.control.init;

      /* Process the setpoint
      Example:
      First sample: 100 to 200, limited to 4%/sec, period 1000ms. sp_wrk is
      200, and is then set to 104. sp_wrk_old is set to 104
      Second sample: 104 to 200. sp_wrk is set to 200, then 108.16, then
      sp_wrk_old is set to 108.16 */
      sp_wrk = hi_lo_limits_w_status (
          pid_wrk.sp, pid_wrk.limits.sp_limit_hi, pid_wrk.limits.sp_limit_lo,
          pid_wrk.limits.enable_sp_limits, &pid->status.sp_hi,
          &pid->status.sp_lo, &pid->status.sp_limited);

      sp_wrk = roc_limits_w_status (
          sp_wrk, sp_wrk_old, pid_wrk.advanced.sample_period,
          pid_wrk.limits.sp_roc_limit_hi, pid_wrk.limits.sp_roc_limit_lo,
          pid_wrk.limits.enable_sp_roc_limits && !init, dummy_ptr_bool,
          dummy_ptr_bool, &pid->status.sp_roc_limited);

      sp_wrk_old = sp_wrk;

      // Process the PV. Sets status only
      hi_lo_limits_w_status (
          *pid->pv, pid_wrk.limits.pv_limit_hi, pid_wrk.limits.pv_limit_lo,
          false, pid->status.pv_hi, pid->status.pv_lo, dummy_ptr_bool);

      // Calculate the error
      last_err = pid->err;
      pid->err
          = pid_wrk.control_action ? *pid->pv - sp_wrk : sp_wrk - *pid->pv;

      // Calculate the components
      pid->p_out = pid->err * pid_wrk.kp;

      integral
          = init ? 0.0 : calc_i_out (&pid_wrk.advanced, pid->err, integral);
      pid->i_out = integral * pid_wrk.ki;

      pid->d_out = (pid->err - last_err) * pid_wrk.kd;

      // Calculate the total CV, then limit it
      cv_wrk = init ? pid_wrk.control.init_value
                    : pid->p_out + pid->i_out + pid->d_out;

      cv_wrk = hi_lo_limits_w_status (
          cv_wrk, pid_wrk.limits.cv_limit_hi, pid_wrk.limits.cv_limit_lo,
          pid_wrk.limits.enable_cv_limits, &pid->status.cv_hi,
          &pid->status.cv_lo, &pid->status.cv_limited);

      cv_wrk = roc_limits_w_status (
          cv_wrk, *pid->cv, pid_wrk.advanced.sample_period,
          pid_wrk.limits.cv_roc_limit_hi, pid_wrk.limits.cv_roc_limit_lo,
          pid_wrk.limits.enable_cv_roc_limits, &pid->status.cv_roc_hi,
          &pid->status.cv_roc_lo, &pid->status.cv_roc_limited);

      // One final check of NaN/inf before setting cv
      pid->status.cv_NaN = isnan (pid->cv) != 0 || isinf (pid->cv) != 0;
      *pid->cv = pid->status.cv_NaN ? pid_wrk.control.init_value : cv_wrk;
    };
};

// Calculate the integral component
float
calc_i_out (struct _pid_c_advanced *adv, float err, float integral)
{
  bool at_pos_limit = err > adv->windup_pos_limit || adv->disable_pos_windup;

  bool at_neg_limit = err < adv->windup_neg_limit || adv->disable_neg_windup;

  if (at_pos_limit || at_neg_limit)
    {
      switch (adv->windup_control_mode)
        {
        case 2: // integrator behaves as though err was equal to limit

          return integral
                 + (at_pos_limit ? adv->windup_pos_limit
                                 : adv->windup_neg_limit);

        case 3: // same as 1, but integrator value is also cleared

          return 0.0;

        default: // 0 (manually disabled) or 1 (auto or manually disabled).
                 // Pause integration

          return integral;
        }
    };

  return integral + err;
};

// Clamps value to hi and lo limits if enabled, and sets alarm statuses
float
hi_lo_limits_w_status (float value, float hi, float lo, bool limit_en,
                       bool *hi_status, bool *lo_status, bool *limited_status)
{
  // Value is within limits
  if (hi > value > lo)
    {
      *hi_status = false;
      *lo_status = false;
      *limited_status = false;
    }
  // Value is lo
  else if (value <= lo)
    {
      value = limit_en ? lo : value;
      *hi_status = false;
      *lo_status = true;

      *limited_status = !limit_en;
    }
  // Value is hi
  else
    {
      value = limit_en ? hi : value;
      *hi_status = true;
      *lo_status = false;
      *limited_status = !limit_en;
    };
  return value;
};

// Clamps value to hi and lo limits based on period if enabled, and sets alarm
// statuses
float
roc_limits_w_status (float new, float old, uint16_t period_ms, float hi_roc,
                     float lo_roc, bool limit_roc_en, bool *hi_roc_status,
                     bool *lo_roc_status, bool *roc_limited_status)
{
  // Calculate rate of change in %/s
  int roc_perc_s = ((new - old) / old) * 100.0 * (1000.0 / period_ms);

  // Within limits
  if (hi_roc > roc_perc_s > lo_roc)
    {
      *hi_roc_status = false;
      *lo_roc_status = false;
      *roc_limited_status = false;
    }
  // ROC is lo
  else if (roc_perc_s <= lo_roc)
    {

      new = limit_roc_en
                ? ((lo_roc * old) / (100.0 * (1000.0 / period_ms))) + old
                : new;
      *hi_roc_status = false;
      *lo_roc_status = true;
      *roc_limited_status = !limit_roc_en;
    }
  // ROC is hi
  else
    {
      new = limit_roc_en
                ? ((hi_roc * old) / (100.0 * (1000.0 / period_ms))) + old
                : new;
      *hi_roc_status = true;
      *lo_roc_status = false;
      *roc_limited_status = !limit_roc_en;
    };
  return new;
};
