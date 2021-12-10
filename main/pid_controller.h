#ifndef SemaphoreHandle_t
#include <freertos/semphr.h>
#endif

#define PID_SAMPLE_PERIOD 20

/*
PID Controller Advanced -- Variables for advanced PID control
*/
struct _pid_c_advanced
{
  uint16_t sample_period; // PID control loop sample period in ms

  /* Windup control
  windup_control_mode = 0: limits not used
  windup_control_mode = 1: when err is above limit, integrator cannot integrate
  in that direction
  windup_control_mode = 2: integrator behaves as though err
  was equal to limit
  windup_control_mode = 3: same as 1, but integrator value
  up to this point is also cleared

  disable_pos_windup and disable_pos_windup function in any mode
  */
  uint8_t windup_control_mode;
  float windup_pos_limit; // when err is above this value, windup control takes
                          // effect
  float windup_neg_limit; // when err is below this value, windup control takes
                          // effect
  bool disable_pos_windup; // when true, integrator cannot integrate in the
                           // positive direction
  bool disable_neg_windup; // when true, integrator cannot integrate in the
                           // negative direction

  // Deadband
  // pv moving average
  // p and d setpoint weight
};

/*
PID Controller Limits
*/
struct _pid_c_limits
{
  bool enable_cv_limits; // when true, cv can't go above or below limits. In
                         // either case, alarm is set
  float cv_limit_hi;
  float cv_limit_lo;

  bool enable_cv_roc_limits;
  float cv_roc_limit_hi; // in %/s
  float cv_roc_limit_lo; // in %/s

  float pv_limit_hi;     // for alarming only
  float pv_limit_lo;     // for alarming only
  float pv_roc_limit_hi; // in %/s
  float pv_roc_limit_lo; // in %/s

  bool enable_sp_limits; // when true, sp can't go above or below limits. In
                         // either case, alarm is set
  float sp_limit_hi;
  float sp_limit_lo;

  bool enable_sp_roc_limits; // can help in windup control. Does not cause
                             // alarms
  float sp_roc_limit_hi;     // in %/s
  float sp_roc_limit_lo;     // in %/s
};

/*
PID controller controls -- Value intended to be changed during runtime for
control of the PID controller
*/
struct _pid_c_controls
{
  SemaphoreHandle_t mutex; // mutex for changing values in the entire
                           // pid_controller structure
  bool init; // when true, cv is held to init_value and integrator is cleared
  float init_value; // when init is true, cv is held to this value
};

/*
PID controller alarms and status -- Bool values indicating aberration in PID
controller
*/
struct _pid_c_status
{
  bool pv_hi;  // pv is at hi level
  bool pv_lo;  // pv is at lo level
  bool pv_NaN; // pv input is not a number

  bool cv_hi;          // cv is at hi level
  bool cv_lo;          // cv is at lo level
  bool cv_limited;     // cv is being limited
  bool cv_NaN;         // cv is not a number
  bool cv_roc_hi;      // cv rate of change is hi
  bool cv_roc_lo;      // cv rate of change is lo
  bool cv_roc_limited; // cv is rate limited

  bool sp_hi;          // sp input is hi
  bool sp_lo;          // sp input is lo
  bool sp_limited;     // sp is limited
  bool sp_NaN;         // sp is not a number
  bool sp_roc_limited; // sp is rate limited
};

/*
PID controller full structure
*/
struct pid_controller_struct
{
  // Inputs
  float *pv; // Process Value
  float sp;  // Setpoint

  float kp; // Proportional Gain
  float ki; // Integral Gain
  float kd; // Derivative Gain

  bool control_action; // true = positive, false = negative

  struct _pid_c_advanced advanced;
  struct _pid_c_limits limits;
  struct _pid_c_controls control;

  // Outputs
  float *cv; // Control Value
  float err; // Calculated Error

  float p_out; // Proportional component of cv
  float i_out; // Integral component of cv
  float d_out; // derivative component of cv

  struct _pid_c_status status;
};

// create a default struct
struct pid_controller_struct create_pid_control_struct (float *pv, float sp,
                                                        float *cv, float kp,
                                                        float ki, float kd,
                                                        bool control_action);

// calculate derivative component
float calc_d_out (struct pid_controller_struct *pid);

// PID Controller
void pid_controller (struct pid_controller_struct *pid);

// start the PID controller task
void pid_controller_start (

);