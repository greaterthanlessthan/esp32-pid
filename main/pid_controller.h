/**
 * @brief allow Linux gcc to compile this library, allowing testing with
 * Python
 */
#define PY_TESTING false

#include <stdbool.h>
#include <stdint.h>

#if PY_TESTING
#define PY_MUTEX_HANDLE bool mutex
#else
#include "freertos/semphr.h"
#define PY_MUTEX_HANDLE SemaphoreHandle_t mutex
#endif

#define PID_SAMPLE_PERIOD 20

/**
 * @brief Advanced features for PID controller
 *
 *
 * TODO: Deadband
 * TODO: P and D setpoint weight
 * TODO: PV moving average?
 *
 */
typedef struct pid_c_advanced
{
  // PID control loop sample period in ms
  uint16_t sample_period;

  /**
   * @brief Windup control mode
   * windup_control_mode = 0: limits not used
   *
   * windup_control_mode = 1: when err is above limit, integrator cannot
   * integrate in that direction
   *
   * windup_control_mode = 2: integrator behaves as though err was equal to
   * limit when above/below limits
   *
   * windup_control_mode = 3: same as 1, but integrator value up to this point
   * is also cleared
   *
   * disable_pos_windup and disable_pos_windup function in any mode
   */
  uint8_t windup_control_mode;

  // When err is above this value, windup control takes effect
  float windup_pos_limit;
  // When err is below this value, windup control takes effect.
  float windup_neg_limit;
  // When set to true, integrator cannot integrate in the positive direction
  bool disable_pos_windup;
  // When set to true, integrator cannot integrate in the negative direction
  bool disable_neg_windup;

} pid_c_advanced;
/**
 * @brief Default values of PID advanced controls. Override if used.
 * Recommended to use mutex when changing during runtime
 *
 */
extern pid_c_advanced pid_advanced_default;

/**
 * @brief Limits for PID controller
 *
 */
typedef struct pid_c_limits
{
  // when true, cv can't go above or below limits. In either case, status is
  // set
  bool enable_cv_limits;
  // Maximum cv
  float cv_limit_hi;
  // Minimum cv
  float cv_limit_lo;

  // When true, cv becomes rate limited. In either case, status is set
  bool enable_cv_roc_limits;
  // Maximum rate of change in %/s
  float cv_roc_limit_hi;
  // Minimum rate of change in %/s
  float cv_roc_limit_lo;

  // Maximum pv. Sets status only
  float pv_limit_hi;
  // Minimum pv. Sets status only
  float pv_limit_lo;
  // Maximum rate of change in %/s. Sets status only
  float pv_roc_limit_hi;
  // Minimum rate of change in %/s. Sets status only
  float pv_roc_limit_lo;

  // when true, sp can't go above or below limits. In either case, status is
  // set
  bool enable_sp_limits;
  // Maximum sp
  float sp_limit_hi;
  // Minimum sp
  float sp_limit_lo;

  // Rate limits sp. Can help in windup control
  bool enable_sp_roc_limits;
  // Maximum rate of change in %/s
  float sp_roc_limit_hi;
  // Minimum rate of change in %/s
  float sp_roc_limit_lo;
} pid_c_limits;
/**
 * @brief Default values of PID limits. Override if used. Recommended to use
 * mutex when changing during runtime
 *
 */
extern pid_c_limits pid_limits_default;

/**
 * @brief Control structure used for starting and stopping the PID controller
 * A mutex is also provided which can be used if race conditions are
 * experienced
 *
 */
typedef struct pid_c_controls
{
  // mutex for changing values in pid_controller_struct
  // This is a macro to support compiling with gcc for testing
  PY_MUTEX_HANDLE;
  // when true, cv is held to init_value and integrator is cleared
  bool init;
  // when init is true, cv is held to this value
  float init_value;
} pid_c_controls;
/**
 * @brief Default values of PID Control. Function as calling a function to
 * create mutex
 *
 */
pid_c_controls pid_control_default ();

/**
 * @brief PID controller status bools
 *
 */
typedef struct pid_c_status
{
  bool pv_hi;  // pv is at hi level
  bool pv_lo;  // pv is at lo level
  bool pv_NaN; // pv input is not a number or is inf

  bool cv_hi;          // cv is at hi level
  bool cv_lo;          // cv is at lo level
  bool cv_limited;     // cv is being limited
  bool cv_NaN;         // cv is not a number or is inf
  bool cv_roc_hi;      // cv rate of change is hi
  bool cv_roc_lo;      // cv rate of change is lo
  bool cv_roc_limited; // cv is rate limited

  bool sp_hi;          // sp input is hi
  bool sp_lo;          // sp input is lo
  bool sp_limited;     // sp is limited
  bool sp_NaN;         // sp is not a number or is inf
  bool sp_roc_limited; // sp is rate limited
} pid_c_status;

/**
 * @brief Structure used to
 *
 */
typedef struct pid_controller_struct
{
  // Inputs
  float *pv; // Process Value Pointer
  float sp;  // Setpoint

  float kp; // Proportional Gain
  float ki; // Integral Gain
  float kd; // Derivative Gain

  bool control_action; // true = positive, false = negative

  struct pid_c_advanced advanced;
  struct pid_c_limits limits;
  struct pid_c_controls control;

  // Outputs
  float *cv; // Control Value Pointer
  float err; // Calculated Error

  float p_out; // Proportional component of cv
  float i_out; // Integral component of cv
  float d_out; // derivative component of cv

  struct pid_c_status status;
} pid_controller_struct;

/**
 * @brief Creates structure for the PID controller. Basic values are given
 * while defaults are used for limits and advanced functions
 *
 * @param pv The memory address of the process value. Typically an analog
 * input, ADC to I2C, etc. coming from a sensor
 * @param sp The setpoint for the PID loop
 * @param cv The memory address of the control value. Typically an analog
 * inpit, ADC to I2C, etc.
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param control_action when true, error is calculated as pv - sp. When false,
 * it's calculated as sp - pv
 * @return pid_controller_struct used for PID controller task
 *
 */
pid_controller_struct create_pid_control_struct (float *pv, float sp,
                                                 float *cv, float kp, float ki,
                                                 float kd,
                                                 bool control_action);

/**
 * @brief The PID Controller. Use pid_controller_start to begin the PID
 * controller an separate task
 *
 * @param pid The struct containing information for the PID controller
 *
 */
void pid_controller (struct pid_controller_struct *pid);

/**
 * @brief Creates the task running the PID Controller
 *
 * @param pid the PID controller structure
 *
 */
void pid_controller_start (struct pid_controller_struct *pid);

/**
 * @brief Calculate the integral component
 *
 * @param adv Advanced control structure, used for windup
 * @param err The calculated error value
 * @param integral The previous integral value
 *
 */
float calc_i_out (pid_c_advanced *adv, float err, float integral);

/**
 * @brief Takes a float value, clamps it between two values if enabled, and
 * sets status bools
 *
 */
float hi_lo_limits_w_status (float value, float hi, float lo, bool limit_en,
                             bool *hi_status, bool *lo_status,
                             bool *limited_status);

/**
 * @brief Takes a float value, compares it to its old value and clamps it to an
 * acceptable rate of change, and sets status bools
 *
 */
float roc_limits_w_status (float new, float old, uint16_t period_ms,
                           float hi_roc, float lo_roc, bool limit_roc_en,
                           bool *hi_roc_status, bool *lo_roc_status,
                           bool *roc_limited_status);