/**
 * @brief allow Linux gcc to compile this library, allowing testing with
 * Python
 */
#define PY_TESTING false

#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#define PID_SAMPLE_PERIOD 20

enum integral_windup_mode
{
  NORMAL,
  CLAMPED,
  HELD,
  CLEARED,
  _CLAMPED_POS,
  _CLAMPED_NEG,
};

/**
 * @brief Advanced features for PID controller
 *
 *
 * TODO: Deadband
 * TODO: P and D setpoint weight
 * TODO: PV moving average?
 *
 */
struct pid_c_advanced
{
  // PID control loop sample period in ms
  uint16_t period;

  /**
   * @brief Windup control mode
   *
   * NORMAL: limits not used
   * CLAMPED: when err is above limit, behaves as though err was equal to limit
   * HELD: when err is above limit, cannot integrate in that direction
   * CLEARED: when err is above limit, integral is set to 0.0
   *
   * dis_pos_wndup and dis_neg_wndup function in any mode
   */
  enum integral_windup_mode wndup_mode;

  // When err is above this value, windup control takes effect
  float wndup_pos_lim;
  // When err is below this value, windup control takes effect.
  float wndup_neg_lim;
  // When set to true, integrator cannot integrate in the positive direction
  bool dis_pos_wndup;
  // When set to true, integrator cannot integrate in the negative direction
  bool dis_neg_wndup;
};

/**
 * @brief Limits for PID controller
 *
 */
struct pid_limit
{
  // when true, can't go above or below limits. In either case, status is set
  bool en_lim;
  // Maximum
  float lim_hi;
  // Minimum
  float lim_lo;

  // When true, becomes rate limited. In either case, status is set
  bool en_roc_lim;
  // Maximum rate of change in %/s
  float roc_lim_hi;
  // Minimum rate of change in %/s
  float roc_lim_lo;
};

struct pid_c_limits
{
  struct pid_limit cv;
  struct pid_limit pv;
  struct pid_limit sp;
};

/**
 * @brief Control structure used for starting and stopping the PID controller
 * A mutex is also provided which can be used if race conditions are
 * experienced
 *
 */
struct pid_c_controls
{
  // mutex for changing values in pid_controller_struct
  // This is a macro to support compiling with gcc for testing
  SemaphoreHandle_t mutex;
  // when true, cv is held to init_value and integrator is cleared
  bool init;
  // when init is true, cv is held to this value
  float init_value;
};

/**
 * @brief PID controller status bools
 *
 */
struct pid_status
{
  bool hi;      // at hi level
  bool lo;      // at lo level
  bool limited; // limited

  bool roc_hi;       // rate of change is hi
  bool roc_lo;       // rate of change is lo
  bool rate_limited; // rate limited

  bool NaN; // not a number/infinity
};

struct pid_c_status
{
  struct pid_status cv;
  struct pid_status pv;
  struct pid_status sp;
};

/**
 * @brief Structure used to
 *
 */
typedef struct pid_controller_struct
{
  // Inputs
  xQueueHandle pv; // Process Value Queue
  float sp;        // Setpoint

  float kp; // Proportional Gain
  float ki; // Integral Gain
  float kd; // Derivative Gain

  bool control_action; // true = positive, false = negative

  struct pid_c_advanced adv;
  struct pid_c_limits lim;
  struct pid_c_controls control;

  // Outputs
  xQueueHandle cv; // Control Value Queue
  float err;       // Calculated Error

  float p_out; // Proportional component of cv
  float i_out; // Integral component of cv
  float d_out; // derivative component of cv

  struct pid_c_status sts;
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
pid_controller_struct create_pid_control_struct (float sp, float kp, float ki,
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
