#ifndef PID_CONTROLLER_PID_CONTROLLER
#define PID_CONTROLLER_PID_CONTROLLER

#include <stdint.h>
#include <string>

namespace pid_controller
{

enum class PIDControllerStatus : uint8_t
{
  Error,
  Idle,
  Running,
  ShuttingDown
};

class PIDController
{
public:
  PIDController();

  PIDControllerStatus controllerStatus() const;
  double currentError() const;
  double proportionalControlSignal() const;
  double integralControlSignal() const;
  double derivativeControlSignal() const;
  double computeNextControlSignal();

  double current_value;
  double cutoff_value;
  double setpoint;
  double proportional_control_gain;
  double integral_control_gain;
  double derivative_control_gain;
  double time_step;

private:
  void computeProportionalControlSignal();
  void computeIntegralControlSignal();
  void computeDerivativeControlSignal();
  void updateInternalVariables();
  void resetInternalVariables();

  PIDControllerStatus controller_status_;
  double accumulated_error_;
  double previous_error_;
  double current_error_;
  double proportional_control_signal_;
  double integral_control_signal_;
  double derivative_control_signal_;
};

}

#endif
