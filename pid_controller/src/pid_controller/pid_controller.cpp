#include "pid_controller/pid_controller.h"

#include <cmath>

pid_controller::PIDController::PIDController() :
    current_value {0.0},
    cutoff_value {0.0},
    setpoint {0.0},
    proportional_control_gain {0.0},
    integral_control_gain {0.0},
    derivative_control_gain {0.0},
    time_step {0.0},
    controller_status_ {PIDControllerStatus::Idle},
    accumulated_error_ {0.0},
    previous_error_ {0.0},
    current_error_ {0.0},
    proportional_control_signal_ {0.0},
    integral_control_signal_ {0.0},
    derivative_control_signal_ {0.0}
{
}

pid_controller::PIDControllerStatus
pid_controller::PIDController::controllerStatus() const
{
  return controller_status_;
}

double
pid_controller::PIDController::currentError() const
{
  return current_error_;
}

double
pid_controller::PIDController::proportionalControlSignal() const
{
  return proportional_control_signal_;
}

double
pid_controller::PIDController::integralControlSignal() const
{
  return integral_control_signal_;
}

double
pid_controller::PIDController::derivativeControlSignal() const
{
  return derivative_control_signal_;
}

double
pid_controller::PIDController::computeNextControlSignal()
{
  current_error_ = current_value - setpoint;

  if (std::abs(current_error_) <= cutoff_value)
  {
    controller_status_ = (controller_status_ == PIDControllerStatus::Running)
                        ? controller_status_ : PIDControllerStatus::Running;
    computeProportionalControlSignal();
    computeIntegralControlSignal();
    computeDerivativeControlSignal();
    updateInternalVariables();
  }
  else
  {
    controller_status_ = (controller_status_ == PIDControllerStatus::Idle)
                        ? controller_status_ : PIDControllerStatus::Idle;
    resetInternalVariables();
  }
  return proportional_control_signal_ + integral_control_signal_ + derivative_control_signal_;
}

void
pid_controller::PIDController::computeProportionalControlSignal()
{
  proportional_control_signal_ = proportional_control_gain * (current_error_);
}

void
pid_controller::PIDController::computeIntegralControlSignal()
{
  constexpr double one_half {0.5};
  accumulated_error_ += time_step * one_half * (previous_error_ + current_error_);
  integral_control_signal_ = integral_control_gain * accumulated_error_;
}

void
pid_controller::PIDController::computeDerivativeControlSignal()
{
}

void
pid_controller::PIDController::updateInternalVariables()
{
  previous_error_ = current_error_;
}

void
pid_controller::PIDController::resetInternalVariables()
{
  constexpr double zero {0.0};
  accumulated_error_ = zero;
  previous_error_ = zero;
  current_error_ = zero;
  proportional_control_signal_ = zero;
  integral_control_signal_ = zero;
  derivative_control_signal_ = zero;
}
