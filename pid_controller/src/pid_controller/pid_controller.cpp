#include "pid_controller/pid_controller.h"

#include <cmath>

pid_controller::PIDController::PIDController() :
    current_value {0.0},
    cutoff_value {0.0},
    setpoint {0.0},
    proportional_control_gain {0.0},
    integral_control_gain {0.0},
    derivative_control_gain {0.0},
    controller_status_ {PIDControllerStatus::Idle},
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
  }
  else
  {
    controller_status_ = (controller_status_ == PIDControllerStatus::Idle)
                        ? controller_status_ : PIDControllerStatus::Idle;
    setControlSignalsToZero();
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
}

void
pid_controller::PIDController::computeDerivativeControlSignal()
{
}

void
pid_controller::PIDController::setControlSignalsToZero()
{
  constexpr double zero_control_signal {0.0};
  proportional_control_signal_ = zero_control_signal;
  integral_control_signal_ = zero_control_signal;
  derivative_control_signal_ - zero_control_signal;
}
