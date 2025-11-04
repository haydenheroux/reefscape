#include "Elevator.hh"

#include "input.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

LinearVelocityCoefficient Elevator::VelocityCoefficient() const {
  return -1 *
         (gear_ratio * gear_ratio * motor.torque_constant_ * au::radians(1)) /
         (motor.resistance_ * drum_radius * drum_radius * mass *
          motor.angular_velocity_constant_);
}

LinearVoltageCoefficient Elevator::VoltageCoefficient() const {
  return (gear_ratio * motor.torque_constant_) /
         (motor.resistance_ * mass * drum_radius);
}

AngularVelocity Elevator::MotorVelocity(LinearVelocity velocity) const {
  return velocity * au::radians(1) * gear_ratio / drum_radius;
}

LinearAcceleration Elevator::Acceleration(LinearVelocity velocity,
                                          Voltage voltage) const {
  return Force(velocity, voltage) / mass;
}

quantities::Force Elevator::Force(LinearVelocity velocity,
                                  Voltage voltage) const {
  auto voltage_force = (gear_ratio * motor.torque_constant_ * voltage) /
                       (motor.resistance_ * drum_radius);

  auto back_emf_force = -1 *
                        (gear_ratio * gear_ratio * motor.torque_constant_ *
                         velocity * au::radians(1)) /
                        (motor.resistance_ * drum_radius * drum_radius *
                         motor.angular_velocity_constant_);

  return voltage_force + back_emf_force;
}

template <>
SystemMatrix<PositionVelocityState::Dimension>
Elevator::ContinuousSystemMatrix<PositionVelocityState>() const {
  SystemMatrix<PositionVelocityState::Dimension> result;
  result << 0, 1, 0,
      VelocityCoefficient().in((au::meters / squared(au::second)) /
                               (au::meters / au::second));
  return result;
}

template <>
InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>
Elevator::ContinuousInputMatrix<PositionVelocityState, VoltageInput>() const {
  InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension> result;
  result << 0,
      VoltageCoefficient().in((au::meters / squared(au::second)) / au::volt);
  return result;
}

};  // namespace reefscape
