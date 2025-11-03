#include "Elevator.hh"

#include "input.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

LinearVelocityCoefficientUnit Elevator::VelocityCoefficient() const {
  return -1 * (gear_ratio * gear_ratio * motor.torque_constant_ * radians(1)) /
         (motor.resistance_ * drum_radius * drum_radius * mass *
          motor.angular_velocity_constant_);
}

LinearVoltageCoefficientUnit Elevator::VoltageCoefficient() const {
  return (gear_ratio * motor.torque_constant_) /
         (motor.resistance_ * mass * drum_radius);
}

AngularVelocityUnit Elevator::MotorVelocity(VelocityUnit velocity) const {
  return velocity * radians(1) * gear_ratio / drum_radius;
}

AccelerationUnit Elevator::Acceleration(VelocityUnit velocity,
                                        VoltageUnit voltage) const {
  return Force(velocity, voltage) / mass;
}

ForceUnit Elevator::Force(VelocityUnit velocity, VoltageUnit voltage) const {
  ForceUnit voltage_force = (gear_ratio * motor.torque_constant_ * voltage) /
                            (motor.resistance_ * drum_radius);

  ForceUnit back_emf_force = -1 *
                             (gear_ratio * gear_ratio * motor.torque_constant_ *
                              velocity * radians(1)) /
                             (motor.resistance_ * drum_radius * drum_radius *
                              motor.angular_velocity_constant_);

  return voltage_force + back_emf_force;
}

template <>
SystemMatrix<PositionVelocityState::Dimension>
Elevator::ContinuousSystemMatrix<PositionVelocityState>() const {
  SystemMatrix<PositionVelocityState::Dimension> result;
  result << 0, 1, 0,
      VelocityCoefficient().in((meters / squared(second)) / (meters / second));
  return result;
}

template <>
InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>
Elevator::ContinuousInputMatrix<PositionVelocityState, VoltageInput>() const {
  InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension> result;
  result << 0, VoltageCoefficient().in((meters / squared(second)) / volt);
  return result;
}

};  // namespace reefscape
