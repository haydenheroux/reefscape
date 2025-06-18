#include "Elevator.hh"

#include "au/math.hh"
#include "input.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

VelocityCoefficientUnit Elevator::VelocityCoefficient() const {
  return -1 * (gear_ratio * gear_ratio * motor.torque_constant_ * radians(1)) /
         (motor.resistance_ * drum_radius * drum_radius * mass *
          motor.angular_velocity_constant_);
}

VoltageCoefficientUnit Elevator::VoltageCoefficient() const {
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

VelocityUnit Elevator::MaximumVelocity() const {
  // Maximize ω with dω/dt = (velocity_coefficient)·ω +
  // (voltage_coefficient)·(motor.nominal_voltage)
  return -1 * motor.nominal_voltage_ * VoltageCoefficient() /
         VelocityCoefficient();
}

AccelerationUnit Elevator::MaximumAcceleration() const {
  VelocityUnit velocity = (meters / second)(0);
  VoltageUnit voltage = LimitVoltage(velocity, volts(12));
  return Acceleration(velocity, voltage);
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

CurrentUnit Elevator::Current(VelocityUnit velocity,
                              VoltageUnit voltage) const {
  return voltage / motor.resistance_ -
         MotorVelocity(velocity) /
             (motor.angular_velocity_constant_ * motor.resistance_);
}

VoltageUnit Elevator::LimitVoltage(VelocityUnit velocity,
                                   VoltageUnit voltage) const {
  voltage = au::clamp(voltage, -motor.nominal_voltage_, motor.nominal_voltage_);

  CurrentUnit current = Current(velocity, voltage);
  if (current > max_current) {
    voltage = max_current * motor.resistance_ +
              MotorVelocity(velocity) / motor.angular_velocity_constant_;
  }
  return voltage;
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
