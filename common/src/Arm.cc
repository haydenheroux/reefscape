#include "Arm.hh"

#include "units.hh"

namespace reefscape {

AngularVelocityCoefficient Arm::VelocityCoefficient() const {
  return -1 *
         (gear_ratio * gear_ratio * motor.torque_constant_ * au::radians(1)) /
         (motor.angular_velocity_constant_ * motor.resistance_ *
          moment_of_inertia);
}

AngularVoltageCoefficient Arm::VoltageCoefficient() const {
  return (gear_ratio * motor.torque_constant_ * au::radians(1)) /
         (motor.resistance_ * moment_of_inertia);
}

// TODO(hayden): Is this backwards? If so, this needs to be changed in other
// places too
AngularVelocity Arm::MotorVelocity(AngularVelocity velocity) const {
  return velocity * gear_ratio;
}

AngularAcceleration Arm::Acceleration(AngularVelocity velocity,
                                      Voltage voltage) const {
  return Torque(velocity, voltage) * au::radians(1) / moment_of_inertia;
}

quantities::Torque Arm::Torque(AngularVelocity velocity,
                               Voltage voltage) const {
  auto voltage_torque =
      (gear_ratio * motor.torque_constant_ * voltage) / motor.resistance_;

  auto back_emf_torque =
      -1 * (gear_ratio * gear_ratio * motor.torque_constant_ * velocity) /
      (motor.resistance_ * motor.angular_velocity_constant_);

  return voltage_torque + back_emf_torque;
}

}  // namespace reefscape
