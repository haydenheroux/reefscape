#include "Arm.hh"

#include "units.hh"

namespace reefscape {

AngularVelocityCoefficientUnit Arm::VelocityCoefficient() const {
  return -1 * (gear_ratio * gear_ratio * motor.torque_constant_ * radians(1)) /
         (motor.angular_velocity_constant_ * motor.resistance_ *
          moment_of_inertia);
}

AngularVoltageCoefficientUnit Arm::VoltageCoefficient() const {
  return (gear_ratio * motor.torque_constant_ * radians(1)) /
         (motor.resistance_ * moment_of_inertia);
}

// TODO(hayden): Is this backwards? If so, this needs to be changed in other
// places too
AngularVelocityUnit Arm::MotorVelocity(AngularVelocityUnit velocity) const {
  return velocity * gear_ratio;
}

AngularAccelerationUnit Arm::Acceleration(AngularVelocityUnit velocity,
                                          VoltageUnit voltage) const {
  return Torque(velocity, voltage) * radians(1) / moment_of_inertia;
}

TorqueUnit Arm::Torque(AngularVelocityUnit velocity,
                       VoltageUnit voltage) const {
  TorqueUnit voltage_torque =
      (gear_ratio * motor.torque_constant_ * voltage) / motor.resistance_;

  TorqueUnit back_emf_torque =
      -1 * (gear_ratio * gear_ratio * motor.torque_constant_ * velocity) /
      (motor.resistance_ * motor.angular_velocity_constant_);

  return voltage_torque + back_emf_torque;
}

}  // namespace reefscape
