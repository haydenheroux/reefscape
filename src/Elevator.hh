#pragma once

#include "Motor.hh"
#include "au/fwd.hh"
#include "au/units/radians.hh"
#include "units.hh"

struct ElevatorConstants {
  RatioUnit gear_ratio_;
  DisplacementUnit drum_radius_;
  MassUnit mass_;
  CurrentUnit max_current_;

  constexpr ElevatorConstants(RatioUnit gear_ratio,
                              DisplacementUnit drum_radius, MassUnit mass,
                              CurrentUnit max_current)
      : gear_ratio_(gear_ratio), drum_radius_(drum_radius), mass_(mass),
        max_current_(max_current) {};
};

class Elevator {
public:
  constexpr Elevator(ElevatorConstants elevator, Motor motor)
      : constants_(elevator), motor_(motor) {};

  constexpr AngularVelocityUnit motor_velocity(VelocityUnit velocity) const {
    return velocity * radians(1) * constants_.gear_ratio_ /
           constants_.drum_radius_;
  }

  constexpr AccelerationUnit acceleration(VelocityUnit velocity,
                                          VoltageUnit voltage) const {
    return force(velocity, voltage) / constants_.mass_;
  }

  constexpr AccelerationUnit maximum_acceleration() const {
    VelocityUnit velocity = mps(0);
    VoltageUnit voltage = this->limited_voltage(velocity, volts(12));
    return acceleration(velocity, voltage);
  }

  constexpr VelocityCoefficient velocity_coefficient() const {
    return -1 *
           (constants_.gear_ratio_ * constants_.gear_ratio_ *
            motor_.torque_constant_ * radians(1)) /
           (motor_.resistance_ * constants_.drum_radius_ *
            constants_.drum_radius_ * constants_.mass_ *
            motor_.angular_velocity_constant_);
  }

  constexpr VoltageCoefficient voltage_coefficient() const {
    return (constants_.gear_ratio_ * motor_.torque_constant_) /
           (motor_.resistance_ * constants_.mass_ * constants_.drum_radius_);
  }

  constexpr ForceUnit force(VelocityUnit velocity, VoltageUnit voltage) const {
    ForceUnit voltage_force =
        (constants_.gear_ratio_ * motor_.torque_constant_ * voltage) /
        (motor_.resistance_ * constants_.drum_radius_);

    ForceUnit back_emf_force =
        -1 *
        (constants_.gear_ratio_ * constants_.gear_ratio_ *
         motor_.torque_constant_ * velocity * radians(1)) /
        (motor_.resistance_ * constants_.drum_radius_ *
         constants_.drum_radius_ * motor_.angular_velocity_constant_);

    return voltage_force + back_emf_force;
  }

  constexpr CurrentUnit current(VelocityUnit velocity,
                                VoltageUnit voltage) const {
    return voltage / motor_.resistance_ -
           motor_velocity(velocity) /
               (motor_.angular_velocity_constant_ * motor_.resistance_);
  }

  constexpr VoltageUnit limited_voltage(VelocityUnit velocity,
                                        VoltageUnit voltage) const {
    CurrentUnit current = this->current(velocity, voltage);
    if (current > constants_.max_current_) {
      voltage =
          constants_.max_current_ * motor_.resistance_ +
          this->motor_velocity(velocity) / motor_.angular_velocity_constant_;
    }
    return voltage;
  }

private:
  ElevatorConstants constants_;
  Motor motor_;
};
