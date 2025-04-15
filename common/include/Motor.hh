#pragma once

#include "au/units/minutes.hh"
#include "units.hh"

namespace reefscape {

struct Motor {
  VoltageUnit nominal_voltage_;
  TorqueUnit stall_torque_;
  CurrentUnit stall_current_;
  AngularVelocityUnit free_speed_;
  CurrentUnit free_current_;
  TorqueConstantUnit torque_constant_;
  ResistanceUnit resistance_;
  AngularVelocityConstantUnit angular_velocity_constant_;

  constexpr Motor(VoltageUnit nominal_voltage, TorqueUnit stall_torque,
                  CurrentUnit stall_current, AngularVelocityUnit free_speed,
                  CurrentUnit free_current)
      : nominal_voltage_(nominal_voltage),
        stall_torque_(stall_torque),
        stall_current_(stall_current),
        free_speed_(free_speed),
        free_current_(free_current),
        torque_constant_(stall_torque_ / stall_current_),
        resistance_(nominal_voltage_ / stall_current_),
        angular_velocity_constant_(
            free_speed_ / (nominal_voltage_ - free_current_ * resistance_)) {};

  constexpr Motor operator*(unsigned int num_motors) const {
    return Motor(nominal_voltage_, stall_torque_ * num_motors,
                 stall_current_ * num_motors, free_speed_,
                 free_current_ * num_motors);
  }

  static constexpr Motor KrakenX60() {
    return {volts(12), newton_meters(7.09), amperes(366),
            (revolutions / minute)(6000), amperes(2)};
  }

  static constexpr Motor KrakenX60FOC() {
    return {volts(12), newton_meters(9.37), amperes(483),
            (revolutions / minute)(5800), amperes(2)};
  }
};

}  // namespace reefscape
