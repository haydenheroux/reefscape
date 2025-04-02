#pragma once

#include "units.hh"

struct Motor {
  VoltageUnit nominal_voltage_;
  TorqueUnit stall_torque_;
  CurrentUnit stall_current_;
  AngularVelocityUnit free_speed_;
  CurrentUnit free_current_;
  TorqueConstantUnit torque_constant_;
  ResistanceUnit resistance_;
  AngularVelocityConstantUnit angular_velocity_constant_;

  Motor(VoltageUnit nominal_voltage, TorqueUnit stall_torque,
        CurrentUnit stall_current, AngularVelocityUnit free_speed,
        CurrentUnit free_current)
      : nominal_voltage_(nominal_voltage), stall_torque_(stall_torque),
        stall_current_(stall_current), free_speed_(free_speed),
        free_current_(free_current),
        torque_constant_(stall_torque_ / stall_current_),
        resistance_(nominal_voltage_ / stall_current_),
        angular_velocity_constant_(
            free_speed_ / (nominal_voltage_ - free_current_ * resistance_)) {};

  Motor operator*(unsigned int num_motors) const {
    return Motor(nominal_voltage_, stall_torque_ * num_motors,
                 stall_current_ * num_motors, free_speed_,
                 free_current_ * num_motors);
  }
};
