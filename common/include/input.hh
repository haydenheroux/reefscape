#pragma once

#include "Eigen.hh"

namespace reefscape {

struct VoltageInput {
  static const int Dimension = 1;
  InputVector<Dimension> vector;

  VoltageInput(VoltageUnit voltage) { SetVoltage(voltage); }
  VoltageInput(const InputVector<Dimension> &input) {
    this->vector[0] = input[0];
  }
  VoltageInput &operator=(const InputVector<Dimension> &input) {
    this->vector[0] = input[0];
    return *this;
  }

  VoltageUnit Voltage() const { return volts(vector[0]); }
  void SetVoltage(VoltageUnit voltage) { vector[0] = voltage.in(volts); }
};

};  // namespace reefscape
