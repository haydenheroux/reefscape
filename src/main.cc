#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "units.hh"

#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

void discretize(const Eigen::Matrix<double, 2, 2> &Ac,
                const Eigen::Matrix<double, 2, 1> &Bc, TimeUnit dt,
                Eigen::Matrix<double, 2, 2> &Ad,
                Eigen::Matrix<double, 2, 1> &Bd) {
  Eigen::Matrix<double, 3, 3> M;
  M.template block<2, 2>(0, 0) = Ac;
  M.template block<2, 1>(0, 2) = Bc;
  M.template block<1, 3>(2, 0).setZero();

  Eigen::Matrix<double, 3, 3> phi = (M * dt.in(seconds)).exp();

  Ad = phi.template block<2, 2>(0, 0);
  Bd = phi.template block<2, 1>(0, 2);
}

struct State {
  Eigen::Vector<double, 2> state_;

  constexpr State() { state_ << 0, 0; };

  constexpr DisplacementUnit Position() const { return meters(state_[0]); }
  constexpr VelocityUnit Velocity() const { return mps(state_[1]); }
  constexpr void SetPosition(DisplacementUnit position) {
    state_[0] = position.in(meters);
  }
  constexpr void SetVelocity(VelocityUnit velocity) {
    state_[1] = velocity.in(meters / second);
  }
};

struct Input {
  Eigen::Vector<double, 1> input_;

  constexpr Input() { input_ << 0; }

  constexpr VoltageUnit Voltage() const { return volts(input_[0]); }
  constexpr void SetVoltage(VoltageUnit voltage) {
    input_[0] = voltage.in(volts);
  }
};

int main() {
  Motor krakenX60{volts(12), newton_meters(7.09), amperes(366), rpm(6000),
                  amperes(2)};

  ElevatorConstants constants{gear_ratio(5), 0.5 * inches(1.273),
                              pounds_mass(30), amperes(120)};

  Elevator elevator{constants, krakenX60 * 2};

  Eigen::Matrix<double, 2, 2> Ac;
  Ac << 0, 1, 0, elevator.velocity_coefficient().in(velocity_coefficient);

  Eigen::Matrix<double, 2, 1> Bc;
  Bc << 0, elevator.voltage_coefficient().in(voltage_coefficient);

  TimeUnit dt = (milli(seconds))(5);

  Eigen::Matrix<double, 2, 2> Ad;
  Eigen::Matrix<double, 2, 1> Bd;
  discretize(Ac, Bc, dt, Ad, Bd);

  State state;
  state.SetPosition(meters(0));
  state.SetVelocity(mps(0));

  Input input;
  input.SetVoltage(volts(12));

  VoltageUnit desired_voltage = volts(12);

  bool limit_voltage = true;

  while (true) {
    std::cout << "state => " << state.state_ << "\n\n";
    VoltageUnit voltage =
        elevator.limited_voltage(state.Velocity(), desired_voltage);
    input.SetVoltage(voltage);
    Eigen::Vector<double, 2> state_dot = Ac * state.state_ + Bc * input.input_;
    std::cout << "acceleration => " << state_dot[1] << std::endl;
    Eigen::Vector<double, 2> next_state = Ad * state.state_ + Bd * input.input_;
    state.SetPosition(meters(next_state[0]));
    state.SetVelocity(mps(next_state[1]));
  }
}
