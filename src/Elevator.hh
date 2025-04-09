#pragma once

#include "Eigen.hh"
#include "Motor.hh"
#include "au/math.hh"
#include "units.hh"

namespace sim {
using namespace units;
struct Elevator {
  RatioUnit gear_ratio;
  DisplacementUnit drum_radius;
  MassUnit mass;
  CurrentUnit max_current;
  DisplacementUnit max_travel;
  Motor motor;

  Elevator(RatioUnit gear_ratio, DisplacementUnit drum_radius, MassUnit mass,
           CurrentUnit max_current, DisplacementUnit max_travel, Motor motor)
      : gear_ratio(gear_ratio),
        drum_radius(drum_radius),
        mass(mass),
        max_current(max_current),
        max_travel(max_travel),
        motor(motor) {};

  AngularVelocityUnit MotorVelocity(VelocityUnit velocity) const;

  AccelerationUnit Acceleration(VelocityUnit velocity,
                                VoltageUnit voltage) const;

  AccelerationUnit MaximumAcceleration() const;

  VelocityCoefficientUnit VelocityCoefficient() const;

  VoltageCoefficientUnit VoltageCoefficient() const;

  VoltageUnit OpposingVoltage(AccelerationUnit acceleration) const;

  ForceUnit Force(VelocityUnit velocity, VoltageUnit voltage) const;

  CurrentUnit Current(VelocityUnit velocity, VoltageUnit voltage) const;

  VoltageUnit LimitVoltage(VelocityUnit velocity, VoltageUnit voltage) const;
};

class ElevatorSim {
 public:
  struct State {
    static const int Dimension = 2;
    Eigen::Vector<double, Dimension> vector;

    State(DisplacementUnit position, VelocityUnit velocity) {
      SetPosition(position);
      SetVelocity(velocity);
    }
    State(const StateVector<Dimension> &state) {
      SetPosition(meters(state[0]));
      SetVelocity((meters / second)(state[1]));
    }
    State &operator=(const StateVector<Dimension> &state) {
      this->vector[0] = state[0];
      this->vector[1] = state[1];
      return *this;
    }
    State operator-(const State &state) { return State{vector - state.vector}; }

    DisplacementUnit Position() const { return meters(vector[0]); }
    VelocityUnit Velocity() const { return (meters / second)(vector[1]); }
    void SetPosition(DisplacementUnit position) {
      vector[0] = position.in(meters);
    }
    void SetVelocity(VelocityUnit velocity) {
      vector[1] = velocity.in(meters / second);
    }

    State Clamped(DisplacementUnit min, DisplacementUnit max) const {
      DisplacementUnit position = Position();

      if (position > max) {
        return {max, Velocity()};
      } else if (position < min) {
        return {min, Velocity()};
      }

      return *this;
    }
  };

  struct Input {
    static const int Dimension = 1;
    InputVector<Dimension> vector;

    Input(VoltageUnit voltage) { SetVoltage(voltage); }
    Input(const InputVector<Dimension> &input) { this->vector[0] = input[0]; }
    Input &operator=(const InputVector<Dimension> &input) {
      this->vector[0] = input[0];
      return *this;
    }
    Input &operator+=(const Input &input) {
      vector += input.vector;
      return *this;
    }

    VoltageUnit Voltage() const { return volts(vector[0]); }
    void SetVoltage(VoltageUnit voltage) { vector[0] = voltage.in(volts); }
    Input Saturated(VoltageUnit nominal_voltage) const {
      return {au::clamp(Voltage(), -nominal_voltage, nominal_voltage)};
    }
  };

  ElevatorSim(const Elevator &elevator, AccelerationUnit gravity,
              TimeUnit time_step);

  // NOTE(hayden): This should be called on every input to maintain accuracy
  Input CurrentLimited(const Input &input) const {
    return {elevator_.LimitVoltage(
        state.Velocity(),
        input.Saturated(elevator_.motor.nominal_voltage_).Voltage())};
  }
  Input OpposingGravity() const;

  void Update();

  State state;
  Input input;

 private:
  // TODO(hayden): Reduce clutter by removing ::Dimension
  SystemMatrix<State::Dimension> continuous_system_;
  InputMatrix<State::Dimension, Input::Dimension> continuous_input_;
  InputLeftPseudoInverseMatrix<State::Dimension, Input::Dimension>
      continuous_input_pseudoinverse_;
  SystemMatrix<State::Dimension> discrete_system_;
  InputMatrix<State::Dimension, Input::Dimension> discrete_input_;
  // TODO(hayden): Introduce a scheme / concept? for vectors with the same
  // number of states / can be converted to states but are not representable by
  // an ElevatorState (these are state derivatives, not actual states)
  Eigen::Vector<double, State::Dimension> continuous_gravity_;
  Eigen::Vector<double, State::Dimension> discrete_gravity_;
  TimeUnit time_step_;
  Elevator elevator_;
};
}  // namespace sim
