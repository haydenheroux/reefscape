#pragma once

#include "Eigen.hh"
#include "Elevator.hh"
#include "au/math.hh"
#include "units.hh"

namespace reefscape {

// TODO(hayden): Scope to `AffineSystemSim::State`
const DisplacementUnit kPositionTolerance = (centi(meters))(1);
const VelocityUnit kVelocityToleracne = (centi(meters) / second)(1);

class AffineSystemSim {
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

    State ClampPosition(DisplacementUnit min, DisplacementUnit max) const {
      DisplacementUnit position = Position();

      if (position > max) {
        return {max, Velocity()};
      } else if (position < min) {
        return {min, Velocity()};
      }

      return *this;
    }

    bool At(State other) const {
      bool position_in_tolerance =
          au::abs(Position() - other.Position()) < kPositionTolerance;
      bool velocity_in_tolerance =
          au::abs(Velocity() - other.Velocity()) < kVelocityToleracne;
      return position_in_tolerance && velocity_in_tolerance;
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
  };

  AffineSystemSim(
      SystemMatrix<State::Dimension> continuous_system,
      InputMatrix<State::Dimension, Input::Dimension> continuous_input,
      Eigen::Vector<double, State::Dimension> continuous_constant,
      TimeUnit time_step);

  AffineSystemSim(const Elevator &elevator, AccelerationUnit gravity,
                  TimeUnit time_step);

  Input StabilizingInput() const;

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
  Eigen::Vector<double, State::Dimension> continuous_constant_;
  Eigen::Vector<double, State::Dimension> discrete_constant_;
  TimeUnit time_step_;
  DisplacementUnit min_position_;
  DisplacementUnit max_position_;
};

}  // namespace reefscape
