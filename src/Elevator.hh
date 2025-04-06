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
      : gear_ratio(gear_ratio), drum_radius(drum_radius), mass(mass),
        max_current(max_current), max_travel(max_travel), motor(motor) {};

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
  ElevatorSim(const Elevator &elevator, AccelerationUnit gravity,
              TimeUnit time_step);

  DisplacementUnit Position() const { return state_.Position(); }
  void SetPosition(DisplacementUnit position) { state_.SetPosition(position); }

  VelocityUnit Velocity() const { return state_.Velocity(); }

  VoltageUnit Voltage() const { return input_.Voltage(); }

  VoltageUnit OpposingGravity() const;
  VoltageUnit Saturate(VoltageUnit voltage) const {
    return au::clamp(voltage, -elevator_.motor.nominal_voltage_,
                     elevator_.motor.nominal_voltage_);
  }
  VoltageUnit CurrentLimit(VoltageUnit voltage) const {
    return elevator_.LimitVoltage(Velocity(), Saturate(voltage));
  }

  void Update(VoltageUnit voltage);

  // TODO
  const static int kNumStates = 2;
  const static int kNumInputs = 1;

private:
  struct ElevatorState {
    Eigen::Vector<double, kNumStates> state;

    ElevatorState(DisplacementUnit position, VelocityUnit velocity) {
      SetPosition(position);
      SetVelocity(velocity);
    }
    ElevatorState &operator=(const Eigen::Vector<double, kNumStates> &state) {
      this->state[0] = state[0];
      this->state[1] = state[1];
      return *this;
    }

    DisplacementUnit Position() const { return meters(state[0]); }
    VelocityUnit Velocity() const { return (meters / second)(state[1]); }
    void SetPosition(DisplacementUnit position) {
      state[0] = position.in(meters);
    }
    void SetVelocity(VelocityUnit velocity) {
      state[1] = velocity.in(meters / second);
    }
  };

  struct ElevatorInput {
    Eigen::Vector<double, kNumInputs> input;

    ElevatorInput(VoltageUnit voltage) { SetVoltage(voltage); }
    ElevatorInput(const Eigen::Vector<double, kNumInputs> &input) {
      this->input[0] = input[0];
    }
    ElevatorInput &operator=(const Eigen::Vector<double, kNumInputs> &input) {
      this->input[0] = input[0];
      return *this;
    }

    VoltageUnit Voltage() const { return volts(input[0]); }
    void SetVoltage(VoltageUnit voltage) { input[0] = voltage.in(volts); }
  };

  SystemMatrix<kNumStates> continuous_system_;
  InputMatrix<kNumStates, kNumInputs> continuous_input_;
  InputMatrix<kNumInputs, kNumStates> continuous_input_pseudoinverse_;
  SystemMatrix<kNumStates> discrete_system_;
  InputMatrix<kNumStates, kNumInputs> discrete_input_;
  Eigen::Vector<double, kNumStates> continuous_gravity_;
  Eigen::Vector<double, kNumStates> discrete_gravity_;
  TimeUnit time_step_;
  ElevatorState state_;
  ElevatorInput input_;
  Elevator elevator_;
};
} // namespace sim
