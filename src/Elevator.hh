#pragma once

#include "Eigen.hh"
#include "Motor.hh"
#include "units.hh"

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

struct ElevatorState {
  Eigen::Vector<double, 2> state;

  ElevatorState(DisplacementUnit position, VelocityUnit velocity) {
    SetPosition(position);
    SetVelocity(velocity);
  }
  ElevatorState &operator=(const Eigen::Vector<double, 2> &state) {
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
  Eigen::Vector<double, 2> input;

  ElevatorInput(VoltageUnit voltage, AccelerationUnit acceleration) {
    SetVoltage(voltage);
    SetAcceleration(acceleration);
  }

  VoltageUnit Voltage() const { return volts(input[0]); }
  AccelerationUnit Acceleration() {
    return (meters / squared(second))(input[1]);
  }
  void SetVoltage(VoltageUnit voltage) { input[0] = voltage.in(volts); }
  void SetAcceleration(AccelerationUnit acceleration) {
    input[1] = acceleration.in(meters / squared(second));
  }
};

class ElevatorSim {
public:
  ElevatorSim(const Elevator &elevator, TimeUnit time_step);

  DisplacementUnit Position() const { return state_.Position(); }

  VelocityUnit Velocity() const { return state_.Velocity(); }

  VoltageUnit Voltage() const { return input_.Voltage(); }

  void SetGravity(AccelerationUnit gravity) { input_.SetAcceleration(gravity); }

  void Update(VoltageUnit voltage);

  // TODO
  const static int kNumStates = 2;
  const static int kNumInputs = 2;

private:
  SystemMatrix<kNumStates> continuous_system_;
  InputMatrix<kNumStates, kNumInputs> continuous_input_;
  SystemMatrix<kNumStates> discrete_system_;
  InputMatrix<kNumStates, kNumInputs> discrete_input_;
  TimeUnit time_step_;
  ElevatorState state_;
  ElevatorInput input_;
  Elevator elevator_;
};
