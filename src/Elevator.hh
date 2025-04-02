#pragma once

#include "Eigen.hh"
#include "Motor.hh"
#include "units.hh"

struct ElevatorConstants {
  RatioUnit gear_ratio_;
  DisplacementUnit drum_radius_;
  MassUnit mass_;
  CurrentUnit max_current_;
  DisplacementUnit max_travel_;

  ElevatorConstants(RatioUnit gear_ratio, DisplacementUnit drum_radius,
                    MassUnit mass, CurrentUnit max_current,
                    DisplacementUnit max_travel)
      : gear_ratio_(gear_ratio), drum_radius_(drum_radius), mass_(mass),
        max_current_(max_current), max_travel_(max_travel) {};
};

struct Elevator {
  ElevatorConstants constants_;
  Motor motor_;

  Elevator(ElevatorConstants elevator, Motor motor)
      : constants_(elevator), motor_(motor) {};

  AngularVelocityUnit motor_velocity(VelocityUnit velocity) const;

  AccelerationUnit acceleration(VelocityUnit velocity,
                                VoltageUnit voltage) const;

  AccelerationUnit maximum_acceleration() const;

  VelocityCoefficient velocity_coefficient() const;

  VoltageCoefficient voltage_coefficient() const;

  VoltageUnit oppose_steady_state(AccelerationUnit acceleration) const;

  ForceUnit force(VelocityUnit velocity, VoltageUnit voltage) const;

  CurrentUnit current(VelocityUnit velocity, VoltageUnit voltage) const;

  VoltageUnit limited_voltage(VelocityUnit velocity, VoltageUnit voltage) const;

  SystemMatrix system_matrix() const;

  InputMatrix input_matrix() const;
};

struct ElevatorState {
  Eigen::Vector<double, 2> state_;

  ElevatorState(DisplacementUnit position, VelocityUnit velocity) {
    set_position(position);
    set_velocity(velocity);
  }
  ElevatorState &operator=(const Eigen::Vector<double, 2> &state) {
    state_[0] = state[0];
    state_[1] = state[1];
    return *this;
  }

  DisplacementUnit position() const { return meters(state_[0]); }
  VelocityUnit velocity() const { return meters_per_second(state_[1]); }
  void set_position(DisplacementUnit position) {
    state_[0] = position.in(meters);
  }
  void set_velocity(VelocityUnit velocity) {
    state_[1] = velocity.in(meters / second);
  }
};

struct ElevatorInput {
  Eigen::Vector<double, 2> input_;

  ElevatorInput(VoltageUnit voltage, AccelerationUnit acceleration) {
    set_voltage(voltage);
    set_acceleration(acceleration);
  }

  VoltageUnit voltage() const { return volts(input_[0]); }
  AccelerationUnit acceleration() {
    return (meters / squared(second))(input_[1]);
  }
  void set_voltage(VoltageUnit voltage) { input_[0] = voltage.in(volts); }
  void set_acceleration(AccelerationUnit acceleration) {
    input_[1] = acceleration.in(meters / squared(second));
  }
};

class ElevatorSim {
private:
  SystemMatrix continuous_system_;
  InputMatrix continuous_input_;
  SystemMatrix discrete_system_;
  InputMatrix discrete_input_;

public:
  TimeUnit time_step_;
  ElevatorState state_;
  ElevatorInput input_;
  Elevator elevator_;

  ElevatorSim(const Elevator &elevator, TimeUnit time_step);

  void update(VoltageUnit voltage);
};
