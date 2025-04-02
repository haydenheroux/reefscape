#include "Elevator.hh"
#include "Eigen.hh"
#include "units.hh"

AngularVelocityUnit Elevator::motor_velocity(VelocityUnit velocity) const {
  return velocity * radians(1) * constants_.gear_ratio_ /
         constants_.drum_radius_;
}

AccelerationUnit Elevator::acceleration(VelocityUnit velocity,
                                        VoltageUnit voltage) const {
  return force(velocity, voltage) / constants_.mass_;
}

AccelerationUnit Elevator::maximum_acceleration() const {
  VelocityUnit velocity = meters_per_second(0);
  VoltageUnit voltage = this->limited_voltage(velocity, volts(12));
  return acceleration(velocity, voltage);
}

VelocityCoefficient Elevator::velocity_coefficient() const {
  return -1 *
         (constants_.gear_ratio_ * constants_.gear_ratio_ *
          motor_.torque_constant_ * radians(1)) /
         (motor_.resistance_ * constants_.drum_radius_ *
          constants_.drum_radius_ * constants_.mass_ *
          motor_.angular_velocity_constant_);
}

VoltageCoefficient Elevator::voltage_coefficient() const {
  return (constants_.gear_ratio_ * motor_.torque_constant_) /
         (motor_.resistance_ * constants_.mass_ * constants_.drum_radius_);
}

VoltageUnit Elevator::oppose_steady_state(AccelerationUnit acceleration) const {
  return -acceleration / voltage_coefficient();
}

ForceUnit Elevator::force(VelocityUnit velocity, VoltageUnit voltage) const {
  ForceUnit voltage_force =
      (constants_.gear_ratio_ * motor_.torque_constant_ * voltage) /
      (motor_.resistance_ * constants_.drum_radius_);

  ForceUnit back_emf_force =
      -1 *
      (constants_.gear_ratio_ * constants_.gear_ratio_ *
       motor_.torque_constant_ * velocity * radians(1)) /
      (motor_.resistance_ * constants_.drum_radius_ * constants_.drum_radius_ *
       motor_.angular_velocity_constant_);

  return voltage_force + back_emf_force;
}

CurrentUnit Elevator::current(VelocityUnit velocity,
                              VoltageUnit voltage) const {
  return voltage / motor_.resistance_ -
         motor_velocity(velocity) /
             (motor_.angular_velocity_constant_ * motor_.resistance_);
}

VoltageUnit Elevator::limited_voltage(VelocityUnit velocity,
                                      VoltageUnit voltage) const {
  CurrentUnit current = this->current(velocity, voltage);
  if (current > constants_.max_current_) {
    voltage =
        constants_.max_current_ * motor_.resistance_ +
        this->motor_velocity(velocity) / motor_.angular_velocity_constant_;
  }
  return voltage;
}

SystemMatrix Elevator::system_matrix() const {
  SystemMatrix system_matrix;
  system_matrix << 0, 1, 0,
      this->velocity_coefficient().in((meters / squared(second)) /
                                      (meters / second));
  return system_matrix;
}

InputMatrix Elevator::input_matrix() const {
  InputMatrix input_matrix;
  input_matrix << 0,
      this->voltage_coefficient().in((meters / squared(second)) / volt);
  return input_matrix;
}

ElevatorSim::ElevatorSim(const Elevator &elevator, TimeUnit time_step)
    : time_step_(time_step), state_(meters(0), meters_per_second(0)),
      input_(volts(0), (meters / squared(second))(0)), elevator_(elevator) {
  continuous_system_ << 0, 1, 0,
      elevator.velocity_coefficient().in(velocity_coefficient);
  continuous_input_ << 0, 0,
      elevator.voltage_coefficient().in(voltage_coefficient), 1;

  auto continuous_matrices =
      std::make_pair(continuous_system_, continuous_input_);
  auto discretized = discretize(continuous_matrices, time_step_);
  discrete_system_ = discretized.first;
  discrete_input_ = discretized.second;
}

void ElevatorSim::update(VoltageUnit voltage) {
  input_.set_voltage(voltage);
  state_ = discrete_system_ * state_.state_ + discrete_input_ * input_.input_;
  // TODO Attempt to find a cleaner clamping method
  if (state_.position() > elevator_.constants_.max_travel_) {
    state_.set_position(elevator_.constants_.max_travel_);
  } else if (state_.position() < meters(0)) {
    state_.set_position(meters(0));
  }
}
