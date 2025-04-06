#include "Elevator.hh"
#include "Eigen.hh"
#include "au/power_aliases.hh"
#include "units.hh"
#include <ctime>

namespace sim {
using namespace units;
AngularVelocityUnit Elevator::MotorVelocity(VelocityUnit velocity) const {
  return velocity * radians(1) * gear_ratio / drum_radius;
}

AccelerationUnit Elevator::Acceleration(VelocityUnit velocity,
                                        VoltageUnit voltage) const {
  return Force(velocity, voltage) / mass;
}

AccelerationUnit Elevator::MaximumAcceleration() const {
  VelocityUnit velocity = (meters / second)(0);
  VoltageUnit voltage = LimitVoltage(velocity, volts(12));
  return Acceleration(velocity, voltage);
}

VelocityCoefficientUnit Elevator::VelocityCoefficient() const {
  return -1 * (gear_ratio * gear_ratio * motor.torque_constant_ * radians(1)) /
         (motor.resistance_ * drum_radius * drum_radius * mass *
          motor.angular_velocity_constant_);
}

VoltageCoefficientUnit Elevator::VoltageCoefficient() const {
  return (gear_ratio * motor.torque_constant_) /
         (motor.resistance_ * mass * drum_radius);
}

VoltageUnit Elevator::OpposingVoltage(AccelerationUnit acceleration) const {
  return -acceleration / VoltageCoefficient();
}

ForceUnit Elevator::Force(VelocityUnit velocity, VoltageUnit voltage) const {
  ForceUnit voltage_force = (gear_ratio * motor.torque_constant_ * voltage) /
                            (motor.resistance_ * drum_radius);

  ForceUnit back_emf_force = -1 *
                             (gear_ratio * gear_ratio * motor.torque_constant_ *
                              velocity * radians(1)) /
                             (motor.resistance_ * drum_radius * drum_radius *
                              motor.angular_velocity_constant_);

  return voltage_force + back_emf_force;
}

CurrentUnit Elevator::Current(VelocityUnit velocity,
                              VoltageUnit voltage) const {
  return voltage / motor.resistance_ -
         MotorVelocity(velocity) /
             (motor.angular_velocity_constant_ * motor.resistance_);
}

VoltageUnit Elevator::LimitVoltage(VelocityUnit velocity,
                                   VoltageUnit voltage) const {
  CurrentUnit current = Current(velocity, voltage);
  if (current > max_current) {
    voltage = max_current * motor.resistance_ +
              MotorVelocity(velocity) / motor.angular_velocity_constant_;
  }
  return voltage;
}

ElevatorSim::ElevatorSim(const Elevator &elevator, AccelerationUnit gravity,
                         TimeUnit time_step)
    : time_step_(time_step), state(meters(0), (meters / second)(0)),
      input(volts(0)), elevator_(elevator) {
  continuous_system_ << 0, 1, 0,
      elevator.VelocityCoefficient().in((meters / squared(second)) /
                                        (meters / second));
  continuous_input_ << 0,
      elevator.VoltageCoefficient().in((meters / squared(second)) / volt);

  continuous_input_pseudoinverse_ = PseudoInverse(continuous_input_);

  auto continuous_matrices =
      std::make_pair(continuous_system_, continuous_input_);
  auto discretized_matrices = Discretize(continuous_matrices, time_step_);
  discrete_system_ = discretized_matrices.first;
  discrete_input_ = discretized_matrices.second;

  continuous_gravity_ << 0, gravity.in(meters / squared(second));
  discrete_gravity_ << discrete_input_ * continuous_input_pseudoinverse_ *
                           continuous_gravity_;
}

ElevatorSim::Input ElevatorSim::OpposingGravity() const {
  return {-1 * continuous_input_pseudoinverse_ * continuous_gravity_};
}

void ElevatorSim::Update() {
  state = discrete_system_ * state.vector + discrete_input_ * input.vector +
           discrete_gravity_;
  state = state.Clamped(meters(0), elevator_.max_travel);
}
}; // namespace sim
