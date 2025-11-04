#pragma once

#include <concepts>

#include "Motor.hh"
#include "au/fwd.hh"
#include "au/math.hh"
#include "units.hh"

namespace reefscape {

template <typename S, typename U>
concept MotorSystem = requires(const S& system,
                               au::QuantityD<units::Velocity<U>> v,
                               quantities::Voltage u) {
  // NOTE(hayden): std::convertible_to is used here to ignore constness
  { system.motor } -> std::convertible_to<Motor>;
  { system.max_current } -> std::convertible_to<quantities::Current>;
  // TODO(hayden): Express this requirement in terms of au:: instead of units::
  { system.MotorVelocity(v) } -> std::same_as<quantities::AngularVelocity>;
  {
    system.Acceleration(v, u)
  } -> std::same_as<au::QuantityD<units::Acceleration<U>>>;
  {
    system.VelocityCoefficient()
  } -> std::same_as<
      au::QuantityD<decltype(units::Acceleration<U>{} / units::Velocity<U>{})>>;
  {
    system.VoltageCoefficient()
  } -> std::same_as<
      au::QuantityD<decltype(units::Acceleration<U>{} / units::VoltageUnit{})>>;
};

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<decltype(NativeUnit{} / units::TimeUnit{})> MaximumVelocity(
    const System& system) {
  // Maximize ω with dω/dt = (velocity_coefficient)·ω +
  // (voltage_coefficient)·(motor.nominal_voltage)
  return -1 * system.motor.nominal_voltage_ * system.VoltageCoefficient() /
         system.VelocityCoefficient();
}

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<units::Acceleration<NativeUnit>> MaximumAcceleration(
    const System& system) {
  // NOTE(hayden): Maximum motor torque (therefore maximum acceleration) occurs
  // when the motor is stationary.
  auto zero_velocity = au::QuantityMaker<units::Velocity<NativeUnit>>{}(0.0);
  auto limited_voltage =
      LimitVoltage(system, zero_velocity, system.motor.nominal_voltage_);
  return system.Acceleration(zero_velocity, limited_voltage);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * units::TimeUnit{})>
quantities::Current Current(const System& system,
                            au::QuantityD<VelocityUnit> velocity,
                            quantities::Voltage voltage) {
  return voltage / system.motor.resistance_ -
         system.MotorVelocity(velocity) /
             (system.motor.angular_velocity_constant_ *
              system.motor.resistance_);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * units::TimeUnit{})>
quantities::Voltage LimitVoltage(const System& system,
                                 au::QuantityD<VelocityUnit> velocity,
                                 quantities::Voltage voltage) {
  voltage = au::clamp(voltage, -system.motor.nominal_voltage_,
                      system.motor.nominal_voltage_);

  auto current = reefscape::Current(system, velocity, voltage);
  if (current > system.max_current) {
    voltage = system.max_current * system.motor.resistance_ +
              system.MotorVelocity(velocity) /
                  system.motor.angular_velocity_constant_;
  }
  return voltage;
}

}  // namespace reefscape
