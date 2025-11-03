#pragma once

#include <concepts>

#include "Motor.hh"
#include "au/fwd.hh"
#include "au/math.hh"
#include "au/unit_of_measure.hh"
#include "units.hh"

namespace reefscape {

template <typename T>
concept HasMotor = requires(const T& obj) {
  { obj.motor } -> std::convertible_to<Motor>;
};

template <typename T>
concept HasMaxCurrent = requires(const T& obj) {
  { obj.max_current } -> std::convertible_to<CurrentUnit>;
};

template <typename T, typename U>
concept HasMotorVelocity =
    requires(const T& obj, QuantityD<decltype(U{} / Seconds{})> v) {
      { obj.MotorVelocity(v) } -> std::same_as<AngularVelocityUnit>;
    };

template <typename T, typename U>
concept HasAcceleration = requires(
    const T& obj, QuantityD<decltype(U{} / Seconds{})> v, VoltageUnit u) {
  { obj.Acceleration(v, u) } -> std::same_as<AccelerationUnit>;
};

template <typename T, typename U>
concept HasVelocityCoefficient = requires(const T& obj) {
  {
    obj.VelocityCoefficient()
  } -> std::same_as<
      QuantityD<decltype(((U{} / Seconds{}) / Seconds{}) / (U{} / Seconds{}))>>;
};

template <typename T, typename U>
concept HasVoltageCoefficient = requires(const T& obj) {
  {
    obj.VoltageCoefficient()
  } -> std::same_as<
      QuantityD<decltype(((U{} / Seconds{}) / Seconds{}) / Volts{})>>;
};

template <typename T, typename U>
concept MotorSystem =
    HasMotor<T> && HasMaxCurrent<T> && HasMotorVelocity<T, U> &&
    HasAcceleration<T, U> && HasVelocityCoefficient<T, U> &&
    HasVoltageCoefficient<T, U>;

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
QuantityD<decltype(NativeUnit{} / Seconds{})> MaximumVelocity(
    const System& system) {
  // Maximize ω with dω/dt = (velocity_coefficient)·ω +
  // (voltage_coefficient)·(motor.nominal_voltage)
  return -1 * system.motor.nominal_voltage_ * system.VoltageCoefficient() /
         system.VelocityCoefficient();
}

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
QuantityD<decltype(NativeUnit{} / squared(Seconds{}))> MaximumAcceleration(
    const System& system) {
  // NOTE(hayden): Maximum motor torque (therefore maximum acceleration) occurs
  // when the motor is stationary.
  auto zero_velocity = QuantityMaker<decltype(NativeUnit{} / Seconds{})>{}(0.0);
  VoltageUnit limited_voltage =
      LimitVoltage(system, zero_velocity, system.motor.nominal_voltage_);
  return system.Acceleration(zero_velocity, limited_voltage);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * Seconds{})>
CurrentUnit Current(const System& system, QuantityD<VelocityUnit> velocity,
                    VoltageUnit voltage) {
  return voltage / system.motor.resistance_ -
         system.MotorVelocity(velocity) /
             (system.motor.angular_velocity_constant_ *
              system.motor.resistance_);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * Seconds{})>
VoltageUnit LimitVoltage(const System& system, QuantityD<VelocityUnit> velocity,
                         VoltageUnit voltage) {
  voltage = au::clamp(voltage, -system.motor.nominal_voltage_,
                      system.motor.nominal_voltage_);

  CurrentUnit current = Current(system, velocity, voltage);
  if (current > system.max_current) {
    voltage = system.max_current * system.motor.resistance_ +
              system.MotorVelocity(velocity) /
                  system.motor.angular_velocity_constant_;
  }
  return voltage;
}

}  // namespace reefscape
