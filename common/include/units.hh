#pragma once

#include "au/fwd.hh"
#include "au/prefix.hh"
#include "au/quantity.hh"
#include "au/units/amperes.hh"
#include "au/units/grams.hh"
#include "au/units/meters.hh"
#include "au/units/newtons.hh"
#include "au/units/ohms.hh"
#include "au/units/radians.hh"
#include "au/units/revolutions.hh"
#include "au/units/seconds.hh"
#include "au/units/volts.hh"

namespace reefscape {

namespace units {

using TimeUnit = au::Seconds;

// TODO(hayden): Rename from Velocity to `Derivative`
template <typename U>
using Velocity = decltype(U{} / TimeUnit{});

// TODO(hayden): Use `Derivative<Derivative<U>>`
template <typename U>
using Acceleration = decltype(U{} / squared(TimeUnit{}));

using DisplacementUnit = au::Meters;
using LinearVelocityUnit = Velocity<DisplacementUnit>;
using LinearAccelerationUnit = Acceleration<DisplacementUnit>;

using AngleUnit = au::Radians;
using AngularVelocityUnit = Velocity<AngleUnit>;
using AngularAccelerationUnit = Acceleration<AngleUnit>;

using GearRatioUnit = decltype(au::Revolutions{} / au::Revolutions{});

// NOTE(hayden): This quantity maker is required because it is impossible to
// create a dimension-less unit
constexpr auto gear_ratio =
    au::QuantityMaker<decltype(au::Revolutions{} / au::Revolutions{})>{};

using VoltageUnit = au::Volts;
using CurrentUnit = au::Amperes;
using ResistanceUnit = au::Ohms;

using MassUnit = au::Kilo<au::Grams>;
using MomentOfInertiaUnit = decltype(MassUnit{} * squared(DisplacementUnit{}));

using ForceUnit = au::Newtons;
using TorqueUnit = decltype(ForceUnit{} * DisplacementUnit{});

// TODO(hayden): Evaluate if this quantity maker is needed
constexpr auto newton_meters =
    au::QuantityMaker<decltype(au::Newtons{} * au::Meters{})>{};

using TorqueConstantUnit = decltype(TorqueUnit{} / CurrentUnit{});
using AngularVelocityConstantUnit =
    decltype(AngularVelocityUnit{} / VoltageUnit{});

// TODO(hayden): Use VelocityCoefficient<U> and VoltageCoefficient<U>
using LinearVelocityCoefficientUnit =
    decltype(LinearVelocityUnit{} / TimeUnit{} / LinearVelocityUnit{});
using AngularVelocityCoefficientUnit =
    decltype(AngularVelocityUnit{} / TimeUnit{} / AngularVelocityUnit{});
using LinearVoltageCoefficientUnit =
    decltype(LinearVelocityUnit{} / TimeUnit{} / VoltageUnit{});
using AngularVoltageCoefficientUnit =
    decltype(AngularVelocityUnit{} / TimeUnit{} / VoltageUnit{});

}  // namespace units

namespace quantities {

using Time = au::QuantityD<units::TimeUnit>;
using Displacement = au::QuantityD<units::DisplacementUnit>;
using LinearVelocity = au::QuantityD<units::LinearVelocityUnit>;
using LinearAcceleration = au::QuantityD<units::LinearAccelerationUnit>;
using Angle = au::QuantityD<units::AngleUnit>;
using AngularVelocity = au::QuantityD<units::AngularVelocityUnit>;
using AngularAcceleration = au::QuantityD<units::AngularAccelerationUnit>;
using GearRatio = au::QuantityD<units::GearRatioUnit>;
using Voltage = au::QuantityD<units::VoltageUnit>;
using Current = au::QuantityD<units::CurrentUnit>;
using Resistance = au::QuantityD<units::ResistanceUnit>;
using Mass = au::QuantityD<units::MassUnit>;
using MomentOfInertia = au::QuantityD<units::MomentOfInertiaUnit>;
using Force = au::QuantityD<units::ForceUnit>;
using Torque = au::QuantityD<units::TorqueUnit>;
using TorqueConstant = au::QuantityD<units::TorqueConstantUnit>;
using AngularVelocityConstant =
    au::QuantityD<units::AngularVelocityConstantUnit>;
using LinearVelocityCoefficient =
    au::QuantityD<units::LinearVelocityCoefficientUnit>;
using AngularVelocityCoefficient =
    au::QuantityD<units::AngularVelocityCoefficientUnit>;
using LinearVoltageCoefficient =
    au::QuantityD<units::LinearVoltageCoefficientUnit>;
using AngularVoltageCoefficient =
    au::QuantityD<units::AngularVoltageCoefficientUnit>;

}  // namespace quantities

}  // namespace reefscape
