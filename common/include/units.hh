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

using namespace au;

using TimeUnit = QuantityD<Seconds>;

using DisplacementUnit = QuantityD<Meters>;
using VelocityUnit = QuantityD<decltype(Meters{} / Seconds{})>;
using AccelerationUnit = QuantityD<decltype(Meters{} / squared(Seconds{}))>;

using AngleUnit = QuantityD<Radians>;
using AngularVelocityUnit = QuantityD<decltype(Radians{} / Seconds{})>;
using AngularAccelerationUnit = QuantityD<decltype(Radians{} / squared(Seconds{}))>;
using RatioUnit = QuantityD<decltype(Revolutions{} / Revolutions{})>;
constexpr auto gear_ratio =
    QuantityMaker<decltype(Revolutions{} / Revolutions{})>{};
using AngularVelocityConstantUnit =
    QuantityD<decltype(Radians{} / Seconds{} / Volts{})>;

using VoltageUnit = QuantityD<Volts>;
using CurrentUnit = QuantityD<Amperes>;
using ResistanceUnit = QuantityD<Ohms>;

using MassUnit = QuantityD<Kilo<Grams>>;
using MomentOfInertiaUnit = QuantityD<decltype(Kilo<Grams>{} * squared(Meters{}))>;

using TorqueUnit = QuantityD<decltype(Newtons{} * Meters{})>;
constexpr auto newton_meters = QuantityMaker<decltype(Newtons{} * Meters{})>{};
using TorqueConstantUnit =
    QuantityD<decltype(Newtons{} * Meters{} / Amperes{})>;
using ForceUnit = QuantityD<Newtons>;

using LinearVelocityCoefficientUnit =
    QuantityD<decltype((Meters{} / squared(Seconds{})) /
                       (Meters{} / Seconds{}))>;
using AngularVelocityCoefficientUnit = QuantityD<decltype((
    Revolutions{} / squared(Seconds{}) / (Revolutions{} / Seconds{})))>;

using LinearVoltageCoefficientUnit =
    QuantityD<decltype((Meters{} / squared(Seconds{})) / (Volts{}))>;
using AngularVoltageCoefficientUnit =
    QuantityD<decltype((Revolutions{} / squared(Seconds{})) / (Volts{}))>;

}  // namespace reefscape
