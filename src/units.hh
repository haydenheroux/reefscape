#pragma once

#include "au/fwd.hh"
#include "au/prefix.hh"
#include "au/quantity.hh"
#include "au/units/amperes.hh"
#include "au/units/grams.hh"
#include "au/units/meters.hh"
#include "au/units/minutes.hh"
#include "au/units/newtons.hh"
#include "au/units/ohms.hh"
#include "au/units/radians.hh"
#include "au/units/revolutions.hh"
#include "au/units/seconds.hh"
#include "au/units/volts.hh"

using namespace au;

using TimeUnit = QuantityD<Seconds>;

using DisplacementUnit = QuantityD<Meters>;
using VelocityUnit = QuantityD<decltype(Meters{} / Seconds{})>;
constexpr auto meters_per_second = QuantityMaker<decltype(Meters{} / Seconds{})>{};
using AccelerationUnit = QuantityD<decltype(Meters{} / squared(Seconds{}))>;

using AngleUnit = QuantityD<Radians>;
using AngularVelocityUnit = QuantityD<decltype(Radians{} / Seconds{})>;
constexpr auto rpm = QuantityMaker<decltype(Revolutions{} / Minutes{})>{};
using RatioUnit = QuantityD<decltype(Revolutions{} / Revolutions{})>;
constexpr auto gear_ratio =
    QuantityMaker<decltype(Revolutions{} / Revolutions{})>{};
using AngularVelocityConstantUnit =
    QuantityD<decltype(Radians{} / Seconds{} / Volts{})>;

using VoltageUnit = QuantityD<Volts>;
using CurrentUnit = QuantityD<Amperes>;
using ResistanceUnit = QuantityD<Ohms>;

using MassUnit = QuantityD<Kilo<Grams>>;

using TorqueUnit = QuantityD<decltype(Newtons{} * Meters{})>;
constexpr auto newton_meters = QuantityMaker<decltype(Newtons{} * Meters{})>{};
using TorqueConstantUnit =
    QuantityD<decltype(Newtons{} * Meters{} / Amperes{})>;
using ForceUnit = QuantityD<Newtons>;

using VelocityCoefficient = QuantityD<decltype((Meters{} / squared(Seconds{})) /
                                               (Meters{} / Seconds{}))>;
constexpr auto velocity_coefficient = (meters / squared(second)) / (meters / second);

using VoltageCoefficient =
    QuantityD<decltype((Meters{} / squared(Seconds{})) / (Volts{}))>;
constexpr auto voltage_coefficient = (meters / squared(second)) / (volt);
