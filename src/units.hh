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

using namespace au;

using TimeUnit = QuantityD<Seconds>;

using DisplacementUnit = QuantityD<Meters>;
using VelocityUnit = QuantityD<decltype(Meters{} / Seconds{})>;
using AccelerationUnit = QuantityD<decltype(Meters{} / squared(Seconds{}))>;

using AngleUnit = QuantityD<Radians>;
using AngularVelocityUnit = QuantityD<decltype(Radians{} / Seconds{})>;
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

using VelocityCoefficientUnit =
    QuantityD<decltype((Meters{} / squared(Seconds{})) /
                       (Meters{} / Seconds{}))>;

using VoltageCoefficientUnit =
    QuantityD<decltype((Meters{} / squared(Seconds{})) / (Volts{}))>;

struct Pixels : decltype(Meters{} / mag<256>()) {
  static constexpr const char label[] = "px";
};
constexpr auto pixel = SingularNameFor<Pixels>{};
constexpr auto pixels = QuantityMaker<Pixels>{};
constexpr auto pixels_pt = QuantityPointMaker<Pixels>{};

struct ViewportUnits : decltype(Meters{} / mag<4>()) {
  static constexpr const char label[] = "vu";
};
constexpr auto vu = SingularNameFor<ViewportUnits>{};
constexpr auto viewport_units = QuantityMaker<ViewportUnits>{};
constexpr auto viewport_unit_pt = QuantityPointMaker<ViewportUnits>{};
