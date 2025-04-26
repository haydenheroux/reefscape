#pragma once

#include "units.hh"

namespace reefscape {

struct Pixels : decltype(Meters{} / mag<256>()) {
  static constexpr const char label[] = "px";
};
constexpr auto pixel = SingularNameFor<Pixels>{};
constexpr auto pixels = QuantityMaker<Pixels>{};
constexpr auto pixels_pt = QuantityPointMaker<Pixels>{};

struct RaylibUnits : decltype(Meters{} / mag<4>()) {
  static constexpr const char label[] = "vu";
};
constexpr auto raylib_unit = SingularNameFor<RaylibUnits>{};
constexpr auto raylib_units = QuantityMaker<RaylibUnits>{};
constexpr auto raylib_unit_pt = QuantityPointMaker<RaylibUnits>{};

};  // namespace reefscape
