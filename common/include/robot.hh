#pragma once

#include "au/math.hh"
#include "au/units/inches.hh"
#include "units.hh"

namespace robot {
using namespace units;
const DisplacementUnit kTubeWidth = inches(1);
const DisplacementUnit kTubeHeight = inches(2);
const DisplacementUnit kThinTubeWidth = inches(1);
const DisplacementUnit kThinTubeHeight = inches(1);

const DisplacementUnit kManipulatorThickness = inches(0.236);

const DisplacementUnit kBaseToFloor = inches(1.77);
const DisplacementUnit kBaseSize = inches(26.0);
const DisplacementUnit kBaseThickness = inches(0.244);

const DisplacementUnit kFrameToBase = kBaseThickness;
const DisplacementUnit kFrameTubeLength = inches(26);
const DisplacementUnit kFrameTubeDistance = inches(12);

const DisplacementUnit kStageOneToFrame = kTubeHeight;
const DisplacementUnit kStageOneInnerWidth = inches(24.0);
const DisplacementUnit kStageOneHeight = inches(25.0);
const DisplacementUnit kStageOneStandoffLength = inches(2);
const DisplacementUnit kStageOneStandoffRadius = inches(0.1875);

const DisplacementUnit kStageTwoToStageOneAtBottom = kTubeWidth;
const DisplacementUnit kStageTwoToStageOneAtTop = inches(23);
const DisplacementUnit kStageTwoTravel =
    au::abs(kStageTwoToStageOneAtTop - kStageTwoToStageOneAtBottom);

const DisplacementUnit kStageTwoInnerWidth = inches(21.0);
const DisplacementUnit kStageTwoHeight = inches(27.0);
const DisplacementUnit kStageTwoThinTubeLength = inches(4.0);

const DisplacementUnit kStageThreeToStageTwoAtBottom = kTubeWidth;
const DisplacementUnit kStageThreeToStageTwoAtTop = inches(23);
const DisplacementUnit kStageThreeTravel =
    au::abs(kStageThreeToStageTwoAtTop - kStageThreeToStageTwoAtBottom);

const DisplacementUnit kStageThreeInnerWidth = inches(18.0);
const DisplacementUnit kStageThreeHeight = inches(30.5);

const DisplacementUnit kCarriageToStageThreeAtBottom = kTubeWidth;
const DisplacementUnit kCarriageToStageThreeAtTop = inches(23);
const DisplacementUnit kCarriageTravel =
    au::abs(kCarriageToStageThreeAtTop - kCarriageToStageThreeAtBottom);

const DisplacementUnit kCarriageInnerWidth = inches(15.0);
const DisplacementUnit kCarriageHeight = inches(6.0);

const DisplacementUnit kTotalTravel =
    kStageTwoTravel + kStageThreeTravel + kCarriageTravel;
}  // namespace robot
