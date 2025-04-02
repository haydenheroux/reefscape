#pragma once

#include "au/math.hh"
#include "au/units/inches.hh"
#include "raylib.h"
#include "units.hh"

const Color k5112Green = {0, 167, 74, 255};

const DisplacementUnit kWindowWidth = inches(48.0);
const DisplacementUnit kWindowHeight = inches(100.0);

const DisplacementUnit kElevatorStageThickness = inches(1);
const DisplacementUnit kManipulatorThickness = inches(0.236);

const DisplacementUnit kStageOneToFloor = inches(3.0625);
const DisplacementUnit kStageOneWidth = inches(26.0);
const DisplacementUnit kStageOneHeight = inches(25.0);

const DisplacementUnit kStageTwoWidth = inches(23.0);
const DisplacementUnit kStageTwoHeight = inches(27.0);
const DisplacementUnit kStageTwoToStageOneAtBottom = inches(-2);
const DisplacementUnit kStageTwoToStageOneAtTop = inches(-24.0);
const DisplacementUnit kStageTwoTravel =
    au::abs(kStageTwoToStageOneAtBottom - kStageTwoToStageOneAtTop);

const DisplacementUnit kStageThreeWidth = inches(20.0);
const DisplacementUnit kStageThreeHeight = inches(30.5);
const DisplacementUnit kStageThreeToStageTwoAtBottom = inches(-4.5);
const DisplacementUnit kStageThreeToStageTwoAtTop = inches(-26.5);
const DisplacementUnit kStageThreeTravel =
    au::abs(kStageThreeToStageTwoAtBottom - kStageThreeToStageTwoAtTop);

const DisplacementUnit kCarriageWidth = inches(17.0);
const DisplacementUnit kCarriageHeight = inches(6.0);
const DisplacementUnit kCarriageToStageThreeAtBottom = inches(23.5);
const DisplacementUnit kCarriageToStageThreeAtTop = inches(1.5);
const DisplacementUnit kCarriageTravel =
    au::abs(kCarriageToStageThreeAtBottom -
            kCarriageToStageThreeAtTop);

const DisplacementUnit manipulator_width = inches(6.472);
const DisplacementUnit manipulator_height = inches(13.774);
const DisplacementUnit kManipulatorToCarriage = inches(9.543);

void DrawElevatorStages(DisplacementUnit position);
