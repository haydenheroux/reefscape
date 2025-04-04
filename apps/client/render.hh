#pragma once

#include "raylib.h"
#include "units.hh"

const Color k5112Green = {0, 167, 74, 255};
const Color k5112GreenShadow = {0, 148, 91, 255};

const DisplacementUnit kWindowWidth = meters(8);
const DisplacementUnit kWindowHeight = meters(4.5);

void DrawRobot(DisplacementUnit position);
