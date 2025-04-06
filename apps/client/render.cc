#include "render.hh"
#include "common.hh"
#include "raylib.h"
#include "units.hh"
#include <cassert>

namespace render {
using namespace robot;

void DrawStandoff(Vector3 start, DisplacementUnit length,
                  DisplacementUnit radius) {
  Vector3 end = start;
  end.z -= length.in(raylib_unit);
  DrawCylinderEx(start, end, radius.in(raylib_unit), radius.in(raylib_unit), 16,
                 BLACK);
}

void DrawStandoffs(Vector3 origin, DisplacementUnit length,
                   DisplacementUnit radius, DisplacementUnit inner_width) {
  origin.y -= inches(0.5).in(raylib_unit);
  origin.z -= (kTubeHeight / 2).in(raylib_units);

  DisplacementUnit half_offset = (inner_width / 2);

  Vector3 left = origin;
  left.x -= half_offset.in(raylib_unit);

  Vector3 right = origin;
  right.x += half_offset.in(raylib_unit);

  DrawStandoff(left, length, radius);
  DrawStandoff(right, length, radius);

  left.y -= inches(1.0).in(raylib_unit);
  right.y -= inches(1.0).in(raylib_unit);

  DrawStandoff(left, length, radius);
  DrawStandoff(right, length, radius);
}

void DrawVerticalTubes(Vector3 origin, DisplacementUnit length,
                       DisplacementUnit inner_width) {
  origin.y += (length / 2).in(raylib_unit);

  DisplacementUnit half_offset = (inner_width / 2 + kTubeWidth / 2);

  Vector3 left = origin;
  left.x -= half_offset.in(raylib_unit);

  Vector3 right = origin;
  right.x += half_offset.in(raylib_unit);

  DrawCube(left, kTubeWidth.in(raylib_unit), length.in(raylib_unit),
           kTubeHeight.in(raylib_unit), k5112Green);
  DrawCubeWires(left, kTubeWidth.in(raylib_unit), length.in(raylib_unit),
                kTubeHeight.in(raylib_unit), BLACK);
  DrawCube(right, kTubeWidth.in(raylib_unit), length.in(raylib_unit),
           kTubeHeight.in(raylib_unit), k5112Green);
  DrawCubeWires(right, kTubeWidth.in(raylib_unit), length.in(raylib_unit),
                kTubeHeight.in(raylib_unit), BLACK);
}

void DrawHorizontalTubeUpZ(Vector3 origin, DisplacementUnit length) {
  origin.y += (kTubeHeight / 2).in(raylib_units);
  origin.z += (kTubeWidth / 2).in(raylib_units);
  DrawCube(origin, length.in(raylib_unit), kTubeHeight.in(raylib_unit),
           kTubeWidth.in(raylib_unit), k5112Green);
  DrawCubeWires(origin, length.in(raylib_unit), kTubeHeight.in(raylib_unit),
                kTubeWidth.in(raylib_unit), BLACK);
}

void DrawHorizontalTubeUpX(Vector3 origin, DisplacementUnit length) {
  origin.y += (kTubeHeight / 2).in(raylib_units);
  origin.x += (kTubeWidth / 2).in(raylib_units);
  DrawCube(origin, kTubeWidth.in(raylib_units), kTubeHeight.in(raylib_units),
           length.in(raylib_units), k5112Green);
  DrawCubeWires(origin, kTubeWidth.in(raylib_units),
                kTubeHeight.in(raylib_units), length.in(raylib_units), BLACK);
}

void DrawHorizontalTubeFlat(Vector3 origin, DisplacementUnit length) {
  origin.y += (kTubeWidth / 2).in(raylib_units);
  DrawCube(origin, length.in(raylib_units), kTubeWidth.in(raylib_units),
           kTubeHeight.in(raylib_units), k5112Green);
  DrawCubeWires(origin, length.in(raylib_units), kTubeWidth.in(raylib_units),
                kTubeHeight.in(raylib_units), BLACK);
}

void DrawThinTubesBack(Vector3 origin, DisplacementUnit thin_tube_length,
                       DisplacementUnit inner_width) {
  origin.y += (kThinTubeWidth / 2).in(raylib_units);
  origin.z -= (thin_tube_length / 2 - kTubeHeight / 2).in(raylib_units);

  DisplacementUnit half_offset = (inner_width / 2 + kThinTubeWidth / 2);

  Vector3 left = origin;
  left.x -= half_offset.in(raylib_unit);

  Vector3 right = origin;
  right.x += half_offset.in(raylib_unit);

  DrawCube(left, kThinTubeWidth.in(raylib_units),
           kThinTubeHeight.in(raylib_units), thin_tube_length.in(raylib_units),
           k5112Green);
  DrawCubeWires(left, kThinTubeWidth.in(raylib_units),
                kThinTubeHeight.in(raylib_units),
                thin_tube_length.in(raylib_units), BLACK);
  DrawCube(right, kThinTubeWidth.in(raylib_units),
           kThinTubeHeight.in(raylib_units), thin_tube_length.in(raylib_units),
           k5112Green);
  DrawCubeWires(right, kThinTubeWidth.in(raylib_units),
                kThinTubeHeight.in(raylib_units),
                thin_tube_length.in(raylib_units), BLACK);
}

void DrawThinTubeAcross(Vector3 origin, DisplacementUnit length) {
  origin.y += (kThinTubeWidth / 2).in(raylib_units);
  origin.z += (kThinTubeHeight / 2).in(raylib_units);

  DrawCube(origin, length.in(raylib_units), kThinTubeWidth.in(raylib_units),
           kThinTubeHeight.in(raylib_units), k5112Green);
  DrawCubeWires(origin, length.in(raylib_units),
                kThinTubeWidth.in(raylib_units),
                kThinTubeHeight.in(raylib_units), BLACK);
}

void DrawStageOne(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kStageOneInnerWidth + 2 * kTubeWidth);
  origin.y += kTubeWidth.in(raylib_units);
  DrawVerticalTubes(origin, kStageOneHeight, kStageOneInnerWidth);
  origin.y += kStageOneHeight.in(raylib_units);
  DrawStandoffs(origin, kStageOneStandoffLength, kStageOneStandoffRadius,
                kStageOneInnerWidth + kTubeWidth);
  origin.y -= kTubeHeight.in(raylib_units);
  origin.z -= kTubeHeight.in(raylib_units);
  origin.z -= kStageOneStandoffLength.in(raylib_units);
  DrawHorizontalTubeUpZ(origin, kStageOneInnerWidth + 2 * kTubeWidth);
}

void DrawStageTwo(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kStageTwoInnerWidth);
  DrawVerticalTubes(origin, kStageTwoHeight, kStageTwoInnerWidth);
  origin.y += kStageTwoHeight.in(raylib_units);
  DrawThinTubesBack(origin, kStageTwoThinTubeLength, kStageTwoInnerWidth);
  origin.z -= kStageTwoThinTubeLength.in(raylib_units);
  DrawThinTubeAcross(origin, kStageTwoInnerWidth + 2 * kTubeWidth);
}

void DrawStageThree(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kStageThreeInnerWidth);
  DrawVerticalTubes(origin, kStageThreeHeight, kStageThreeInnerWidth);
  origin.y += (kStageThreeHeight - kTubeWidth).in(raylib_units);
  DrawHorizontalTubeFlat(origin, kStageThreeInnerWidth);
}

void DrawCarriage(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kCarriageInnerWidth);
  DrawVerticalTubes(origin, kCarriageHeight, kCarriageInnerWidth);
  origin.y += (kCarriageHeight - kTubeWidth).in(raylib_units);
  DrawHorizontalTubeFlat(origin, kCarriageInnerWidth);
}

void DrawBase(Vector3 origin) {
  DrawCube(origin, kBaseSize.in(raylib_units), kBaseThickness.in(raylib_units),
           kBaseSize.in(raylib_units), GRAY);
  DrawCubeWires(origin, kBaseSize.in(raylib_units),
                kBaseThickness.in(raylib_units), kBaseSize.in(raylib_units),
                BLACK);
}

void DrawFrameTubes(Vector3 origin) {
  Vector3 frame_origin;
  frame_origin = origin;
  frame_origin.z += kFrameTubeDistance.in(raylib_units);
  DrawHorizontalTubeUpZ(frame_origin, kFrameTubeLength);
  frame_origin = origin;
  frame_origin.z -= kFrameTubeDistance.in(raylib_units);
  frame_origin.z -= kTubeWidth.in(raylib_units);
  DrawHorizontalTubeUpZ(frame_origin, kFrameTubeLength);
  frame_origin = origin;
  frame_origin.x += kFrameTubeDistance.in(raylib_units);
  DrawHorizontalTubeUpX(frame_origin, kFrameTubeLength - 2 * kTubeWidth);
  frame_origin = origin;
  frame_origin.x -= kFrameTubeDistance.in(raylib_units);
  frame_origin.x -= kTubeWidth.in(raylib_units);
  DrawHorizontalTubeUpX(frame_origin, kFrameTubeLength - 2 * kTubeWidth);
}

void DrawRobot(DisplacementUnit elevator_position) {
  Vector3 origin = {0, 0, 0};

  Vector3 base_origin = origin;
  base_origin.y += kBaseToFloor.in(raylib_units);
  DrawBase(base_origin);

  Vector3 frame_origin = base_origin;
  frame_origin.y += kFrameToBase.in(raylib_units);
  DrawFrameTubes(frame_origin);

  Vector3 stage_one_origin = frame_origin;
  stage_one_origin.y += kStageOneToFrame.in(raylib_units);
  DrawStageOne(stage_one_origin);

  double elevator_percent = elevator_position / kTotalTravel;

  Vector3 stage_two_origin = stage_one_origin;
  stage_two_origin.y +=
      (kStageTwoToStageOneAtBottom + elevator_percent * kStageTwoTravel)
          .in(raylib_units);
  DrawStageTwo(stage_two_origin);

  Vector3 stage_three_origin = stage_two_origin;
  stage_three_origin.y +=
      (kStageThreeToStageTwoAtBottom + elevator_percent * kStageThreeTravel)
          .in(raylib_units);
  DrawStageThree(stage_three_origin);

  Vector3 carriage_origin = stage_three_origin;
  carriage_origin.y +=
      (kCarriageToStageThreeAtBottom + elevator_percent * kCarriageTravel)
          .in(raylib_units);
  DrawCarriage(carriage_origin);
}
}; // namespace render
