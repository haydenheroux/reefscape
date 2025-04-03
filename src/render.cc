#include "render.hh"
#include "raylib.h"
#include "units.hh"
#include <cassert>

void DrawStandoff(Vector3 start, DisplacementUnit length,
                  DisplacementUnit radius) {
  Vector3 end = start;
  end.z -= length.in(vu);
  DrawCylinderEx(start, end, radius.in(vu), radius.in(vu), 16, BLACK);
}

void DrawStandoffs(Vector3 origin, DisplacementUnit length,
                   DisplacementUnit radius, DisplacementUnit inner_width) {
  origin.y -= inches(0.5).in(vu);
  origin.z -= (kTubeHeight / 2).in(vu);

  DisplacementUnit half_offset = (inner_width / 2);

  Vector3 left = origin;
  left.x -= half_offset.in(vu);

  Vector3 right = origin;
  right.x += half_offset.in(vu);

  DrawStandoff(left, length, radius);
  DrawStandoff(right, length, radius);

  left.y -= inches(1.0).in(vu);
  right.y -= inches(1.0).in(vu);

  DrawStandoff(left, length, radius);
  DrawStandoff(right, length, radius);
}

void DrawVerticalTubes(Vector3 origin, DisplacementUnit length,
                       DisplacementUnit inner_width) {
  origin.y += (length / 2).in(vu);

  DisplacementUnit half_offset = (inner_width / 2 + kTubeWidth / 2);

  Vector3 left = origin;
  left.x -= half_offset.in(vu);

  Vector3 right = origin;
  right.x += half_offset.in(vu);

  DrawCube(left, kTubeWidth.in(vu), length.in(vu), kTubeHeight.in(vu),
           k5112Green);
  DrawCubeWires(left, kTubeWidth.in(vu), length.in(vu), kTubeHeight.in(vu),
                BLACK);
  DrawCube(right, kTubeWidth.in(vu), length.in(vu), kTubeHeight.in(vu),
           k5112Green);
  DrawCubeWires(right, kTubeWidth.in(vu), length.in(vu), kTubeHeight.in(vu),
                BLACK);
}

void DrawHorizontalTubeUp(Vector3 origin, DisplacementUnit length) {
  origin.y -= kTubeWidth.in(vu);
  origin.z -= kTubeHeight.in(vu);
  origin.z += (kTubeWidth / 2).in(vu);
  DrawCube(origin, length.in(vu), kTubeHeight.in(vu), kTubeWidth.in(vu),
           k5112Green);
  DrawCubeWires(origin, length.in(vu), kTubeHeight.in(vu), kTubeWidth.in(vu),
                BLACK);
}

void DrawHorizontalTubeFlat(Vector3 origin, DisplacementUnit length) {
  origin.y += (kTubeWidth / 2).in(vu);
  DrawCube(origin, length.in(vu), kTubeWidth.in(vu), kTubeHeight.in(vu),
           k5112Green);
  DrawCubeWires(origin, length.in(vu), kTubeWidth.in(vu), kTubeHeight.in(vu),
                BLACK);
}

void DrawThinTubesBack(Vector3 origin, DisplacementUnit thin_tube_length,
                       DisplacementUnit inner_width) {
  origin.y += (kThinTubeWidth / 2).in(vu);
  origin.z -= (thin_tube_length / 2 - kTubeHeight / 2).in(vu);

  DisplacementUnit half_offset = (inner_width / 2 + kThinTubeWidth / 2);

  Vector3 left = origin;
  left.x -= half_offset.in(vu);

  Vector3 right = origin;
  right.x += half_offset.in(vu);

  DrawCube(left, kThinTubeWidth.in(vu), kThinTubeHeight.in(vu),
           thin_tube_length.in(vu), k5112Green);
  DrawCubeWires(left, kThinTubeWidth.in(vu), kThinTubeHeight.in(vu),
                thin_tube_length.in(vu), BLACK);
  DrawCube(right, kThinTubeWidth.in(vu), kThinTubeHeight.in(vu),
           thin_tube_length.in(vu), k5112Green);
  DrawCubeWires(right, kThinTubeWidth.in(vu), kThinTubeHeight.in(vu),
                thin_tube_length.in(vu), BLACK);
}

void DrawThinTubeAcross(Vector3 origin, DisplacementUnit length) {
  origin.y += (kThinTubeWidth / 2).in(vu);
  origin.z += (kTubeHeight / 2).in(vu);

  DrawCube(origin, length.in(vu), kThinTubeWidth.in(vu), kThinTubeHeight.in(vu),
           k5112Green);
  DrawCubeWires(origin, length.in(vu), kThinTubeWidth.in(vu),
                kThinTubeHeight.in(vu), BLACK);
}

void DrawStageOne(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kStageOneInnerWidth + 2 * kTubeWidth);
  origin.y += kTubeWidth.in(vu);
  DrawVerticalTubes(origin, kStageOneHeight, kStageOneInnerWidth);
  origin.y += kStageOneHeight.in(vu);
  DrawStandoffs(origin, kStageOneStandoffLength, kStageOneStandoffRadius,
                kStageOneInnerWidth + kTubeWidth);
  origin.z -= kTubeHeight.in(vu);
  DrawHorizontalTubeUp(origin, kStageOneInnerWidth + 2 * kTubeWidth);
}

void DrawStageTwo(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kStageTwoInnerWidth);
  DrawVerticalTubes(origin, kStageTwoHeight, kStageTwoInnerWidth);
  origin.y += kStageTwoHeight.in(vu);
  DrawThinTubesBack(origin, kStageTwoThinTubeLength, kStageTwoInnerWidth);
  origin.z -= kStageTwoThinTubeLength.in(vu);
  DrawThinTubeAcross(origin, kStageTwoInnerWidth + 2 * kTubeWidth);
}

void DrawStageThree(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kStageThreeInnerWidth);
  DrawVerticalTubes(origin, kStageThreeHeight, kStageThreeInnerWidth);
  origin.y += (kStageThreeHeight - kTubeWidth).in(vu);
  DrawHorizontalTubeFlat(origin, kStageThreeInnerWidth);
}

void DrawCarriage(Vector3 origin) {
  DrawHorizontalTubeFlat(origin, kCarriageInnerWidth);
  DrawVerticalTubes(origin, kCarriageHeight, kCarriageInnerWidth);
  origin.y += (kCarriageHeight - kTubeWidth).in(vu);
  DrawHorizontalTubeFlat(origin, kCarriageInnerWidth);
}

void DrawElevatorStages(DisplacementUnit position) {
  double percent = position / kTotalTravel;

  Vector3 origin = {0, 0, 0};

  Vector3 test_origin = origin;
  test_origin.y = kTubeWidth.in(vu);

  Vector3 stage_one_origin = origin;
  stage_one_origin.y += kStageOneToFloor.in(vu);
  DrawStageOne(stage_one_origin);

  Vector3 stage_two_origin = stage_one_origin;
  stage_two_origin.y +=
      (kStageTwoToStageOneAtBottom + percent * kStageTwoTravel).in(vu);
  DrawStageTwo(stage_two_origin);

  Vector3 stage_three_origin = stage_two_origin;
  stage_three_origin.y +=
      (kStageThreeToStageTwoAtBottom + percent * kStageThreeTravel).in(vu);
  DrawStageThree(stage_three_origin);

  Vector3 carriage_origin = stage_three_origin;
  carriage_origin.y +=
      (kCarriageToStageThreeAtBottom + percent * kCarriageTravel).in(vu);
  DrawCarriage(carriage_origin);
}
