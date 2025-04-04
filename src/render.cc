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

void DrawHorizontalTubeUpZ(Vector3 origin, DisplacementUnit length) {
  origin.y += (kTubeHeight / 2).in(vu);
  origin.z += (kTubeWidth / 2).in(vu);
  DrawCube(origin, length.in(vu), kTubeHeight.in(vu), kTubeWidth.in(vu),
           k5112Green);
  DrawCubeWires(origin, length.in(vu), kTubeHeight.in(vu), kTubeWidth.in(vu),
                BLACK);
}

void DrawHorizontalTubeUpX(Vector3 origin, DisplacementUnit length) {
  origin.y += (kTubeHeight / 2).in(vu);
  origin.x += (kTubeWidth / 2).in(vu);
  DrawCube(origin, kTubeWidth.in(vu), kTubeHeight.in(vu), length.in(vu),
           k5112Green);
  DrawCubeWires(origin, kTubeWidth.in(vu), kTubeHeight.in(vu), length.in(vu),
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
  origin.z += (kThinTubeHeight / 2).in(vu);

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
  origin.y -= kTubeHeight.in(vu);
  origin.z -= kTubeHeight.in(vu);
  origin.z -= kStageOneStandoffLength.in(vu);
  DrawHorizontalTubeUpZ(origin, kStageOneInnerWidth + 2 * kTubeWidth);
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

void DrawBase(Vector3 origin) {
  DrawCube(origin, kBaseSize.in(vu), kBaseThickness.in(vu), kBaseSize.in(vu),
           GRAY);
  DrawCubeWires(origin, kBaseSize.in(vu), kBaseThickness.in(vu),
                kBaseSize.in(vu), BLACK);
}

void DrawFrameTubes(Vector3 origin) {
  Vector3 frame_origin;
  frame_origin = origin;
  frame_origin.z += kFrameTubeDistance.in(vu);
  DrawHorizontalTubeUpZ(frame_origin, kFrameTubeLength);
  frame_origin = origin;
  frame_origin.z -= kFrameTubeDistance.in(vu);
  frame_origin.z -= kTubeWidth.in(vu);
  DrawHorizontalTubeUpZ(frame_origin, kFrameTubeLength);
  frame_origin = origin;
  frame_origin.x += kFrameTubeDistance.in(vu);
  DrawHorizontalTubeUpX(frame_origin, kFrameTubeLength - 2 * kTubeWidth);
  frame_origin = origin;
  frame_origin.x -= kFrameTubeDistance.in(vu);
  frame_origin.x -= kTubeWidth.in(vu);
  DrawHorizontalTubeUpX(frame_origin, kFrameTubeLength - 2 * kTubeWidth);
}

void DrawRobot(DisplacementUnit elevator_position) {
  Vector3 origin = {0, 0, 0};

  Vector3 base_origin = origin;
  base_origin.y += kBaseToFloor.in(vu);
  DrawBase(base_origin);

  Vector3 frame_origin = base_origin;
  frame_origin.y += kFrameToBase.in(vu);
  DrawFrameTubes(frame_origin);

  Vector3 stage_one_origin = frame_origin;
  stage_one_origin.y += kStageOneToFrame.in(vu);
  DrawStageOne(stage_one_origin);

  double elevator_percent = elevator_position / kTotalTravel;

  Vector3 stage_two_origin = stage_one_origin;
  stage_two_origin.y +=
      (kStageTwoToStageOneAtBottom + elevator_percent * kStageTwoTravel).in(vu);
  DrawStageTwo(stage_two_origin);

  Vector3 stage_three_origin = stage_two_origin;
  stage_three_origin.y +=
      (kStageThreeToStageTwoAtBottom + elevator_percent * kStageThreeTravel)
          .in(vu);
  DrawStageThree(stage_three_origin);

  Vector3 carriage_origin = stage_three_origin;
  carriage_origin.y +=
      (kCarriageToStageThreeAtBottom + elevator_percent * kCarriageTravel)
          .in(vu);
  DrawCarriage(carriage_origin);
}
