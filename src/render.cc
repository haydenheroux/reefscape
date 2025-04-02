#include "render.hh"

void DrawElevatorStages(DisplacementUnit position) {
  DisplacementUnit carriage_inside_stage_three;
  if (position < kCarriageTravel) {
    carriage_inside_stage_three =
        kCarriageToStageThreeAtBottom - position;
    position = meters(0);
  } else {
    carriage_inside_stage_three = kCarriageToStageThreeAtTop;
    position -= kCarriageTravel;
  }
  DisplacementUnit stage_three_inside_stage_two;
  if (position < kStageThreeTravel) {
    stage_three_inside_stage_two =
        kStageThreeToStageTwoAtBottom - position;
    position = meters(0);
  } else {
    stage_three_inside_stage_two = kStageThreeToStageTwoAtTop;
    position -= kStageThreeTravel;
  }
  DisplacementUnit stage_two_inside_stage_one;
  if (position < kStageTwoTravel) {
    stage_two_inside_stage_one =
        kStageTwoToStageOneAtBottom - position;
    position = meters(0);
  } else {
    stage_two_inside_stage_one = kStageTwoToStageOneAtTop;
    position -= kStageTwoTravel;
  }

  DisplacementUnit stage_one_position =
      kWindowHeight - kStageOneHeight - kStageOneToFloor;
  DisplacementUnit stage_two_position =
      stage_one_position + stage_two_inside_stage_one;
  DisplacementUnit stage_three_position =
      stage_two_position + stage_three_inside_stage_two;
  DisplacementUnit carriage_position =
      stage_three_position + carriage_inside_stage_three;
  DisplacementUnit manipulator_position =
      carriage_position - kManipulatorToCarriage;

  Rectangle stage_one;
  stage_one.x = (kWindowWidth / 2 - kStageOneWidth / 2).in(pixels);
  stage_one.y = stage_one_position.in(pixels);
  stage_one.width = kStageOneWidth.in(pixels);
  stage_one.height = kStageOneHeight.in(pixels);

  Rectangle stage_two;
  stage_two.x = (kWindowWidth / 2 - kStageTwoWidth / 2).in(pixels);
  stage_two.y = stage_two_position.in(pixels);
  stage_two.width = kStageTwoWidth.in(pixels);
  stage_two.height = kStageTwoHeight.in(pixels);

  Rectangle stage_three;
  stage_three.x = (kWindowWidth / 2 - kStageThreeWidth / 2).in(pixels);
  stage_three.y = stage_three_position.in(pixels);
  stage_three.width = kStageThreeWidth.in(pixels);
  stage_three.height = kStageThreeHeight.in(pixels);

  Rectangle carriage;
  carriage.x = (kWindowWidth / 2 - kCarriageWidth / 2).in(pixels);
  carriage.y = carriage_position.in(pixels);
  carriage.width = kCarriageWidth.in(pixels);
  carriage.height = kCarriageHeight.in(pixels);

  Rectangle manipulator;
  manipulator.x = (kWindowWidth / 2 - manipulator_width / 2).in(pixels);
  manipulator.y = manipulator_position.in(pixels);
  manipulator.width = manipulator_width.in(pixels);
  manipulator.height = manipulator_height.in(pixels);

  DrawRectangleLinesEx(stage_one, kElevatorStageThickness.in(pixels), k5112Green);
  DrawRectangleLinesEx(stage_two, kElevatorStageThickness.in(pixels), k5112Green);
  DrawRectangleLinesEx(stage_three, kElevatorStageThickness.in(pixels), k5112Green);
  DrawRectangleLinesEx(carriage, kElevatorStageThickness.in(pixels), k5112Green);
  DrawRectangleLinesEx(manipulator, kManipulatorThickness.in(pixels), GRAY);
}
