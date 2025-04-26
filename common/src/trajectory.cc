#include "trajectory.hh"

namespace reefscape {

PositionVelocityState TrapezoidTrajectory::Calculate(
    TimeUnit time_step, PositionVelocityState state,
    PositionVelocityState goal) {
  // NOTE(hayden): Algorithm assumes positive motion
  bool flip = goal.Position() < state.Position();
  if (flip) {
    state.vector = -state.vector;
    goal.vector = -goal.vector;
  }

  if (state.Velocity() > max_velocity) {
    state.SetVelocity(max_velocity);
  }

  TimeUnit start_time = state.Velocity() / max_acceleration;
  DisplacementUnit start_distance =
      0.5 * start_time * start_time * max_acceleration;

  TimeUnit end_time = goal.Velocity() / max_acceleration;
  DisplacementUnit end_distance = 0.5 * end_time * end_time * max_acceleration;

  DisplacementUnit distance =
      start_distance + (goal.Position() - state.Position()) + end_distance;
  TimeUnit acceleration_time = max_velocity / max_acceleration;
  DisplacementUnit cruise_distance =
      distance - (acceleration_time * acceleration_time * max_acceleration);
  if (cruise_distance < meters(0)) {
    acceleration_time = au::sqrt(distance / max_acceleration);
    cruise_distance = meters(0);
  }
  TimeUnit end_acceleration = acceleration_time - start_time;
  TimeUnit end_cruise = end_acceleration + cruise_distance / max_velocity;
  TimeUnit end_deceleration = end_cruise + acceleration_time - end_time;

  PositionVelocityState result{state};

  if (time_step < end_acceleration) {
    result.SetPosition(result.Position() +
                       (state.Velocity() + 0.5 * time_step * max_acceleration) *
                           time_step);
    result.SetVelocity(result.Velocity() + time_step * max_acceleration);
  } else if (time_step <= end_cruise) {
    result.SetPosition(
        state.Position() +
        (state.Velocity() + 0.5 * end_acceleration * max_acceleration) *
            end_acceleration +
        max_velocity * (time_step - end_acceleration));
    result.SetVelocity(max_velocity);
  } else if (time_step <= end_deceleration) {
    TimeUnit time_left = end_deceleration - time_step;
    result.SetPosition(
        goal.Position() -
        time_left * (goal.Velocity() + 0.5 * time_left * max_acceleration));
    result.SetVelocity(goal.Velocity() + time_left * max_acceleration);
  } else {
    result = goal;
  }

  if (flip) {
    result.vector = -result.vector;
  }

  return result;
}

}  // namespace reefscape
