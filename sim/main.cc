#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>

#include "Eigen.hh"
#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/minutes.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "ntcore_cpp.h"
#include "robot.hh"
#include "units.hh"

using namespace units;
using namespace robot;
using State = sim::ElevatorSim::State;

State calculate(TimeUnit dt, State state, State goal, VelocityUnit max_velocity,
                AccelerationUnit max_acceleration) {
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

  State result{state};

  if (dt < end_acceleration) {
    result.SetPosition(result.Position() +
                       (state.Velocity() + 0.5 * dt * max_acceleration) * dt);
    result.SetVelocity(result.Velocity() + dt * max_acceleration);
  } else if (dt <= end_cruise) {
    result.SetPosition(
        state.Position() +
        (state.Velocity() + 0.5 * end_acceleration * max_acceleration) *
            end_acceleration +
        max_velocity * (dt - end_acceleration));
    result.SetVelocity(max_velocity);
  } else if (dt <= end_deceleration) {
    TimeUnit time_left = end_deceleration - dt;
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

int main() {
  sim::Motor krakenX60{volts(12), newton_meters(7.09), amperes(366),
                       (revolutions / minute)(6000), amperes(2)};
  sim::Motor krakenX60FOC{volts(12), newton_meters(9.37), amperes(483),
                          (revolutions / minute)(5800), amperes(2)};

  sim::Motor motor = krakenX60FOC;

  sim::Elevator elevator{gear_ratio(5), 0.5 * inches(1.273), pounds_mass(30),
                         amperes(120),  kTotalTravel,        motor * 2};

  auto server = nt::CreateInstance();
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);

  auto position_publisher = nt::Publish(
      nt::GetTopic(server, "/elevator_position"), NT_DOUBLE, "double");
  auto velocity_publisher = nt::Publish(
      nt::GetTopic(server, "/elevator_velocity"), NT_DOUBLE, "double");

  TimeUnit sim_time_step = (milli(seconds))(2.5);
  sim::ElevatorSim sim{elevator, (meters / squared(second))(-9.81),
                       sim_time_step};

  TimeUnit sim_time = seconds(0);

  // TODO(hayden): Implement LQR for feedback control
  auto kP = (volts / meter)(191.2215);
  auto kD = (volts / (meters / second))(4.811);

  static const int kStates = sim::ElevatorSim::State::Dimension;
  static const int kInputs = sim::ElevatorSim::Input::Dimension;
  Eigen::Matrix<double, kInputs, kStates> K;
  K << kP.in(volts / meter), kD.in(volts / (meters / second));

  sim::ElevatorSim::State reference = sim.state;
  VelocityUnit max_velocity = (meters / second)(4.0);
  AccelerationUnit max_acceleration = elevator.MaximumAcceleration();
  sim::ElevatorSim::State goal(kTotalTravel, (meters / second)(0));

  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::ofstream csv{"result.csv"};
  csv << "time (s),reference position (m),reference velocity (m/s),position "
         "(m),velocity (m/s),voltage (V)\n";

  while (true) {
    TimeUnit time = au::fmod(sim_time, seconds(6));
    if (time < seconds(3)) {
      goal = sim::ElevatorSim::State{kTotalTravel, (meters / second)(0)};
    } else {
      goal = sim::ElevatorSim::State{meters(0), (meters / second)(0)};
    }

    reference = calculate(sim_time_step, reference, goal, max_velocity,
                          max_acceleration);
    sim::ElevatorSim::State error = reference - sim.state;
    sim::ElevatorSim::Input input{K * error.vector};
    input += sim.OpposingGravity();
    sim.input = sim.CurrentLimited(input);
    sim.Update();
    nt::SetDouble(position_publisher, sim.state.Position().in(meters));
    nt::SetDouble(velocity_publisher, sim.state.Velocity().in(meters / second));
    nt::Flush(server);

    csv << sim_time.in(seconds) << "," << reference.Position().in(meters) << ","
        << reference.Velocity().in(meters / second) << ","
        << sim.state.Position().in(meters) << ","
        << sim.state.Velocity().in(meters / second) << ","
        << sim.input.Voltage().in(volts) << std::endl;

    sim_time += sim_time_step;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(sim_time_step.in<int>(milli(seconds))));
  }
}
