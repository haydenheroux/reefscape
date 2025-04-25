#include <chrono>
#include <cmath>
#include <thread>

#include "Elevator.hh"
#include "Motor.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "ntcore_cpp.h"
#include "publisher.hh"
#include "robot.hh"
#include "trajectory.hh"
#include "units.hh"

using namespace reefscape;
using State = AffineSystemSim::State;
using Input = AffineSystemSim::Input;

int main() {
  Elevator elevator{gear_ratio(5),   0.5 * inches(1.273),
                    pounds_mass(30), amperes(120),
                    kTotalTravel,    Motor::KrakenX60FOC() * 2};

  auto server = nt::CreateInstance();
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);
  Publisher publisher{server};

  TimeUnit time_step = (milli(seconds))(1);
  auto wait_time = std::chrono::microseconds(time_step.in<int>(micro(seconds)));
  AccelerationUnit gravity = (meters / squared(second))(-9.81);

  AffineSystemSim sim{elevator, gravity, time_step};

  // TODO(hayden): Implement LQR to find the optimal K
  auto kP = (volts / meter)(191.2215);
  auto kD = (volts / (meters / second))(4.811);
  Eigen::Matrix<double, Input::Dimension, State::Dimension> K;
  K << kP.in(volts / meter), kD.in(volts / (meters / second));

  // TODO Implement `TrapezoidTrajectory` construction from `Elevator`
  TrapezoidTrajectory profile;
  profile.max_acceleration = elevator.MaximumAcceleration();
  profile.max_velocity = (meters / second)(1.92);

  State reference = sim.state;
  State goal{kTotalTravel, (meters / second)(0)};

  TimeUnit total_sim_time = seconds(0);

  while (true) {
    // TODO(hayden): Make this event-based
    TimeUnit cycle_time = au::fmod(total_sim_time, seconds(6));
    if (cycle_time < seconds(3)) {
      goal = State{kTotalTravel, (meters / second)(0)};
    } else {
      goal = State{meters(0), (meters / second)(0)};
    }

    reference = profile.Calculate(time_step, reference, goal);

    State error = reference - sim.state;
    sim.input = K * error.vector;
    sim.input += sim.StabilizingInput();
    sim.input =
        elevator.LimitVoltage(sim.state.Velocity(), sim.input.Voltage());
    sim.Update();

    publisher.Publish(sim.state, reference, sim.input, sim.state.At(goal));

    total_sim_time += time_step;
    std::this_thread::sleep_for(wait_time);
  }
}
