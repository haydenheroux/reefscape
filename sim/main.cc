#include <chrono>
#include <thread>

#include "AffineSystemSim.hh"
#include "Elevator.hh"
#include "Motor.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "ntcore_cpp.h"
#include "pubsub.hh"
#include "robot.hh"
#include "trajectory.hh"
#include "units.hh"

using namespace reefscape;
using State = PositionVelocityState;
using Input = VoltageInput;

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

  AffineSystemSim<State, Input> sim{elevator, gravity, time_step};

  // TODO(hayden): Implement LQR to find the optimal K
  auto kP = (volts / meter)(191.2215);
  auto kD = (volts / (meters / second))(4.811);
  Eigen::Matrix<double, Input::Dimension, State::Dimension> K;
  K << kP.in(volts / meter), kD.in(volts / (meters / second));

  TrapezoidTrajectory profile{elevator};

  State top{kTotalTravel, (meters / second)(0)};
  State bottom{meters(0), (meters / second)(0)};

  State reference = bottom;
  State goal = top;

  TimeUnit total_sim_time = seconds(0);

  while (true) {
    // TODO(hayden): Make this event-based
    TimeUnit cycle_time = au::fmod(total_sim_time, seconds(6));
    if (cycle_time < seconds(3)) {
      goal = top;
    } else {
      goal = bottom;
    }

    reference = profile.Calculate(time_step, reference, goal);
    State error{reference.vector - sim.State().vector};

    Input input{K * error.vector + sim.StabilizingInput().vector};
    VoltageUnit limited_voltage =
        elevator.LimitVoltage(sim.State().Velocity(), input.Voltage());
    Input limited_input{limited_voltage};
    sim.Update(limited_input);
    sim.SetState(sim.State().PositionClamped(meters(0), elevator.max_travel));

    State new_state = sim.State();
    bool at_goal = new_state.At(goal);
    publisher.Publish(sim.State(), reference, sim.Input(), at_goal);

    total_sim_time += time_step;
    std::this_thread::sleep_for(wait_time);
  }
}
