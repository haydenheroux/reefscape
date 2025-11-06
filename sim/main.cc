#include <chrono>
#include <thread>

#include "AffineSystemSim.hh"
#include "Elevator.hh"
#include "Motor.hh"
#include "MotorSystem.hh"
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
  // TODO(hayden): Move quantity makers to separate namespace?
  Elevator elevator{units::gear_ratio(5), 0.5 * au::inches(1.273),
                    au::pounds_mass(30),  au::amperes(120),
                    kTotalTravel,         Motor::KrakenX60FOC() * 2};

  auto server = nt::CreateInstance();
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);
  Publisher publisher{server};

  Time time_step = (au::milli(au::seconds))(1);
  auto wait_time =
      std::chrono::microseconds(time_step.in<int>(au::micro(au::seconds)));
  // TODO(hayden): Make this a universal constant
  LinearAcceleration gravity = (au::meters / squared(au::second))(-9.81);

  AffineSystemSim<State, Input> sim{elevator, gravity, time_step};

  // TODO(hayden): Implement LQR to find the optimal K
  auto kP = (au::volts / au::meter)(191.2215);
  auto kD = (au::volts / (au::meters / au::second))(4.811);
  Eigen::Matrix<double, Input::Dimension, State::Dimension> K;
  K << kP.in(au::volts / au::meter),
      kD.in(au::volts / (au::meters / au::second));

  // TODO(hayden): Determine if it is possible to avoid explicit declaration
  TrapezoidTrajectory<units::DisplacementUnit> profile{elevator};

  // TODO(hayden): State constructors with only position, no velocity
  State top{kTotalTravel, (au::meters / au::second)(0)};
  // TODO(hayden): Zero state constructor
  State bottom{au::meters(0), (au::meters / au::second)(0)};

  State reference = bottom;
  State goal = top;

  Time total_sim_time = au::seconds(0);

  while (true) {
    // TODO(hayden): Determine goal based on events
    auto cycle_time = au::fmod(total_sim_time, au::seconds(6));
    if (cycle_time < au::seconds(3)) {
      goal = top;
    } else {
      goal = bottom;
    }

    reference = profile.Calculate(time_step, reference, goal);
    State error{reference.vector - sim.State().vector};

    Input input{K * error.vector + sim.StabilizingInput().vector};
    auto limited_voltage =
        LimitVoltage(elevator, sim.State().Velocity(), input.Voltage());
    Input limited_input{limited_voltage};
    sim.Update(limited_input);
    sim.SetState(
        sim.State().PositionClamped(au::meters(0), elevator.max_travel));

    State new_state = sim.State();
    bool at_goal = new_state.At(goal);
    publisher.Publish(sim.State(), reference, sim.Input(), at_goal);

    total_sim_time += time_step;
    std::this_thread::sleep_for(wait_time);
  }
}
