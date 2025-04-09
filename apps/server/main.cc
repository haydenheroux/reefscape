#include <chrono>
#include <cmath>
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

int main() {
  sim::Motor krakenX60{volts(12), newton_meters(7.09), amperes(366),
                       (revolutions / minute)(6000), amperes(2)};

  sim::Elevator elevator{gear_ratio(5), 0.5 * inches(1.273), pounds_mass(30),
                         amperes(120),  kTotalTravel,        krakenX60 * 2};

  auto server = nt::CreateInstance();

  auto publisher =
      nt::Publish(nt::GetTopic(server, "/position"), NT_DOUBLE, "double");
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);

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

  sim::ElevatorSim::State reference(meters(0), (meters / second)(0));

  while (true) {
    TimeUnit time = au::fmod(sim_time, seconds(6));
    if (time <= seconds(2)) {
      reference.SetPosition(meters(0));
    } else if (time >= seconds(4)) {
      reference.SetPosition(meters(1.25));
    } else {
      reference.SetPosition(kTotalTravel);
    }

    sim::ElevatorSim::State error = reference - sim.state;
    sim::ElevatorSim::Input input{K * error.vector};
    input += sim.OpposingGravity();
    sim.input = sim.CurrentLimited(input);
    sim.Update();
    nt::SetDouble(publisher, sim.state.Position().in(meters));
    nt::Flush(server);

    sim_time += sim_time_step;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(sim_time_step.in<int>(milli(seconds))));
  }
}
