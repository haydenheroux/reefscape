#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/minutes.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "common.hh"
#include "ntcore_cpp.h"
#include "units.hh"
#include <chrono>
#include <cmath>
#include <thread>

int main() {
  Motor krakenX60{volts(12), newton_meters(7.09), amperes(366),
                  (revolutions / minute)(6000), amperes(2)};

  Elevator elevator{gear_ratio(5), 0.5 * inches(1.273), pounds_mass(30),
                    amperes(120),  kTotalTravel,        krakenX60 * 2};

  auto server = nt::CreateInstance();

  auto publisher =
      nt::Publish(nt::GetTopic(server, "/position"), NT_DOUBLE, "double");
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);

  TimeUnit sim_time_step = (milli(seconds))(1);
  ElevatorSim sim{elevator, (meters / squared(second))(-9.81), sim_time_step};

  TimeUnit sim_time = seconds(0);

  auto kP = (volts / meter)(48.0);

  while (true) {
    DisplacementUnit setpoint;

    TimeUnit time = au::fmod(sim_time, seconds(6));
    if (time <= seconds(2)) {
      setpoint = meters(0);
    } else if (time >= seconds(4)) {
      setpoint = meters(1.25);
    } else {
      setpoint = kTotalTravel;
    }

    DisplacementUnit error = setpoint - sim.Position();
    VoltageUnit voltage = error * kP;
    voltage += sim.OpposingGravity();
    voltage = sim.CurrentLimit(voltage);
    sim.Update(voltage);
    nt::SetDouble(publisher, sim.Position().in(meters));

    sim_time += sim_time_step;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(sim_time_step.in<int>(milli(seconds))));
  }
}
