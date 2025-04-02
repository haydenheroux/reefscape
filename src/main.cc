#include "Elevator.hh"
#include "au/io.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "units.hh"
#include <iostream>

int main() {
  Motor krakenX60{volts(12), newton_meters(7.09), amperes(366), rpm(6000),
                  amperes(2)};

  ElevatorConstants constants{gear_ratio(5), 0.5 * inches(1.273),
                              pounds_mass(30), amperes(40)};

  Elevator elevator{constants, krakenX60 * 2};

  TimeUnit time_step = (milli(seconds))(5);

  ElevatorSim sim{elevator, time_step};

  for (TimeUnit time = seconds(0); time < seconds(50); time += time_step) {
    std::cout << time << " => " << sim.state_.velocity() << "\n";
    VoltageUnit voltage =
        elevator.limited_voltage(sim.state_.velocity(), volts(12));
    sim.update(voltage);
  }
}
