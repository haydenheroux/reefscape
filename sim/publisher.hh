#pragma once

#include "Elevator.hh"
#include "ntcore_c.h"

namespace reefscape {

struct Publisher {
  NT_Inst instance;
  NT_Publisher position;
  NT_Publisher velocity;
  NT_Publisher reference_position;
  NT_Publisher reference_velocity;
  NT_Publisher voltage;

  Publisher(NT_Inst instance);

  void Publish(ElevatorSim::State state, ElevatorSim::State reference,
               ElevatorSim::Input input) const;
};

};  // namespace reefscape
