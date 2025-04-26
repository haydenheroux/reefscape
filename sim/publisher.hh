#pragma once

#include "input.hh"
#include "ntcore_c.h"
#include "state.hh"

namespace reefscape {

struct Publisher {
  NT_Inst instance;
  NT_Publisher position;
  NT_Publisher velocity;
  NT_Publisher reference_position;
  NT_Publisher reference_velocity;
  NT_Publisher voltage;
  NT_Publisher at_goal;

  Publisher(NT_Inst instance);

  void Publish(PositionVelocityState state, PositionVelocityState reference,
               VoltageInput input, bool at_goal) const;
};

};  // namespace reefscape
