#pragma once

#include "AffineSystemSim.hh"
#include "ntcore_c.h"

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

  void Publish(AffineSystemSim::State state, AffineSystemSim::State reference,
               AffineSystemSim::Input input, bool at_goal) const;
};

};  // namespace reefscape
