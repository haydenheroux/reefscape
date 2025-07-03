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

struct Subscriber {
  NT_Inst instance;
  NT_Subscriber position;
  NT_Subscriber velocity;
  NT_Subscriber reference_position;
  NT_Subscriber reference_velocity;
  NT_Subscriber voltage;
  NT_Subscriber at_goal;

  Subscriber(NT_Inst instance);

  DisplacementUnit Position() const;

  VelocityUnit Velocity() const;

  DisplacementUnit ReferencePosition() const;

  VelocityUnit ReferenceVelocity() const;

  VoltageUnit Voltage() const;

  bool AtGoal() const;
};

};  // namespace reefscape
