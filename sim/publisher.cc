#include "publisher.hh"

#include "ntcore_cpp.h"
#include "robot.hh"

namespace reefscape {

Publisher::Publisher(NT_Inst instance) {
  this->instance = instance;

  position = nt::Publish(nt::GetTopic(instance, kElevatorPositionKey),
                         NT_DOUBLE, "double");
  velocity = nt::Publish(nt::GetTopic(instance, kElevatorVelocityKey),
                         NT_DOUBLE, "double");
  reference_position =
      nt::Publish(nt::GetTopic(instance, kElevatorReferencePositionKey),
                  NT_DOUBLE, "double");
  reference_velocity =
      nt::Publish(nt::GetTopic(instance, kElevatorReferenceVelocityKey),
                  NT_DOUBLE, "double");
  voltage = nt::Publish(nt::GetTopic(instance, kElevatorVoltageKey), NT_DOUBLE,
                        "double");
  at_goal = nt::Publish(nt::GetTopic(instance, kElevatorAtGoalKey), NT_BOOLEAN,
                        "boolean");
}

void Publisher::Publish(AffineSystemSim::State state,
                        AffineSystemSim::State reference,
                        AffineSystemSim::Input input, bool at_goal) const {
  nt::SetDouble(position, state.Position().in(meters));
  nt::SetDouble(velocity, state.Velocity().in(meters / second));
  nt::SetDouble(reference_position, reference.Position().in(meters));
  nt::SetDouble(reference_velocity, reference.Velocity().in(meters / second));
  nt::SetDouble(voltage, input.Voltage().in(volts));
  nt::SetBoolean(this->at_goal, at_goal);
  nt::Flush(instance);
}

};  // namespace reefscape
