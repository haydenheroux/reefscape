#include "AffineSystemSim.hh"

#include <limits>

#include "Eigen.hh"

namespace reefscape {

AffineSystemSim::AffineSystemSim(
    SystemMatrix<State::Dimension> continuous_system,
    InputMatrix<State::Dimension, Input::Dimension> continuous_input,
    Eigen::Vector<double, State::Dimension> continuous_constant,
    TimeUnit time_step)
    : time_step_(time_step),
      continuous_system_(continuous_system),
      continuous_input_(continuous_input),
      continuous_constant_(continuous_constant),
      state(meters(0), (meters / second)(0)),
      input(volts(0)),
      min_position_(meters(-std::numeric_limits<double>::infinity())),
      max_position_(meters(std::numeric_limits<double>::infinity())) {
  auto continuous_matrices =
      std::make_pair(continuous_system_, continuous_input_);
  auto discretized_matrices = Discretize(continuous_matrices, time_step_);
  discrete_system_ = discretized_matrices.first;
  discrete_input_ = discretized_matrices.second;
  continuous_input_pseudoinverse_ = PseudoInverse(continuous_input_);
  discrete_constant_ << discrete_input_ * continuous_input_pseudoinverse_ *
                            continuous_constant_;
}

AffineSystemSim::AffineSystemSim(const Elevator &elevator,
                                 AccelerationUnit gravity, TimeUnit time_step)
    : AffineSystemSim(elevator.ContinuousSystemMatrix(),
                      elevator.ContinuousInputMatrix(),
                      {0, gravity.in(meters / squared(second))}, time_step) {}

AffineSystemSim::Input AffineSystemSim::StabilizingInput() const {
  return {-1 * continuous_input_pseudoinverse_ * continuous_constant_};
}

void AffineSystemSim::Update() {
  state = discrete_system_ * state.vector + discrete_input_ * input.vector +
          discrete_constant_;
  // Clamp position to prevent simulator from moving out of bounds
  state = state.ClampPosition(min_position_, max_position_);
}

};  // namespace reefscape
