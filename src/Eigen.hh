#pragma once

#include "units.hh"
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>

namespace sim {
using namespace units;
template <int NumStates>
using SystemMatrix = Eigen::Matrix<double, NumStates, NumStates>;

template <int NumStates, int NumInputs>
using InputMatrix = Eigen::Matrix<double, NumStates, NumInputs>;

template <int NumStates, int NumInputs>
std::pair<SystemMatrix<NumStates>, InputMatrix<NumStates, NumInputs>>
Discretize(
    const std::pair<SystemMatrix<NumStates>, InputMatrix<NumStates, NumInputs>>
        &continuous_matrices,
    TimeUnit sample_period) {
  Eigen::Matrix<double, NumStates + NumInputs, NumStates + NumInputs> M;
  M.template block<NumStates, NumStates>(0, 0) = continuous_matrices.first;
  M.template block<NumStates, NumInputs>(0, NumStates) =
      continuous_matrices.second;
  M.template block<NumInputs, NumStates + NumInputs>(NumStates, 0).setZero();

  Eigen::Matrix<double, NumStates + NumInputs, NumStates + NumInputs> phi =
      (M * sample_period.in(seconds)).exp();

  Eigen::Matrix<double, NumStates, NumStates> Ad =
      phi.template block<NumStates, NumStates>(0, 0);
  Eigen::Matrix<double, NumStates, NumInputs> Bd =
      phi.template block<NumStates, NumInputs>(0, NumStates);
  return std::make_pair(Ad, Bd);
}

template <int NumStates, int NumInputs>
Eigen::Matrix<double, NumInputs, NumStates>
PseudoInverse(const InputMatrix<NumStates, NumInputs> &continuous_input) {
  static_assert(NumStates > NumInputs,
                "left pseudoinverse expects a tall matrix");
  auto continuous_input_transpose = continuous_input.transpose();
  return (continuous_input_transpose * continuous_input).inverse() *
         continuous_input_transpose;
}
}; // namespace sim
