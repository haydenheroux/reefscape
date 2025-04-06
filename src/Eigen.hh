#pragma once

#include "units.hh"
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>

namespace sim {
template <int States> using StateVector = Eigen::Vector<double, States>;

template <int States>
using SystemMatrix = Eigen::Matrix<double, States, States>;

template <int Inputs> using InputVector = Eigen::Vector<double, Inputs>;

template <int States, int Inputs>
using InputMatrix = Eigen::Matrix<double, States, Inputs>;

// NOTE(hayden): The Moore-Penrose left pseudoinverse of an input matrix expects
// a "tall" rather than a "wide" shape
template <int States, int Inputs>
  requires(States > Inputs)
using InputLeftPseudoInverseMatrix = Eigen::Matrix<double, Inputs, States>;

template <int States, int Inputs>
std::pair<SystemMatrix<States>, InputMatrix<States, Inputs>> Discretize(
    const std::pair<SystemMatrix<States>, InputMatrix<States, Inputs>> &AcBc,
    units::TimeUnit sample_period) {
  using BlockMatrix = Eigen::Matrix<double, States + Inputs, States + Inputs>;

  // M = ⎡ Ac Bc ⎤
  //     ⎣ 0  0  ⎦
  BlockMatrix M;
  M.template block<States, States>(0, 0) = AcBc.first;
  M.template block<States, Inputs>(0, States) = AcBc.second;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  // ϕ = ⎡ Ad Bd ⎤
  //     ⎣ 0  I  ⎦
  BlockMatrix phi = (M * sample_period.in(units::seconds)).exp();
  SystemMatrix<States> Ad = phi.template block<States, States>(0, 0);
  InputMatrix<States, Inputs> Bd =
      phi.template block<States, Inputs>(0, States);
  return std::make_pair(Ad, Bd);
}

template <int States, int Inputs>
InputLeftPseudoInverseMatrix<States, Inputs>
PseudoInverse(const InputMatrix<States, Inputs> &Bc) {
  // Bc⁺ = (BcᵀBc)⁻¹Bcᵀ
  auto BcT = Bc.transpose();
  return (BcT * Bc).inverse() * BcT;
}
}; // namespace sim
