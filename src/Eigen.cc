#include "Eigen.hh"
#include <unsupported/Eigen/MatrixFunctions>

std::pair<SystemMatrix, InputMatrix>
discretize(std::pair<SystemMatrix, InputMatrix> continuous_matrices,
           TimeUnit sample_period) {
  Eigen::Matrix<double, 4, 4> M;
  M.template block<2, 2>(0, 0) = continuous_matrices.first;
  M.template block<2, 2>(0, 2) = continuous_matrices.second;
  M.template block<2, 4>(2, 0).setZero();

  Eigen::Matrix<double, 4, 4> phi = (M * sample_period.in(seconds)).exp();

  Eigen::Matrix<double, 2, 2> Ad = phi.template block<2, 2>(0, 0);
  Eigen::Matrix<double, 2, 2> Bd = phi.template block<2, 2>(0, 2);
  return std::make_pair(Ad, Bd);
}
