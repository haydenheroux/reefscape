#pragma once

#include "units.hh"
#include <Eigen/Core>
#include <utility>

using SystemMatrix = Eigen::Matrix<double, 2, 2>;
using InputMatrix = Eigen::Matrix<double, 2, 1>;

std::pair<SystemMatrix, InputMatrix>
discretize(std::pair<SystemMatrix, InputMatrix> continuous_matrices,
           TimeUnit sample_period);
