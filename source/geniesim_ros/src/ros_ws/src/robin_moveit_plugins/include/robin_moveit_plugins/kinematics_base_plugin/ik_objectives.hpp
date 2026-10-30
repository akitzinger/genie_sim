#pragma once

#include <vector>

namespace robin_moveit_plugins
{
  // Joint inputs are 7 values per arm, ordered joint1..joint7. All functions return a cost (lower is better).

  // Jacobian-only: -log det(1/4 (J_l + J_r)(J_l + J_r)^T).
  double evalDualArmKinManipulability(const double *q_left, const double *q_right);

  // Dynamic manipulability (z direction) of both arms.
  double evalDualArmDynManipulability(const double *q_left, const double *q_right);

  // Torque-normalized, gravity-aware z-acceleration ellipsoid of both arms; cost = -log(extent).
  double evalDualArmDynManipulabilityExtended(const double *q_left, const double *q_right);

  // Static z-lift force capacity of both hands from the force ellipsoid; cost = 1 - tanh((f_l + f_r) / ref).
  double evalDualArmForceManipulability(const double *q_left, const double *q_right);

  inline double evalDualArmForceManipulability(
      const std::vector<double> &q_left, const std::vector<double> &q_right)
  {
    return (q_left.size() == 7 && q_right.size() == 7)
               ? evalDualArmForceManipulability(q_left.data(), q_right.data())
               : 0.0;
  }
}  // namespace robin_moveit_plugins
