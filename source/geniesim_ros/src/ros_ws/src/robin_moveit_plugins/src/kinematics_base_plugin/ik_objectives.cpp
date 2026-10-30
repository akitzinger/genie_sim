#include "robin_moveit_plugins/kinematics_base_plugin/ik_objectives.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robin_moveit_plugins
{
  namespace
  {
    constexpr int kArmDof = 7;
    constexpr double kEps = 1e-9;

    struct Arm
    {
      std::array<pinocchio::JointIndex, kArmDof> joints{};
      pinocchio::FrameIndex tip = 0;
    };

    struct DualArmModel
    {
      pinocchio::Model model;
      Arm left, right;

      DualArmModel()
      {
        const std::string urdf =
            ament_index_cpp::get_package_share_directory("genie_sim_robot_model") +
            "/urdf/genie_g2_crs_omnipicker.urdf";
        pinocchio::Model full;
        pinocchio::urdf::buildModel(urdf, full);

        // Lock every non-arm joint at neutral so crba/Jacobians only span the two arms.
        std::vector<pinocchio::JointIndex> to_lock;
        for (pinocchio::JointIndex j = 1; j < full.joints.size(); ++j)
        {
          const std::string &name = full.names[j];
          if (name.find("_arm_l_joint") == std::string::npos &&
              name.find("_arm_r_joint") == std::string::npos)
            to_lock.push_back(j);
        }
        pinocchio::buildReducedModel(full, to_lock, pinocchio::neutral(full), model);
        left = makeArm("arm_l");
        right = makeArm("arm_r");
      }

      Arm makeArm(const std::string &prefix) const
      {
        Arm arm;
        for (int i = 0; i < kArmDof; ++i)
        {
          // URDF joint names carry an "idxNN_" prefix, e.g. idx21_arm_l_joint1
          const std::string suffix = prefix + "_joint" + std::to_string(i + 1);
          bool found = false;
          for (pinocchio::JointIndex j = 1; j < model.joints.size() && !found; ++j)
          {
            const std::string &name = model.names[j];
            if (name.size() >= suffix.size() &&
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
            {
              arm.joints[i] = j;
              found = true;
            }
          }
          if (!found)
            throw std::runtime_error("Joint not found in URDF: " + suffix);
        }
        const std::string tip = prefix + "_end_link";
        if (!model.existFrame(tip))
          throw std::runtime_error("Frame not found in URDF: " + tip);
        arm.tip = model.getFrameId(tip);
        return arm;
      }
    };

    const DualArmModel &dualArmModel()
    {
      static const DualArmModel instance;
      return instance;
    }

    // Dynamic manipulability along world z only: w_d = ||J_z M^-1|| over the arm's own joints.
    // Requires the joint Jacobians, frame placements and data.M for the current configuration.
    double armDynManipulability(
        const pinocchio::Model &model, pinocchio::Data &data, const Arm &arm)
    {
      Eigen::Matrix<double, 6, Eigen::Dynamic> J_full(6, model.nv);
      J_full.setZero();
      pinocchio::getFrameJacobian(model, data, arm.tip, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

      Eigen::Matrix<double, 1, kArmDof> J;
      Eigen::Matrix<double, kArmDof, kArmDof> M;
      for (int i = 0; i < kArmDof; ++i)
      {
        const int vi = model.joints[arm.joints[i]].idx_v();
        J(i) = J_full(2, vi);
        for (int j = 0; j < kArmDof; ++j)
          M(i, j) = data.M(vi, model.joints[arm.joints[j]].idx_v());
      }

      // crba fills only the upper triangle of data.M
      const Eigen::Matrix<double, kArmDof, kArmDof> Msym = M.selfadjointView<Eigen::Upper>();

      return (J * Msym.inverse()).norm();
    }

    struct Workspace
    {
      pinocchio::Data data;
      Eigen::VectorXd q;
      explicit Workspace(const pinocchio::Model &m) : data(m), q(pinocchio::neutral(m)) {}
    };

    // Sets the arm joints, then computes joint Jacobians and frame placements.
    Workspace &updateKinematics(
        const DualArmModel &dm, const double *q_left, const double *q_right)
    {
      thread_local Workspace ws(dm.model);
      for (int i = 0; i < kArmDof; ++i)
      {
        ws.q[dm.model.joints[dm.left.joints[i]].idx_q()] = q_left[i];
        ws.q[dm.model.joints[dm.right.joints[i]].idx_q()] = q_right[i];
      }
      pinocchio::computeJointJacobians(dm.model, ws.data, ws.q);
      pinocchio::updateFramePlacements(dm.model, ws.data);
      return ws;
    }
  }  // namespace

  double evalDualArmKinManipulability(
      const double *q_left, const double *q_right)
  {

    const DualArmModel &dm = dualArmModel();
    const pinocchio::Model &model = dm.model;
    Workspace &ws = updateKinematics(dm, q_left, q_right);

    // Full-space Jacobians (zero columns for the other arm), so J_l J_r^T cross terms are included.
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_l = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_r = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, ws.data, dm.left.tip, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
    pinocchio::getFrameJacobian(model, ws.data, dm.right.tip, pinocchio::LOCAL_WORLD_ALIGNED, J_r);

    // W = [I I]/2 applied to J = [J_l; J_r]: A = 1/4 (J_l + J_r)(J_l + J_r)^T
    const Eigen::Matrix<double, 6, Eigen::Dynamic> J_w = 0.5 * (J_l + J_r);
    const Eigen::Matrix<double, 6, 6> A = J_w * J_w.transpose();

    return -std::log(std::max(A.determinant(), 0.0) + kEps);
  }

  double evalDualArmDynManipulability(
      const double *q_left, const double *q_right)
  {

    const DualArmModel &dm = dualArmModel();
    const pinocchio::Model &model = dm.model;
    Workspace &ws = updateKinematics(dm, q_left, q_right);
    pinocchio::Data &data = ws.data;
    pinocchio::crba(model, data, ws.q);

    const double w_left = armDynManipulability(model, data, dm.left);
    const double w_right = armDynManipulability(model, data, dm.right);

    // Log-barrier on w^2, same form as the single-arm cost in kinematics_base_plugin.cpp
    return -std::log(w_left * w_left + kEps) - std::log(w_right * w_right + kEps);
  }

  double evalDualArmDynManipulabilityExtended(
      const double *q_left, const double *q_right)
  {

    using Mat67 = Eigen::Matrix<double, 6, kArmDof>;
    using Mat77 = Eigen::Matrix<double, kArmDof, kArmDof>;
    using Vec7 = Eigen::Matrix<double, kArmDof, 1>;

    const DualArmModel &dm = dualArmModel();
    const pinocchio::Model &model = dm.model;
    Workspace &ws = updateKinematics(dm, q_left, q_right);
    pinocchio::Data &data = ws.data;

    // Jacobians must be read before crba/gravity overwrite the kinematics data.
    std::array<Mat67, 2> J;
    const std::array<const Arm *, 2> arms = {&dm.left, &dm.right};
    for (int a = 0; a < 2; ++a)
    {
      Eigen::Matrix<double, 6, Eigen::Dynamic> J_full = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
      pinocchio::getFrameJacobian(model, data, arms[a]->tip, pinocchio::LOCAL_WORLD_ALIGNED, J_full);
      for (int i = 0; i < kArmDof; ++i)
        J[a].col(i) = J_full.col(model.joints[arms[a]->joints[i]].idx_v());
    }

    pinocchio::crba(model, data, ws.q);
    const Eigen::VectorXd g_full = pinocchio::computeGeneralizedGravity(model, data, ws.q);

    // Ellipsoid a^T P a + 2 a^T b + c = 1 with P = Q^-1, Q = sum_i J_i (Mz_i^T Mz_i)^-1 J_i^T,
    // b = sum_i J_i^{+T} Mz_i^T gz_i, c = sum_i gz_i^T gz_i, where Mz = Z M, gz = Z g (Z = diag(1/tau_max)).
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    double c = 0.0;
    for (int a = 0; a < 2; ++a)
    {
      Mat77 M;
      Vec7 g, z;
      for (int i = 0; i < kArmDof; ++i)
      {
        const int vi = model.joints[arms[a]->joints[i]].idx_v();
        g(i) = g_full(vi);
        const double tau_max = model.effortLimit(vi);
        z(i) = (std::isfinite(tau_max) && tau_max > 0.0) ? 1.0 / tau_max : 1.0;
        for (int j = 0; j < kArmDof; ++j)
          M(i, j) = data.M(vi, model.joints[arms[a]->joints[j]].idx_v());
      }
      // crba fills only the upper triangle of data.M
      const Mat77 Msym = M.selfadjointView<Eigen::Upper>();
      const Mat77 Mz = z.asDiagonal() * Msym;
      const Vec7 gz = z.asDiagonal() * g;

      const Mat77 Mz_inv = Mz.inverse();
      Q += J[a] * Mz_inv * Mz_inv.transpose() * J[a].transpose();

      const Eigen::Matrix<double, 6, 6> JJt_inv = (J[a] * J[a].transpose()).inverse();
      b += JJt_inv * J[a] * Mz.transpose() * gz;  // J^{+T} = (J J^T)^-1 J
      c += gz.squaredNorm();
    }

    // Restrict a to the world-z direction: a = a_z e_z  ->  P_zz a_z^2 + 2 b_z a_z + (c - 1) = 0
    const double P_zz = Q.inverse()(2, 2);
    const double b_z = b(2);
    const double disc = b_z * b_z + P_zz * (1.0 - c);

    // disc <= 0: gravity alone exceeds the torque limits, no feasible z acceleration
    const double extent = disc > 0.0 ? 2.0 * std::sqrt(disc) / P_zz : 0.0;
    return -std::log(extent + kEps);
  }

  double evalDualArmForceManipulability(
      const double *q_left, const double *q_right)
  {

    const DualArmModel &dm = dualArmModel();
    const pinocchio::Model &model = dm.model;
    Workspace &ws = updateKinematics(dm, q_left, q_right);

    // Zeroed once; each arm's frame Jacobian only writes its own columns.
    thread_local Eigen::Matrix<double, 6, Eigen::Dynamic> J_l = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
    thread_local Eigen::Matrix<double, 6, Eigen::Dynamic> J_r = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, ws.data, dm.left.tip, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
    pinocchio::getFrameJacobian(model, ws.data, dm.right.tip, pinocchio::LOCAL_WORLD_ALIGNED, J_r);

    // Force ellipsoid h^T J J^T h = 1 with h = f e_z: f_max = 1 / ||J_z||, summed over both hands.
    const double f_l = 1.0 / std::sqrt(J_l.row(2).squaredNorm() + kEps);
    const double f_r = 1.0 / std::sqrt(J_r.row(2).squaredNorm() + kEps);

    // Saturating cost in [0,1): -log(f) is unbounded as J_z -> 0 and drags the search into singularities.
    constexpr double kForceRef = 2.0;  // total lift capacity at which the cost reaches ~0.24
    return 1.0 - std::tanh((f_l + f_r) / kForceRef);
  }
}  // namespace robin_moveit_plugins
