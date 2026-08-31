//
// Copyright (c) 2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/autodiff/casadi.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/autodiff/casadi.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace internal
  {
    template<typename _Scalar, int Options>
    struct SplineKinematics<::casadi::Matrix<_Scalar>, Options>
    {
      using Scalar = ::casadi::Matrix<_Scalar>;
      typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> BasisVectorType;

      static Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
      allocateBasis(int degree, int knot_size)
      {
        return Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>(
          degree + 1, knot_size - degree - 1);
      }

      static void compute(
        int degree,
        const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
        const std::vector<SE3Tpl<Scalar, Options>> & ctrlFrames,
        const std::vector<MotionTpl<Scalar, Options>> & relativeMotions,
        Scalar q,
        const Eigen::Matrix<Scalar, 1, 1, Options> & joint_v,
        bool computeVelocity,
        SE3Tpl<Scalar, Options> & M,
        MotionTpl<Scalar, Options> & v,
        MotionTpl<Scalar, Options> & c,
        JointMotionSubspaceTpl<1, Scalar, Options, 1> & S,
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis)
      {
        return computeSplineKinematicsFull(
          degree, knots, ctrlFrames, relativeMotions, q, joint_v, computeVelocity, M, v, c, S,
          basis);
      }
    };
  } // namespace internal
} // namespace pinocchio
