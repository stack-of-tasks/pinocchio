//
// Copyright (c) 2026 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace internal
  {
    ///
    /// \brief Projector mapping a spatial velocity of joint 1 to the time derivative of the
    /// position of the second point expressed in the frame of the first one.
    ///
    /// This is the very same matrix as PointConstraintModelBase::getA1. It is factored out here
    /// so that the scalar constraints built on top of a pair of points -- constant length, point
    /// on an ellipsoid, ... -- all share a single implementation: such a constraint whose
    /// residual reads phi(q) = f(x(q)), with x the relative position of the two points, has
    /// A1 = grad f(x)^T * A1_point.
    ///
    /// \param[in] oMc1 Placement of the first point frame with respect to the WORLD frame.
    /// \param[in] oMc2 Placement of the second point frame with respect to the WORLD frame.
    /// \param[in] joint1_placement Placement of the first point with respect to joint 1.
    /// \param[in] relative_position Position of the second point in the frame of the first one.
    ///
    template<typename Scalar, int Options, ReferenceFrame rf>
    Eigen::Matrix<Scalar, 3, 6, Options> relativePointProjector1(
      const SE3Tpl<Scalar, Options> & oMc1,
      const SE3Tpl<Scalar, Options> & oMc2,
      const SE3Tpl<Scalar, Options> & joint1_placement,
      const Eigen::Matrix<Scalar, 3, 1, Options> & relative_position,
      ReferenceFrameTag<rf>)
    {
      typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;
      typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

      Eigen::Matrix<Scalar, 3, 6, Options> res;

      if constexpr (std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value)
      {
        const Matrix3 c1Ro = oMc1.rotation().transpose();
        res.template leftCols<3>() = -c1Ro;
        res.template rightCols<3>().noalias() = c1Ro * skew(oMc2.translation());
      }
      else if constexpr (std::is_same<ReferenceFrameTag<rf>, LocalFrameTag>::value)
      {
        const Matrix3 c1Rj1 = joint1_placement.rotation().transpose();
        // Position of the second point expressed in the frame of joint 1.
        const Vector3 j1_p_c2 = joint1_placement.act(relative_position);
        res.template leftCols<3>() = -c1Rj1;
        res.template rightCols<3>().noalias() = c1Rj1 * skew(j1_p_c2);
      }
      else
      {
        PINOCCHIO_UNUSED_VARIABLE(oMc1);
        PINOCCHIO_UNUSED_VARIABLE(oMc2);
        PINOCCHIO_UNUSED_VARIABLE(joint1_placement);
        PINOCCHIO_UNUSED_VARIABLE(relative_position);
        PINOCCHIO_UNREACHABLE();
      }

      return res;
    }

    ///
    /// \brief Projector mapping a spatial velocity of joint 2 to the time derivative of the
    /// position of the second point expressed in the frame of the first one.
    ///
    /// \copydetails relativePointProjector1
    ///
    /// \param[in] c1Mc2 Relative placement of the second point frame w.r.t. the first one.
    /// \param[in] joint2_placement Placement of the second point with respect to joint 2.
    ///
    template<typename Scalar, int Options, ReferenceFrame rf>
    Eigen::Matrix<Scalar, 3, 6, Options> relativePointProjector2(
      const SE3Tpl<Scalar, Options> & oMc1,
      const SE3Tpl<Scalar, Options> & oMc2,
      const SE3Tpl<Scalar, Options> & c1Mc2,
      const SE3Tpl<Scalar, Options> & joint2_placement,
      ReferenceFrameTag<rf>)
    {
      typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;

      Eigen::Matrix<Scalar, 3, 6, Options> res;

      if constexpr (std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value)
      {
        const Matrix3 c1Ro = oMc1.rotation().transpose();
        res.template leftCols<3>() = c1Ro;
        res.template rightCols<3>().noalias() = -c1Ro * skew(oMc2.translation());
      }
      else if constexpr (std::is_same<ReferenceFrameTag<rf>, LocalFrameTag>::value)
      {
        const Matrix3 c1Rj2 = c1Mc2.rotation() * joint2_placement.rotation().transpose();
        res.template leftCols<3>() = c1Rj2;
        res.template rightCols<3>().noalias() = -c1Rj2 * skew(joint2_placement.translation());
      }
      else
      {
        PINOCCHIO_UNUSED_VARIABLE(oMc1);
        PINOCCHIO_UNUSED_VARIABLE(oMc2);
        PINOCCHIO_UNUSED_VARIABLE(c1Mc2);
        PINOCCHIO_UNUSED_VARIABLE(joint2_placement);
        PINOCCHIO_UNREACHABLE();
      }

      return res;
    }

  } // namespace internal
} // namespace pinocchio
