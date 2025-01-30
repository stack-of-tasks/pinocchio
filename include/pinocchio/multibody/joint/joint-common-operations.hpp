//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_multibody_joint_joint_common_operations_hpp__
#define __pinocchio_multibody_joint_joint_common_operations_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/math/matrix.hpp"
#include "pinocchio/math/fwd.hpp"

#include <boost/type_traits.hpp>
#include <boost/variant.hpp>

namespace pinocchio
{
  namespace internal
  {
    ///
    /// \brief Operation called in JointModelBase<JointModel>::calc_aba
    ///
    template<typename Scalar, bool is_floating_point = pinocchio::is_floating_point<Scalar>::value>
    struct PerformStYSInversion
    {
      template<typename M1, typename M2>
      static EIGEN_STRONG_INLINE void
      run(const Eigen::MatrixBase<M1> & StYS, const Eigen::MatrixBase<M2> & Dinv)
      {
        M2 & Dinv_ = PINOCCHIO_EIGEN_CONST_CAST(M2, Dinv);
        Dinv_.setIdentity();
        StYS.llt().solveInPlace(Dinv_);
      }
    };

    template<typename Scalar>
    struct PerformStYSInversion<Scalar, false>
    {
      template<typename M1, typename M2>
      static EIGEN_STRONG_INLINE void
      run(const Eigen::MatrixBase<M1> & StYS, const Eigen::MatrixBase<M2> & Dinv)
      {
        M2 & Dinv_ = PINOCCHIO_EIGEN_CONST_CAST(M2, Dinv);
        inverse(StYS, Dinv_);
      }
    };
  } // namespace internal

  ///
  /// \brief Linear affine transformation of the configuration vector.
  ///        Valide for most common joints which are evolving on a vector space.
  ///
  struct LinearAffineTransform
  {
    template<typename ConfigVectorIn, typename Scalar, typename ConfigVectorOut>
    static void run(
      const Eigen::MatrixBase<ConfigVectorIn> & qIn,
      const Scalar & scaling,
      const Scalar & offset,
      const Eigen::MatrixBase<ConfigVectorOut> & qOut)
    {
      assert(qIn.size() == qOut.size());
      PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorOut, qOut).noalias() =
        scaling * qIn + ConfigVectorOut::Constant(qOut.size(), offset);
    }
  };

  struct UnboundedRevoluteAffineTransform
  {
    template<typename ConfigVectorIn, typename Scalar, typename ConfigVectorOut>
    static void run(
      const Eigen::MatrixBase<ConfigVectorIn> & qIn,
      const Scalar & scaling,
      const Scalar & offset,
      const Eigen::MatrixBase<ConfigVectorOut> & qOut)
    {
      assert(qIn.size() == 2);
      assert(qOut.size() == 2);

      const typename ConfigVectorIn::Scalar & ca = qIn(0);
      const typename ConfigVectorIn::Scalar & sa = qIn(1);

      const typename ConfigVectorIn::Scalar & theta = math::atan2(sa, ca);
      const typename ConfigVectorIn::Scalar & theta_transform = scaling * theta + offset;

      ConfigVectorOut & dest_ = PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorOut, qOut);
      SINCOS(theta_transform, &dest_.coeffRef(1), &dest_.coeffRef(0));
    }
  };

  struct NoAffineTransform
  {
    template<typename ConfigVectorIn, typename Scalar, typename ConfigVectorOut>
    static void run(
      const Eigen::MatrixBase<ConfigVectorIn> &,
      const Scalar &,
      const Scalar &,
      const Eigen::MatrixBase<ConfigVectorOut> &)
    {
      assert(false && "Joint cannot be used with JointMimic.");
    }
  };

  ///
  /// \brief Assign the correct configuration vector space affine transformation according to the
  /// joint type. Must be specialized for every joint type.
  ///
  template<typename Joint>
  struct ConfigVectorAffineTransform
  {
    typedef NoAffineTransform Type;
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_joint_joint_common_operations_hpp__
