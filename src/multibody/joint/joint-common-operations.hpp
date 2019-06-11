//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_multibody_joint_joint_common_operations_hpp__
#define __pinocchio_multibody_joint_joint_common_operations_hpp__

#include "pinocchio/macros.hpp"
#include <boost/type_traits.hpp>

namespace pinocchio
{
  namespace internal
  {
    template<typename Scalar, bool is_floating_point = boost::is_floating_point<Scalar>::value>
    struct PerformStYSInversion
    {
      template<typename M1, typename M2>
      static void run(const Eigen::MatrixBase<M1> & StYS,
                      const Eigen::MatrixBase<M2> & Dinv)
      {
        M2 & Dinv_ = PINOCCHIO_EIGEN_CONST_CAST(M2,Dinv);
        Dinv_.setIdentity();
        StYS.llt().solveInPlace(Dinv_);
      }
    };
    
    template<typename Scalar>
    struct PerformStYSInversion<Scalar, false>
    {
      template<typename M1, typename M2>
      static void run(const Eigen::MatrixBase<M1> & StYS,
                      const Eigen::MatrixBase<M2> & Dinv)
      {
        M2 & Dinv_ = PINOCCHIO_EIGEN_CONST_CAST(M2,Dinv);
        inverse(StYS,Dinv_);
      }
    };
  }
}

#endif // ifndef __pinocchio_multibody_joint_joint_common_operations_hpp__
