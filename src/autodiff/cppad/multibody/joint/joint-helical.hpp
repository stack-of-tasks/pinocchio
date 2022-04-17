//
// Copyright (c) 2022 INRIA
//


#ifndef __pinocchio_autodiff_cppad_multibody_joint_joint_helical_hxx__
#define __pinocchio_autodiff_cppad_multibody_joint_joint_helical_hxx__

#include "pinocchio/multibody/joint/joint-helical.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, int axis>
  struct JointModelHelicalTpl<::CppAD::AD<Scalar>, Options, axis>
  {
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    JointModelHelicalTpl<Scalar,Options,axis> cast() const
    {
      typedef JointModelHelicalTpl<Scalar,Options,axis> ReturnType;
      ReturnType res(::CppAD::Value(this->m_pitch));
      res.setIndexes(this->id(),this->idx_q(),this->idx_v());
      return res;
    }
  }; // struct JointModelHelicalTpl
}

#endif // ifndef  __pinocchio_autodiff_cppad_multibody_joint_joint_helical_hxx__
