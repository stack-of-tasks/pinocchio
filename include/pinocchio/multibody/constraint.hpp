//
// Copyright (c) 2015-2019 CNRS, INRIA
//

#ifndef __pinocchio_multibody_constraint_hpp__
#define __pinocchio_multibody_constraint_hpp__

#include "pinocchio/macros.hpp"

namespace pinocchio
{
  
  template<int _Dim, typename _Scalar, int _Options=0> struct ConstraintTpl;
  
  typedef ConstraintTpl<1,double,0> Constraint1d;
  typedef ConstraintTpl<3,double,0> Constraint3d;
  typedef ConstraintTpl<6,double,0> Constraint6d;
  typedef ConstraintTpl<Eigen::Dynamic,double,0> ConstraintXd;
  
} // namespace pinocchio

#include "pinocchio/multibody/constraint-base.hpp"
#include "pinocchio/multibody/constraint-generic.hpp"

#endif // ifndef __pinocchio_multibody_constraint_hpp__

