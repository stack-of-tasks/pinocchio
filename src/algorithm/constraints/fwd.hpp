//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_constraints_fwd_hpp__
#define __pinocchio_algorithm_constraints_fwd_hpp__

#include "pinocchio/algorithm/fwd.hpp"

namespace pinocchio
{
  template<typename Scalar> struct CoulombFrictionConeTpl;
  typedef CoulombFrictionConeTpl<context::Scalar> CoulombFrictionCone;

  template<typename Scalar> struct DualCoulombFrictionConeTpl;
  typedef DualCoulombFrictionConeTpl<context::Scalar> DualCoulombFrictionCone;
}

#endif // ifndef __pinocchio_algorithm_constraints_fwd_hpp__
