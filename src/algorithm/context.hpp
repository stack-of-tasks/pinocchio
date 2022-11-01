//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_context_hpp__
#define __pinocchio_algorithm_context_hpp__

#include "pinocchio/container/aligned-vector.hpp"

namespace pinocchio {

  template<typename Scalar> struct CoulombFrictionConeTpl;
  typedef CoulombFrictionConeTpl<double> CoulombFrictionCone;

  template<typename Scalar> struct DualCoulombFrictionConeTpl;
  typedef DualCoulombFrictionConeTpl<double> DualCoulombFrictionCone;

  namespace context {
    typedef CoulombFrictionConeTpl<context::Scalar> CoulombFrictionCone;
    typedef DualCoulombFrictionConeTpl<context::Scalar> DualCoulombFrictionCone;

    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(CoulombFrictionCone) CoulombFrictionConeVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(DualCoulombFrictionCone) DualCoulombFrictionConeVector;
  } // namespace context

} // namespace pinocchio

#endif // #ifndef __pinocchio_algorithm_context_hpp__
