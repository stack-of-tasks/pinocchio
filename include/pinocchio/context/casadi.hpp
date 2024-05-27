//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_context_casadi_hpp__
#define __pinocchio_context_casadi_hpp__

#define PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS_DERIVATIVES
#define PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
#define PINOCCHIO_SKIP_ALGORITHM_CONTACT_DYNAMICS
#define PINOCCHIO_SKIP_ALGORITHM_CONTACT_CHOLESKY
#define PINOCCHIO_SKIP_ALGORITHM_CONTACT_JACOBIAN
#define PINOCCHIO_SKIP_ALGORITHM_CHOLESKY
#define PINOCCHIO_SKIP_ALGORITHM_MODEL
#define PINOCCHIO_SKIP_MULTIBODY_SAMPLE_MODELS

#define PINOCCHIO_SKIP_CASADI_UNSUPPORTED

namespace pinocchio
{
  // forward declarations which are necessary to include pinocchio/autodiff/casadi.hpp
  template<typename _Scalar, int _Options>
  struct MotionZeroTpl;
  template<typename _Scalar, int _Options>
  class ForceTpl;
  template<typename _Scalar, int _Options>
  class MotionTpl;
  template<typename _Scalar, int _Options>
  struct SE3Tpl;

  template<typename Derived>
  class ForceBase;
  template<typename Derived>
  class ForceDense;
  template<typename Derived>
  class MotionDense;
  template<typename Vector6>
  class MotionRef;

  namespace internal
  {
    template<typename Class, typename NewScalar, typename Scalar>
    struct cast_call_normalize_method;
    template<typename Type, typename Scalar>
    struct RHSScalarMultiplication;
    template<typename Type, typename Scalar>
    struct LHSScalarMultiplication;
  } // namespace internal

} // namespace pinocchio

#include "pinocchio/autodiff/casadi.hpp"
#define PINOCCHIO_SCALAR_TYPE ::casadi::SX
#include "pinocchio/context/generic.hpp"

#undef PINOCCHIO_SCALAR_TYPE
#endif // #ifndef __pinocchio_context_casadi_hpp__
