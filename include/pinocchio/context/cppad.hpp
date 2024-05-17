//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_context_cppad_hpp__
#define __pinocchio_context_cppad_hpp__

#include <cppad/cppad.hpp>

#define PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS_DERIVATIVES
#define PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
#define PINOCCHIO_SKIP_ALGORITHM_CONTACT_DYNAMICS
#define PINOCCHIO_SKIP_ALGORITHM_CONTACT_CHOLESKY
#define PINOCCHIO_SKIP_ALGORITHM_CONTACT_JACOBIAN
#define PINOCCHIO_SKIP_ALGORITHM_CHOLESKY
#define PINOCCHIO_SKIP_MULTIBODY_SAMPLE_MODELS

namespace pinocchio
{
  // forward declarations which are necessary to include pinocchio/autodiff/cppad.hpp
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

  typedef MotionTpl<::CppAD::AD<double>, 0> Motion;

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

#include "pinocchio/autodiff/cppad.hpp"
#define PINOCCHIO_SCALAR_TYPE ::CppAD::AD<double>
#include "pinocchio/context/generic.hpp"

#undef PINOCCHIO_SCALAR_TYPE
#endif // #ifndef __pinocchio_context_cppad_hpp__
