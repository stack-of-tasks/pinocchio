//
// Copyright (c) 2018 CNRS INRIA
//

#ifndef __pinocchio_fwd_hpp__
#define __pinocchio_fwd_hpp__

// Forward declaration of main pinocchio namespace
namespace pinocchio {}

#include "pinocchio/macros.hpp"
#include "pinocchio/deprecation.hpp"
#include "pinocchio/warning.hpp"
#include "pinocchio/config.hpp"

#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT
#include "pinocchio/math/cppad.hpp"
#else
#include <Eigen/Dense>
#endif

#include "pinocchio/eigen-macros.hpp"

namespace pinocchio
{
  ///
  /// \brief Common traits structure to fully define base classes for CRTP.
  ///
  template<class C> struct traits {};
  
  ///
  /// \brief Type of the cast of a class C templated by Scalar and Options, to a new NewScalar type.
  ///        This class should be specialized for each types.
  ///
  template<typename NewScalar, class C> struct CastType;

  /// \brief Argument position.
  ///        Used as template parameter to refer to an argument.
  enum ArgumentPosition
  {
    ARG0 = 0,
    ARG1 = 1,
    ARG2 = 2,
    ARG3 = 3,
    ARG4 = 4
  };
}

#endif // #ifndef __pinocchio_fwd_hpp__
