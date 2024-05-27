//
// Copyright (c) 2018-2024 CNRS INRIA
//

#ifndef __pinocchio_fwd_hpp__
#define __pinocchio_fwd_hpp__

// Forward declaration of the main pinocchio namespace
namespace pinocchio
{
}

#ifdef _WIN32
  #include <windows.h>
  #undef far
  #undef near
#endif

#include <cassert>

#include "pinocchio/macros.hpp"
#include "pinocchio/deprecation.hpp"
#include "pinocchio/warning.hpp"
#include "pinocchio/config.hpp"
#include "pinocchio/unsupported.hpp"

#include "pinocchio/utils/helpers.hpp"
#include "pinocchio/utils/cast.hpp"
#include "pinocchio/utils/check.hpp"

#include "pinocchio/container/boost-container-limits.hpp"

#ifdef PINOCCHIO_EIGEN_CHECK_MALLOC
  #ifndef EIGEN_RUNTIME_NO_MALLOC
    #define EIGEN_RUNTIME_NO_MALLOC_WAS_NOT_DEFINED
    #define EIGEN_RUNTIME_NO_MALLOC
  #endif
#endif

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#ifdef PINOCCHIO_WITH_ACCELERATE_SUPPORT
  #include <Eigen/AccelerateSupport>
#endif

#ifdef PINOCCHIO_EIGEN_CHECK_MALLOC
  #ifdef EIGEN_RUNTIME_NO_MALLOC_WAS_NOT_DEFINED
    #undef EIGEN_RUNTIME_NO_MALLOC
    #undef EIGEN_RUNTIME_NO_MALLOC_WAS_NOT_DEFINED
  #endif
#endif

#include "pinocchio/eigen-macros.hpp"
#ifdef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE
  #include <unsupported/Eigen/CXX11/Tensor>
#endif

#include "pinocchio/core/binary-op.hpp"
#include "pinocchio/core/unary-op.hpp"

#include <cstddef> // std::size_t

namespace pinocchio
{
  ///
  /// \brief Common traits structure to fully define base classes for CRTP.
  ///
  template<class C>
  struct traits
  {
  };

  /// \brief Blank type
  struct Blank
  {
  };

  namespace internal
  {
    template<typename T>
    struct traits
    {
    };
  } // namespace internal

  template<class Derived>
  struct NumericalBase
  {
    typedef typename traits<Derived>::Scalar Scalar;
  };

  ///
  /// \brief Type of the cast of a class C templated by Scalar and Options, to a new NewScalar type.
  ///        This class should be specialized for each types.
  ///
  template<typename NewScalar, class C>
  struct CastType;

  ///
  ///  \brief Cast scalar type from type FROM to type TO.
  ///
  template<typename To, typename From>
  struct ScalarCast
  {
    static To cast(const From & value)
    {
      return static_cast<To>(value);
    }
  };

  template<typename To, typename From>
  To scalar_cast(const From & value)
  {
    return ScalarCast<To, From>::cast(value);
  }

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

  enum AssignmentOperatorType
  {
    SETTO,
    ADDTO,
    RMTO
  };

  /** This value means that a positive quantity (e.g., a size) is not known at compile-time, and
   * that instead the value is stored in some runtime variable.
   */
  const int Dynamic = -1;

  /// \brief Return type undefined
  ///        This is an helper structure to help internal diagnosis.
  struct ReturnTypeNotDefined;

  typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;
} // namespace pinocchio

#include "pinocchio/context.hpp"

#endif // #ifndef __pinocchio_fwd_hpp__
