//
// Copyright (c) 2018-2023 CNRS INRIA
//

#ifndef __pinocchio_fwd_hpp__
#define __pinocchio_fwd_hpp__

// Forward declaration of the main pinocchio namespace
namespace pinocchio {}

#ifdef _WIN32
  #include <windows.h>
  #undef far
  #undef near
#endif

#include "pinocchio/macros.hpp"
#include "pinocchio/deprecation.hpp"
#include "pinocchio/warning.hpp"
#include "pinocchio/config.hpp"

#include "pinocchio/utils/helpers.hpp"
#include "pinocchio/utils/cast.hpp"
#include "pinocchio/utils/check.hpp"

#include "pinocchio/container/boost-container-limits.hpp"

#define PINOCCHIO_SCALAR_TYPE_DEFAULT double
#ifndef PINOCCHIO_SCALAR_TYPE
#define PINOCCHIO_SCALAR_TYPE PINOCCHIO_SCALAR_TYPE_DEFAULT
#endif

#ifdef PINOCCHIO_EIGEN_CHECK_MALLOC
  #ifndef EIGEN_RUNTIME_NO_MALLOC
    #define EIGEN_RUNTIME_NO_MALLOC_WAS_NOT_DEFINED
    #define EIGEN_RUNTIME_NO_MALLOC
  #endif
#endif
  
#include <Eigen/Core>

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
  template<class C> struct traits {};
  
  namespace internal
  {
    template<typename T> struct traits {};
  }

  template<class Derived>
  struct NumericalBase
  {
    typedef typename traits<Derived>::Scalar Scalar;
  };
  
  ///
  /// \brief Type of the cast of a class C templated by Scalar and Options, to a new NewScalar type.
  ///        This class should be specialized for each types.
  ///
  template<typename NewScalar, class C> struct CastType;

  ///
  /// \brief Cast scalar type from type FROM to type TO.
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
    return ScalarCast<To,From>::cast(value);
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

  /** This value means that a positive quantity (e.g., a size) is not known at compile-time, and that instead the value is
    * stored in some runtime variable.
    */
  const int Dynamic = -1;

  /// \brief Return type undefined
  ///        This is an helper structure to help internal diagnosis.
  struct ReturnTypeNotDefined;

  // Read and write
  template <typename Derived>
  Eigen::Ref<typename Derived::PlainObject> make_ref(const Eigen::MatrixBase<Derived> & x){
      return Eigen::Ref<typename Derived::PlainObject>(x.const_cast_derived());
  }

  // Read-only
  template <typename M>
  auto make_const_ref(Eigen::MatrixBase<M> const & m)
      -> Eigen::Ref<typename M::PlainObject const> {
      return m;
  }
}

#include "pinocchio/context.hpp"

#endif // #ifndef __pinocchio_fwd_hpp__
