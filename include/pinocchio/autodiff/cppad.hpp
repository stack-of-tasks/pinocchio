//
// Copyright (c) 2018-2023 CNRS INRIA
//

#pragma once

// IWYU pragma: always_keep

#define PINOCCHIO_WITH_CPPAD_SUPPORT

// IWYU pragma: begin_keep
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/version.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/mpl/int.hpp>

#include <cppad/cppad.hpp>

#include "pinocchio/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/eigen-common.hpp"
#include "pinocchio/fwd.hpp"

#include "pinocchio/utils/static-if.hpp"

#include "pinocchio/math.hpp"
#include "pinocchio/spatial.hpp"
// IWYU pragma: end_keep

namespace boost
{
  namespace math
  {
    namespace constants
    {
      namespace detail
      {
        template<typename Scalar>
        struct constant_pi<CppAD::AD<Scalar>> : constant_pi<Scalar>
        {
          typedef CppAD::AD<Scalar> ADScalar;

          template<int N>
          static inline ADScalar get(const mpl::int_<N> & n)
          {
            return ADScalar(constant_pi<Scalar>::get(n));
          }

#if BOOST_VERSION >= 107700
          template<class T, T value>
          static inline ADScalar get(const std::integral_constant<T, value> & n)
          {
            return ADScalar(constant_pi<Scalar>::get(n));
          }
#else
          template<class T, T value>
          static inline ADScalar get(const boost::integral_constant<T, value> & n)
          {
            return ADScalar(constant_pi<Scalar>::get(n));
          }
#endif
        };
      } // namespace detail
    } // namespace constants
  } // namespace math
} // namespace boost

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for CppAD input types
    template<typename Scalar>
    struct cast_impl<CppAD::AD<Scalar>, Scalar>
    {
      EIGEN_DEVICE_FUNC
      static inline Scalar run(const CppAD::AD<Scalar> & x)
      {
        return CppAD::Value(x);
      }
    };
  } // namespace internal
} // namespace Eigen

// Source from #include <cppad/example/cppad_eigen.hpp>
namespace Eigen
{
  template<class Base>
  struct NumTraits<CppAD::AD<Base>>
  { // type that corresponds to the real part of an AD<Base> value
    typedef CppAD::AD<Base> Real;
    // type for AD<Base> operations that result in non-integer values
    typedef CppAD::AD<Base> NonInteger;
    //  type to use for numeric literals such as "2" or "0.5".
    typedef CppAD::AD<Base> Literal;
    // type for nested value inside an AD<Base> expression tree
    typedef CppAD::AD<Base> Nested;

    enum
    {
      // does not support complex Base types
      IsComplex = 0,
      // does not support integer Base types
      IsInteger = 0,
      // only support signed Base types
      IsSigned = 1,
      // must initialize an AD<Base> object
      RequireInitialization = 1,
      // computational cost of the corresponding operations
      ReadCost = 1,
      AddCost = 2,
      MulCost = 2
    };

    // machine epsilon with type of real part of x
    // (use assumption that Base is not complex)
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline CppAD::AD<Base> epsilon(void)
    {
      return CppAD::numeric_limits<CppAD::AD<Base>>::epsilon();
    }

    // relaxed version of machine epsilon for comparison of different
    // operations that should result in the same value
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline CppAD::AD<Base> dummy_precision(void)
    {
      return 100. * CppAD::numeric_limits<CppAD::AD<Base>>::epsilon();
    }

    // minimum normalized positive value
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline CppAD::AD<Base> lowest(void)
    {
      return CppAD::numeric_limits<CppAD::AD<Base>>::min();
    }

    // maximum finite value
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline CppAD::AD<Base> highest(void)
    {
      return CppAD::numeric_limits<CppAD::AD<Base>>::max();
    }

    // number of decimal digits that can be represented without change.
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits10(void)
    {
      return CppAD::numeric_limits<CppAD::AD<Base>>::digits10;
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits()
    {
      return NumTraits<Base>::digits();
    }

#if EIGEN_VERSION_AT_LEAST(3, 4, 90)
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int max_digits10()
    {
      return NumTraits<Base>::max_digits10();
    }
#endif
  };
} // namespace Eigen

// Source from #include <cppad/example/cppad_eigen.hpp>
namespace CppAD
{
  // functions that return references
  template<class Base>
  const AD<Base> & conj(const AD<Base> & x)
  {
    return x;
  }
  template<class Base>
  const AD<Base> & real(const AD<Base> & x)
  {
    return x;
  }

  // functions that return values (note abs is defined by cppad.hpp)
  template<class Base>
  AD<Base> imag(const AD<Base> & /*x*/)
  {
    return CppAD::AD<Base>(0.);
  }
  template<class Base>
  AD<Base> abs2(const AD<Base> & x)
  {
    return x * x;
  }

  template<typename Scalar>
  AD<Scalar> min(const AD<Scalar> & x, const AD<Scalar> & y)
  {
    using ::pinocchio::internal::if_then_else;
    using ::pinocchio::internal::LT;
    return if_then_else(LT, y, x, y, x);
  }

  template<typename Scalar>
  AD<Scalar> max(const AD<Scalar> & x, const AD<Scalar> & y)
  {
    using ::pinocchio::internal::if_then_else;
    using ::pinocchio::internal::LT;
    return if_then_else(LT, x, y, y, x);
  }
} // namespace CppAD

namespace CppAD
{
  template<class Scalar>
  bool isfinite(const AD<Scalar> & x)
  {
    return isfinite(Value(x));
  }
} // namespace CppAD

// IWYU pragma: begin_exports
#include "pinocchio/src/autodiff/cppad/utils/static-if.hxx"
#include "pinocchio/src/autodiff/cppad/math/quaternion.hxx"
#include "pinocchio/src/autodiff/cppad/math/taylor-series-expansion.hxx"
#include "pinocchio/src/autodiff/cppad/spatial/se3-tpl.hxx"
#include "pinocchio/src/autodiff/cppad/spatial/log.hxx"
#include "pinocchio/src/autodiff/cppad/algorithm/aba.hxx"
// IWYU pragma: end_exports
