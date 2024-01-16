//
// Copyright (c) 2018-2023 CNRS INRIA
//

#ifndef __pinocchio_codegen_ccpadcg_hpp__
#define __pinocchio_codegen_ccpadcg_hpp__

#define PINOCCHIO_WITH_CPPADCG_SUPPORT

#include "pinocchio/math/fwd.hpp"

#include <cmath>
#include <cppad/cg/support/cppadcg_eigen.hpp>

#include "pinocchio/autodiff/cppad.hpp"

#ifndef PINOCCHIO_WITH_CXX11_SUPPORT
  #error CppADCodeGen requires C++11 or more
#endif

namespace boost
{
  namespace math
  {
    namespace constants
    {
      namespace detail
      {
        template<typename Scalar>
        struct constant_pi< CppAD::cg::CG<Scalar> > : constant_pi<Scalar>
        {
          typedef CppAD::cg::CG<Scalar> CGScalar;
          
          template <int N>
          static inline CGScalar get(const mpl::int_<N>& n)
          {
            return CGScalar(constant_pi<Scalar>::get(n));
          }

#if BOOST_VERSION >= 107700
          template <class T, T value>
          static inline CGScalar get(const std::integral_constant<T, value> &n)
          {
            return CGScalar(constant_pi<Scalar>::get(n));
          }
#else
          template <class T, T value>
          static inline CGScalar get(const boost::integral_constant<T, value> &n)
          {
            return CGScalar(constant_pi<Scalar>::get(n));
          }
#endif
        };
      }
    }
  }
}

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for CppAD input types
    template<typename Scalar>
    struct cast_impl<CppAD::cg::CG<Scalar>,Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const CppAD::cg::CG<Scalar> & x)
      {
        return x.getValue();
      }
    };
  }
} // namespace Eigen

namespace CppAD
{
  template <class Scalar>
  bool isfinite(const AD<Scalar> & x) { return isfinite(Value(x)); }

  namespace cg
  {
    template <class Scalar>
    bool isfinite(const CG<Scalar> &x) { return std::isfinite(x.getValue()); }
  }
}

namespace pinocchio
{
  template<typename Scalar>
  struct TaylorSeriesExpansion< CppAD::cg::CG<Scalar> >
  {
    typedef TaylorSeriesExpansion<Scalar> Base;
    typedef CppAD::cg::CG<Scalar> CGScalar;

    template<int degree>
    static CGScalar precision()
    {
      return CGScalar(Base::template precision<degree>());
    }

  };
} // namespace pinocchio

#endif // #ifndef __pinocchio_codegen_ccpadcg_hpp__
