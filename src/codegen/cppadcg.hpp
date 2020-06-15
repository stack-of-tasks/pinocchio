//
// Copyright (c) 2018-2020 CNRS INRIA
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
  bool isfinite(const cg::CG<Scalar> & x) { return std::isfinite(x.getValue()); }
  
  template <class Scalar>
  bool isfinite(const AD<Scalar> & x) { return isfinite(Value(x)); }
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
