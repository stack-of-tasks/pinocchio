//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_autodiff_cppad_minmax_hpp__
#define __pinocchio_autodiff_cppad_minmax_hpp__

#include "pinocchio/utils/static-if.hpp"

#ifdef __CUDACC__
#define EIGEN_DEVICE_FUNC __host__ __device__
// We need cuda_runtime.h to ensure that that EIGEN_USING_STD_MATH macro
// works properly on the device side
#include <cuda_runtime.h>
#else
#define EIGEN_DEVICE_FUNC
#endif

#ifndef EIGEN_STRONG_INLINE
#if EIGEN_COMP_MSVC || EIGEN_COMP_ICC
#define EIGEN_STRONG_INLINE __forceinline
#else
#define EIGEN_STRONG_INLINE inline
#endif
#endif

#if (__GNUC__ > 4) || ((__GNUC__ == 4) && (__GNUC_MINOR__ >= 2))
#define EIGEN_ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define EIGEN_ALWAYS_INLINE EIGEN_STRONG_INLINE
#endif


namespace CppAD
{
  template<typename Scalar> struct AD;
}

namespace Eigen
{
  namespace numext
  {
    template<typename Scalar>
    EIGEN_DEVICE_FUNC
    EIGEN_ALWAYS_INLINE ::CppAD::AD<Scalar> mini(const ::CppAD::AD<Scalar>& x,
                                                 const ::CppAD::AD<Scalar>& y)
    {
      using ::pinocchio::internal::if_then_else;
      using ::pinocchio::internal::LT;
      return if_then_else(LT, y, x, y, x);
    }
    
    template<typename Scalar>
    EIGEN_DEVICE_FUNC
    EIGEN_ALWAYS_INLINE ::CppAD::AD<Scalar> maxi(const ::CppAD::AD<Scalar>& x,
                                                 const ::CppAD::AD<Scalar>& y)
    {
      using ::pinocchio::internal::if_then_else;
      using ::pinocchio::internal::LT;
      return if_then_else(LT, x, y, y, x);
    }
  }
}

#endif
