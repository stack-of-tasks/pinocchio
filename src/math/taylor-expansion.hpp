//
// Copyright (c) 2018 INRIA
//

#ifndef __pinocchio_math_taylor_expansion_hpp__
#define __pinocchio_math_taylor_expansion_hpp__

#include "pinocchio/math/fwd.hpp"
#include <limits>

namespace pinocchio
{
  
  ///
  /// \brief Helper struct to retrieve some useful information for a Taylor series
  ///        expansion according to the a given Scalar type.
  ///
  /// \tparam Scalar the Scalar type of the Taylor series expansion.
  ///
  template<typename Scalar>
  struct TaylorSeriesExpansion
  {
    ///
    /// \brief Computes the expected tolerance of the argument of a Taylor series expansion for a certain degree
    ///        according to the machine precision of the given input Scalar.
    ///
    /// \tparam degree the degree of the Taylor series expansion.
    ///
    template<int degree>
    static Scalar precision()
    {
      static Scalar value = std::pow(std::numeric_limits<Scalar>::epsilon(),Scalar(1)/Scalar(degree+1));
      return value;
    }
  }; // struct TaylorSeriesExpansion
  
#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT
  
  template<typename Scalar>
  struct TaylorSeriesExpansion< CppAD::AD<Scalar> > : TaylorSeriesExpansion<Scalar>
  {
    typedef TaylorSeriesExpansion<Scalar> Base;
    using Base::precision;
  };
  
#if defined(PINOCCHIO_WITH_CPPADCG_SUPPORT) && defined(PINOCCHIO_WITH_CXX11_SUPPORT)
  
  template<typename Scalar>
  struct TaylorSeriesExpansion< CppAD::cg::CG<Scalar> > : TaylorSeriesExpansion<Scalar>
  {
    typedef TaylorSeriesExpansion<Scalar> Base;
    using Base::precision;
  };
  
#endif
#endif
  
}

#endif // ifndef __pinocchio_math_taylor_expansion_hpp__
