//
// Copyright (c) 2018-2019 INRIA
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
      static Scalar value = math::pow(std::numeric_limits<Scalar>::epsilon(),Scalar(1)/Scalar(degree+1));
      return value;
    }
  }; // struct TaylorSeriesExpansion

}

#endif // ifndef __pinocchio_math_taylor_expansion_hpp__
