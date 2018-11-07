//
// Copyright (c) 2018 INRIA
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_math_taylor_expansion_hpp__
#define __se3_math_taylor_expansion_hpp__

#include <cmath>
#include <limits>

namespace se3
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
  
}

#endif // ifndef __se3_math_taylor_expansion_hpp__
