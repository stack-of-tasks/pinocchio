//
// Copyright (c) 2016,2018 CNRS
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

#ifndef __math_fwd_hpp__
#define __math_fwd_hpp__

#include <boost/math/constants/constants.hpp>

namespace se3
{
  ///
  /// \brief Returns the value of PI according to the template parameters Scalar
  ///
  /// \tparam Scalar The scalar type of the return pi value
  ///
  template<typename Scalar>
  const Scalar PI()
  { return boost::math::constants::pi<Scalar>(); }
  
  /// The value of PI for double scalar type
  const double PId = PI<double>();
}


#endif //#ifndef __math_fwd_hpp__
