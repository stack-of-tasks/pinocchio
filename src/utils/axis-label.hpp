//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_axis_label_hpp__
#define __se3_axis_label_hpp__

namespace se3
{
  
  ///
  /// \brief Generate the label (X, Y or Z) of the axis relative to its index.
  ///
  /// \tparam axis Index of the axis (either 0 for X, 1 for Y and Z for 2).
  ///
  /// \returns a char containing the label of the axis.
  ///
  template<int axis> inline char axisLabel();
  
  template<> inline char axisLabel<0>() { return 'X'; }
  template<> inline char axisLabel<1>() { return 'Y'; }
  template<> inline char axisLabel<2>() { return 'Z'; }
}

#endif // __se3_axis_label_hpp__
