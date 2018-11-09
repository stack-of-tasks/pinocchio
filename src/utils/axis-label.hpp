//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_axis_label_hpp__
#define __pinocchio_axis_label_hpp__

namespace pinocchio
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

#endif // __pinocchio_axis_label_hpp__
