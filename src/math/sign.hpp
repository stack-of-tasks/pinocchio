//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_math_sign_hpp__
#define __pinocchio_math_sign_hpp__

namespace pinocchio
{
  ///
  /// \brief Returns the robust sign of t
  ///
  template<typename Scalar>
  Scalar sign(const Scalar & t)
  {
    return (t > Scalar(0)) - (t < Scalar(0));
  }
}

#endif //#ifndef __pinocchio_math_sign_hpp__
