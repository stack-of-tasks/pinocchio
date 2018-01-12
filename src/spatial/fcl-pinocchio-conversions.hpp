//
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_fcl_convertion_hpp__
#define __se3_fcl_convertion_hpp__

#include <hpp/fcl/math/transform.h>
#include "pinocchio/spatial/se3.hpp"

namespace se3
{
  inline fcl::Transform3f toFclTransform3f(const SE3 & m)
  {
    return fcl::Transform3f(m.rotation(), m.translation());
  }

  inline SE3 toPinocchioSE3(const fcl::Transform3f & tf)
  {
    return SE3(tf.getRotation(), tf.getTranslation());
  }

} // namespace se3

#endif // ifndef __se3_fcl_convertion_hpp__
