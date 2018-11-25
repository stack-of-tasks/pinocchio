//
// Copyright (c) 2015-2016 CNRS
//

#ifndef __pinocchio_fcl_convertion_hpp__
#define __pinocchio_fcl_convertion_hpp__

#include <hpp/fcl/math/transform.h>
#include "pinocchio/spatial/se3.hpp"

namespace pinocchio
{
  inline fcl::Transform3f toFclTransform3f(const SE3 & m)
  {
    return fcl::Transform3f(m.rotation(), m.translation());
  }

  inline SE3 toPinocchioSE3(const fcl::Transform3f & tf)
  {
    return SE3(tf.getRotation(), tf.getTranslation());
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_fcl_convertion_hpp__
