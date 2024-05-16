//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_collision_fcl_convertion_hpp__
#define __pinocchio_collision_fcl_convertion_hpp__

#include <hpp/fcl/math/transform.h>
#include "pinocchio/spatial/se3.hpp"

namespace pinocchio
{
  template<typename Scalar>
  inline hpp::fcl::Transform3f toFclTransform3f(const SE3Tpl<Scalar> & m)
  {
    SE3Tpl<double, 0> m_ = m.template cast<double>();
    return hpp::fcl::Transform3f(m_.rotation(), m_.translation());
  }

  inline SE3 toPinocchioSE3(const hpp::fcl::Transform3f & tf)
  {
    typedef SE3::Scalar Scalar;
    return SE3(tf.getRotation().cast<Scalar>(), tf.getTranslation().cast<Scalar>());
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_fcl_convertion_hpp__
