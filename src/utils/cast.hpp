//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_utils_cast_hpp__
#define __pinocchio_utils_cast_hpp__

#include <Eigen/Core>

namespace pinocchio
{
  template<typename NewScalar, typename Scalar>
  NewScalar cast(const Scalar & value)
  {
    return Eigen::internal::cast_impl<Scalar,NewScalar>::run(value);
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_utils_cast_hpp__
