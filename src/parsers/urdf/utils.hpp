//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_parsers_urdf_utils_hpp__
#define __pinocchio_parsers_urdf_utils_hpp__

#include "pinocchio/spatial/se3.hpp"
#include <urdf_model/pose.h>

namespace pinocchio
{
  namespace urdf
  {
    namespace details
    {
      ///
      /// \brief Convert URDF Pose quantity to SE3.
      ///
      /// \param[in] M The input URDF Pose.
      ///
      /// \return The converted pose/transform pinocchio::SE3.
      ///
      SE3 convertFromUrdf(const ::urdf::Pose & M);
    }
  }
}

#endif // __pinocchio_parsers_urdf_utils_hpp__
