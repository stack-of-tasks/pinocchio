//
// Copyright (c) 2015-2017 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_motion_hpp__
#define __se3_motion_hpp__

#include <Eigen/Core>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/se3.hpp"
//#include "pinocchio/spatial/force.hpp"

namespace se3
{
  namespace internal
  {
    ///
    /// \brief Return type of the ation of a Motion onto an object of type D
    ///
    template<typename D>
    struct MotionAlgebraAction { typedef D ReturnType; };
  }

} // namespace se3

#include "pinocchio/spatial/motion-base.hpp"
#include "pinocchio/spatial/motion-dense.hpp"
#include "pinocchio/spatial/motion-tpl.hpp"
#include "pinocchio/spatial/motion-ref.hpp"
#include "pinocchio/spatial/motion-zero.hpp"

#endif // ifndef __se3_motion_hpp__
