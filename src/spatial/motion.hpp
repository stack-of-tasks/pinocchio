//
// Copyright (c) 2015-2018 CNRS
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

#define MOTION_TYPEDEF_GENERIC(Derived,TYPENAME) \
typedef TYPENAME traits<Derived>::Scalar Scalar; \
typedef TYPENAME traits<Derived>::Vector3 Vector3; \
typedef TYPENAME traits<Derived>::Vector6 Vector6; \
typedef TYPENAME traits<Derived>::Matrix6 Matrix6; \
typedef TYPENAME traits<Derived>::ToVectorReturnType ToVectorReturnType; \
typedef TYPENAME traits<Derived>::ToVectorConstReturnType ToVectorConstReturnType; \
typedef TYPENAME traits<Derived>::AngularType AngularType; \
typedef TYPENAME traits<Derived>::LinearType LinearType; \
typedef TYPENAME traits<Derived>::ConstAngularType ConstAngularType; \
typedef TYPENAME traits<Derived>::ConstLinearType ConstLinearType; \
typedef TYPENAME traits<Derived>::ActionMatrixType ActionMatrixType; \
typedef TYPENAME traits<Derived>::MotionPlain MotionPlain; \
enum {  \
LINEAR = traits<Derived>::LINEAR,  \
ANGULAR = traits<Derived>::ANGULAR \
}

#define MOTION_TYPEDEF_TPL(Derived) \
MOTION_TYPEDEF_GENERIC(Derived,typename)

#define MOTION_TYPEDEF(Derived) \
MOTION_TYPEDEF_GENERIC(Derived,PINOCCHIO_MACRO_EMPTY_ARG)

namespace se3
{
  namespace internal
  {
    ///
    /// \brief Return type of the ation of a Motion onto an object of type D
    ///
    template<typename D, typename MotionDerived>
    struct MotionAlgebraAction { typedef D ReturnType; };
  }

} // namespace se3

#include "pinocchio/spatial/motion-base.hpp"
#include "pinocchio/spatial/motion-dense.hpp"
#include "pinocchio/spatial/motion-tpl.hpp"
#include "pinocchio/spatial/motion-ref.hpp"
#include "pinocchio/spatial/motion-zero.hpp"

#endif // ifndef __se3_motion_hpp__
