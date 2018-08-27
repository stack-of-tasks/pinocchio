//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_force_hpp__
#define __se3_force_hpp__

#include <Eigen/Core>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"

#define FORCE_TYPEDEF_GENERIC(Derived,TYPENAME) \
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
typedef TYPENAME traits<Derived>::ForcePlain ForcePlain; \
enum {  \
LINEAR = traits<Derived>::LINEAR,  \
ANGULAR = traits<Derived>::ANGULAR \
}

#define FORCE_TYPEDEF_TPL(Derived) \
FORCE_TYPEDEF_GENERIC(Derived,typename)

#define FORCE_TYPEDEF(Derived) \
FORCE_TYPEDEF_GENERIC(Derived,PINOCCHIO_MACRO_EMPTY_ARG)

#include "pinocchio/spatial/force-base.hpp"
#include "pinocchio/spatial/force-dense.hpp"
#include "pinocchio/spatial/force-tpl.hpp"
#include "pinocchio/spatial/force-ref.hpp"

#endif // ifndef __se3_force_hpp__

