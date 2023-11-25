//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_force_hpp__
#define __pinocchio_force_hpp__

#include "pinocchio/spatial/fwd.hpp"
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

#endif // ifndef __pinocchio_force_hpp__

