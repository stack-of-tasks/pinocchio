//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_se3_hpp__
#define __pinocchio_se3_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/macros.hpp"

#define PINOCCHIO_SE3_TYPEDEF_GENERIC(Derived,TYPENAME) \
typedef TYPENAME traits<Derived>::Scalar Scalar; \
typedef TYPENAME traits<Derived>::AngularType AngularType; \
typedef TYPENAME traits<Derived>::LinearType LinearType; \
typedef TYPENAME traits<Derived>::AngularRef AngularRef; \
typedef TYPENAME traits<Derived>::LinearRef LinearRef; \
typedef TYPENAME traits<Derived>::ConstAngularRef ConstAngularRef; \
typedef TYPENAME traits<Derived>::ConstLinearRef ConstLinearRef; \
typedef TYPENAME traits<Derived>::ActionMatrixType ActionMatrixType; \
typedef TYPENAME traits<Derived>::HomogeneousMatrixType HomogeneousMatrixType; \
typedef TYPENAME traits<Derived>::PlainType PlainType; \
enum {  \
  Options = traits<Derived>::Options,  \
  LINEAR = traits<Derived>::LINEAR,  \
  ANGULAR = traits<Derived>::ANGULAR \
}

#define PINOCCHIO_SE3_TYPEDEF_TPL(Derived) \
PINOCCHIO_SE3_TYPEDEF_GENERIC(Derived,typename)

#define PINOCCHIO_SE3_TYPEDEF(Derived) \
PINOCCHIO_SE3_TYPEDEF_GENERIC(Derived,PINOCCHIO_MACRO_EMPTY_ARG)

namespace pinocchio
{

  /* Type returned by the "se3Action" and "se3ActionInverse" functions. */
  template<typename D>
  struct SE3GroupAction { typedef D ReturnType; };

} // namespace pinocchio

#include "pinocchio/spatial/se3-base.hpp"
#include "pinocchio/spatial/se3-tpl.hpp"

#endif // ifndef __pinocchio_se3_hpp__
