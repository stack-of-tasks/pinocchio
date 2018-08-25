//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_se3_hpp__
#define __se3_se3_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/macros.hpp"

#define SE3_TYPEDEF_GENERIC(Derived,TYPENAME) \
typedef TYPENAME traits<Derived>::Scalar Scalar; \
typedef TYPENAME traits<Derived>::AngularType AngularType; \
typedef TYPENAME traits<Derived>::LinearType LinearType; \
typedef TYPENAME traits<Derived>::AngularRef AngularRef; \
typedef TYPENAME traits<Derived>::LinearRef LinearRef; \
typedef TYPENAME traits<Derived>::ConstAngularRef ConstAngularRef; \
typedef TYPENAME traits<Derived>::ConstLinearRef ConstLinearRef; \
typedef TYPENAME traits<Derived>::ActionMatrixType ActionMatrixType; \
typedef TYPENAME traits<Derived>::HomogeneousMatrixType HomogeneousMatrixType; \
enum {  \
Options = traits<Derived>::Options,  \
LINEAR = traits<Derived>::LINEAR,  \
ANGULAR = traits<Derived>::ANGULAR \
}

#define SE3_TYPEDEF_TPL(Derived) \
SE3_TYPEDEF_GENERIC(Derived,typename)

#define SE3_TYPEDEF(Derived) \
SE3_TYPEDEF_GENERIC(Derived,PINOCCHIO_MACRO_EMPTY_ARG)

namespace se3
{

  /* Type returned by the "se3Action" and "se3ActionInverse" functions. */
  namespace internal 
  {
    template<typename D>
    struct SE3GroupAction { typedef D ReturnType; };
  }

} // namespace se3

#include "pinocchio/spatial/se3-base.hpp"
#include "pinocchio/spatial/se3-tpl.hpp"

#endif // ifndef __se3_se3_hpp__
