//
// Copyright (c) 2017-2018 CNRS INRIA
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

#ifndef __se3_macros_hpp__
#define __se3_macros_hpp__

#if __cplusplus >= 201103L
  #define PINOCCHIO_WITH_CXX11_SUPPORT
#endif

/// \brief Macro to check the current Pinocchio version against a version provided by x.y.z
#define PINOCCHIO_VERSION_AT_LEAST(x,y,z) (PINOCCHIO_MAJOR_VERSION>x || (PINOCCHIO_MAJOR_VERSION>=x && \
(PINOCCHIO_MINOR_VERSION>y || (PINOCCHIO_MINOR_VERSION>=y && \
PINOCCHIO_PATCH_VERSION>=z))))

// This macro can be used to prevent from macro expansion, similarly to EIGEN_NOT_A_MACRO
#define PINOCCHIO_NOT_A_MACRO

namespace se3
{
  namespace helper
  {
    template<typename T> struct argument_type;
    template<typename T, typename U> struct argument_type<T(U)> { typedef U type; };
  }
}

/// \brief Empty macro argument
#define PINOCCHIO_MACRO_EMPTY_ARG

namespace se3
{
  namespace helper
  {
    template<typename D, template<typename> class TypeAccess>
    struct handle_return_type_without_typename
    {
      typedef typename TypeAccess< typename argument_type<void(D)>::type >::type type;
    };
  }
}

#endif // ifndef __se3_macros_hpp__
