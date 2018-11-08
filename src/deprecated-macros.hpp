//
// Copyright (c) 2018 INRIA
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

#ifndef __pinocchio_deprecated_macros_hpp__
#define __pinocchio_deprecated_macros_hpp__

#ifdef PINOCCHIO_WITH_HPP_FCL
  #ifndef WITH_HPP_FCL
    #define WITH_HPP_FCL // for backward compatibility
  #endif
  #ifndef PINOCCHIO_DISABLE_DEPRECATED_MACRO_WARNING
    #pragma message("'WITH_HPP_FCL' is deprecated. Please use PINOCCHIO_WITH_HPP_FCL instead.")
  #endif
#endif

#ifdef PINOCCHIO_WITH_URDFDOM
  #ifndef WITH_URDFDOM
    #define WITH_URDFDOM // for backward compatibility
  #endif
  #ifndef PINOCCHIO_DISABLE_DEPRECATED_MACRO_WARNING
    #pragma message("'WITH_URDFDOM' is deprecated. Please use PINOCCHIO_WITH_URDFDOM instead.")
  #endif
#endif

#ifdef PINOCCHIO_WITH_LUA5
  #ifndef WITH_LUA5
    #define WITH_LUA5 // for backward compatibility
  #endif
  #ifndef PINOCCHIO_DISABLE_DEPRECATED_MACRO_WARNING
    #pragma message("'WITH_LUA5' is deprecated. Please use PINOCCHIO_WITH_LUA5 instead.")
  #endif
#endif

#endif // ifndef __pinocchio_deprecated_macros_hpp__
