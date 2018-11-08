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
  #ifdef PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_1 // for backward compatibility
    #define WITH_HPP_FCL
  #endif
#endif

#ifdef PINOCCHIO_WITH_URDFDOM
  #ifdef PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_1 // for backward compatibility
    #define WITH_URDFDOM
  #endif
#endif

#ifdef PINOCCHIO_WITH_LUA5
  #ifdef PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_1 // for backward compatibility
    #define WITH_LUA5
  #endif
#endif

#endif // ifndef __pinocchio_deprecated_macros_hpp__
