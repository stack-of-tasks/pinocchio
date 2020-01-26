//
// Copyright (c) 2018 INRIA
//

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

#endif // ifndef __pinocchio_deprecated_macros_hpp__
