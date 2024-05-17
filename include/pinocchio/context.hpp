//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_context_hpp__
#define __pinocchio_context_hpp__

#define PINOCCHIO_SCALAR_TYPE_DEFAULT double

#define PINOCCHIO_CONTEXT_FILE_DEFAULT "pinocchio/context/default.hpp"

#ifndef PINOCCHIO_CONTEXT_FILE
  #define PINOCCHIO_CONTEXT_FILE PINOCCHIO_CONTEXT_FILE_DEFAULT
#endif

#include PINOCCHIO_CONTEXT_FILE

#endif // #ifndef __pinocchio_context_hpp__
