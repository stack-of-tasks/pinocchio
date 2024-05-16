//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_context_hpp__
#define __pinocchio_python_context_hpp__

#define PINOCCHIO_PYTHON_SCALAR_TYPE_DEFAULT double

#define PINOCCHIO_PYTHON_CONTEXT_FILE_DEFAULT "pinocchio/bindings/python/context/default.hpp"

#ifndef PINOCCHIO_PYTHON_CONTEXT_FILE
  #define PINOCCHIO_PYTHON_CONTEXT_FILE PINOCCHIO_PYTHON_CONTEXT_FILE_DEFAULT
#endif

#include PINOCCHIO_PYTHON_CONTEXT_FILE

#endif // #ifndef __pinocchio_python_context_hpp__
