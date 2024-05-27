//
// Copyright (c) 2024 CNRS INRIA
//

#ifndef __pinocchio_python_extra_extras_hpp__
#define __pinocchio_python_extra_extras_hpp__

#include "pinocchio/bindings/python/fwd.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

#if defined(PINOCCHIO_WITH_EXTRA_SUPPORT)
    void exposeReachableWorkspace();
#endif // defined(PINOCCHIO_WITH_EXTRA_SUPPORT)

    void exposeExtras();

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_extra_extras_hpp__
