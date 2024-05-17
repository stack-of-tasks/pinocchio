//
// Copyright (c) 2024 CNRS INRIA
//

#include "pinocchio/bindings/python/extra/extras.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeExtras()
    {
#if defined(PINOCCHIO_WITH_EXTRA_SUPPORT)
      exposeReachableWorkspace();
#endif // defined(PINOCCHIO_WITH_EXTRA_SUPPORT)
    }

  } // namespace python
} // namespace pinocchio
