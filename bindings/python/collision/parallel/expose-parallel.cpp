//
// Copyright (c) 2024 INRIA
//

#include <omp.h>

#include "pinocchio/bindings/python/fwd.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeParallelGeometry();
    void exposeParallelBroadPhase();

    void exposeParallelCollision()
    {
      namespace bp = boost::python;

      exposeParallelGeometry();
      exposeParallelBroadPhase();
    }

  } // namespace python
} // namespace pinocchio
