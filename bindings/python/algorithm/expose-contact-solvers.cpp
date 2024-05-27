//
// Copyright (c) 2022-2024 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"

namespace pinocchio
{
  namespace python
  {

    // Forward declaration
    void exposePGSContactSolver();
    void exposeADMMContactSolver();

    void exposeContactSolvers()
    {
      exposePGSContactSolver();
      exposeADMMContactSolver();
    }

  } // namespace python
} // namespace pinocchio
