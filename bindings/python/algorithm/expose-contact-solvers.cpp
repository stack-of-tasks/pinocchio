//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"

namespace pinocchio
{
namespace python
{

// Forward declaration
void exposePGSContactSolver();

void exposeContactSolvers()
{
  exposePGSContactSolver();
}

} // namespace python
} // namespace pinocchio

