//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/pool/model.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposePool()
    {
      ModelPoolPythonVisitor<context::ModelPool>::expose();
    }

  } // namespace python
} // namespace pinocchio
