//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/model.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeModel()
    {
      ModelPythonVisitor<context::Model>::expose();
    }

  } // namespace python
} // namespace pinocchio
