//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/model.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeModel()
    {
      ModelPythonVisitor<Model>::expose();
    }
    
  } // namespace python
} // namespace pinocchio
