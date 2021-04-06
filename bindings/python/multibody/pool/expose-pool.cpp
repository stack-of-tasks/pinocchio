//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/pool/model.hpp"
#include "pinocchio/bindings/python/multibody/pool/geometry.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposePool()
    {
      ModelPoolPythonVisitor<ModelPool>::expose();
      GeometryPoolPythonVisitor<GeometryPool>::expose();
    }
    
  } // namespace python
} // namespace pinocchio

