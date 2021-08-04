//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/pool/model.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
#include "pinocchio/bindings/python/multibody/pool/geometry.hpp"
#endif

namespace pinocchio
{
  namespace python
  {
    
    void exposePool()
    {
      ModelPoolPythonVisitor<ModelPool>::expose();
#ifdef PINOCCHIO_WITH_HPP_FCL
      GeometryPoolPythonVisitor<GeometryPool>::expose();
#endif
    }
    
  } // namespace python
} // namespace pinocchio

