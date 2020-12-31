//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/force.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeForce()
    {
      ForcePythonVisitor<context::Force>::expose();
      StdAlignedVectorPythonVisitor<context::Force>::expose("StdVec_Force");
    }
    
  } // namespace python
} // namespace pinocchio
