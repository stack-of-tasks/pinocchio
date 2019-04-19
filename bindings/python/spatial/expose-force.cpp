//
// Copyright (c) 2015-2016 CNRS
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
      ForcePythonVisitor<Force>::expose();
      StdAlignedVectorPythonVisitor<Force>::expose("StdVec_Force");
    }
    
  } // namespace python
} // namespace pinocchio
