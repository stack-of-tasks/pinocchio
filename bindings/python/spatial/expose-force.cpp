//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/force.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/force.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeForce()
    {
      ForcePythonVisitor<Force>::expose();
      StdAlignedVectorPythonVisitor<Force>::expose("StdVec_Force");
      serialize<StdAlignedVectorPythonVisitor<Force>::vector_type>();
    }
    
  } // namespace python
} // namespace pinocchio
