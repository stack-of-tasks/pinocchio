//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/inertia.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeInertia()
    {
      InertiaPythonVisitor<Inertia>::expose();
      StdAlignedVectorPythonVisitor<Inertia>::expose("StdVec_Inertia");
    }
    
  } // namespace python
} // namespace pinocchio
