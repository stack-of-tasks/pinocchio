//
// Copyright (c) 2015-2020 CNRS INRIA
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
      InertiaPythonVisitor<context::Inertia>::expose();
      StdAlignedVectorPythonVisitor<context::Inertia>::expose("StdVec_Inertia");
    }
    
  } // namespace python
} // namespace pinocchio
