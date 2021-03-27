//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/inertia.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/inertia.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeInertia()
    {
      InertiaPythonVisitor<Inertia>::expose();
      StdAlignedVectorPythonVisitor<Inertia>::expose("StdVec_Inertia");
      serialize<StdAlignedVectorPythonVisitor<Inertia>::vector_type>();
    }
    
  } // namespace python
} // namespace pinocchio
