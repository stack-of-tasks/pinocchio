//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/symmetric3.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/symmetric3.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeSymmetric3()
    {
      Symmetric3PythonVisitor<Symmetric3>::expose();
      StdAlignedVectorPythonVisitor<Symmetric3>::expose("StdVec_Symmetric3");
      serialize<StdAlignedVectorPythonVisitor<Symmetric3>::vector_type>();
    }
    
  } // namespace python
} // namespace pinocchio
