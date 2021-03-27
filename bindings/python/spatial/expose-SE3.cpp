//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/se3.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/se3.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeSE3()
    {
      SE3PythonVisitor<SE3>::expose();
      StdAlignedVectorPythonVisitor<SE3>::expose("StdVec_SE3");
      serialize<StdAlignedVectorPythonVisitor<SE3>::vector_type>();
    }
    
  } // namespace python
} // namespace pinocchio
