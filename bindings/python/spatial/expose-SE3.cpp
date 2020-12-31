//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/se3.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeSE3()
    {
      SE3PythonVisitor<context::SE3>::expose();
      StdAlignedVectorPythonVisitor<context::SE3>::expose("StdVec_SE3");
    }
    
  } // namespace python
} // namespace pinocchio
