//
// Copyright (c) 2015-2016 CNRS
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
      SE3PythonVisitor<SE3>::expose();
      StdAlignedVectorPythonVisitor<SE3>::expose("StdVec_SE3");
    }
    
  } // namespace python
} // namespace pinocchio
