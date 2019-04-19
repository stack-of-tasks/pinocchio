//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/motion.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeMotion()
    {
      MotionPythonVisitor<Motion>::expose();
      StdAlignedVectorPythonVisitor<Motion>::expose("StdVec_Motion");
    }
    
  } // namespace python
} // namespace pinocchio
