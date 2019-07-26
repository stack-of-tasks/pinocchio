//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/motion.hpp"
#include "pinocchio/bindings/python/spatial/classic-acceleration.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeMotion()
    {
      MotionPythonVisitor<Motion>::expose();
      StdAlignedVectorPythonVisitor<Motion>::expose("StdVec_Motion");
      
      exposeClassicAcceleration<Motion,Motion>();
    }
    
  } // namespace python
} // namespace pinocchio
