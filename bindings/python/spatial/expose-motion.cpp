//
// Copyright (c) 2015-2020 CNRS INRIA
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
      MotionPythonVisitor<context::Motion>::expose();
      StdAlignedVectorPythonVisitor<context::Motion>::expose("StdVec_Motion");
      
      exposeClassicAcceleration();
    }
    
  } // namespace python
} // namespace pinocchio
