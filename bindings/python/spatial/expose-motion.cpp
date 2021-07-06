//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/motion.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/motion.hpp"
#include "pinocchio/bindings/python/spatial/classic-acceleration.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeMotion()
    {
      exposeClassicAcceleration();
      MotionPythonVisitor<context::Motion>::expose();
      StdAlignedVectorPythonVisitor<context::Motion>::expose("StdVec_Motion");
      serialize<StdAlignedVectorPythonVisitor<context::Motion>::vector_type>();
    }
    
  } // namespace python
} // namespace pinocchio
