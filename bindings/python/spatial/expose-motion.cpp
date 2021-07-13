//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/motion.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/motion.hpp"
#include "pinocchio/bindings/python/spatial/classic-acceleration.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
namespace pinocchio
{
  namespace python
  {
    
    void exposeMotion()
    {
      MotionPythonVisitor<context::Motion>::expose();
      StdAlignedVectorPythonVisitor<context::Motion>::expose("StdVec_Motion");
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
      serialize<StdAlignedVectorPythonVisitor<context::Motion>::vector_type>();
#endif
      exposeClassicAcceleration();
    }
    
  } // namespace python
} // namespace pinocchio
