//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/motion.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/motion.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeMotion()
    {
      MotionPythonVisitor<Motion>::expose();
      StdAlignedVectorPythonVisitor<Motion>::expose("StdVec_Motion");
      serialize<StdAlignedVectorPythonVisitor<Motion>::vector_type>();
    }
    
  } // namespace python
} // namespace pinocchio
