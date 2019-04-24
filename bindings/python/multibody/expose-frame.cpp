//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/frame.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeFrame()
    {
      FramePythonVisitor::expose();
      StdAlignedVectorPythonVisitor<Frame>::expose("StdVec_Frame");
    }
    
  } // namespace python
} // namespace pinocchio
