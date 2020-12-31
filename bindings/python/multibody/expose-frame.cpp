//
// Copyright (c) 2015-2020 CNRS INRIA
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
      FramePythonVisitor<context::Frame>::expose();
      StdAlignedVectorPythonVisitor<context::Frame>::expose("StdVec_Frame");
    }
    
  } // namespace python
} // namespace pinocchio
