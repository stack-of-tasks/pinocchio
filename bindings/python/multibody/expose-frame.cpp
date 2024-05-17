//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/frame.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/frame.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeFrame()
    {
      FramePythonVisitor<context::Frame>::expose();
      StdAlignedVectorPythonVisitor<context::Frame>::expose("StdVec_Frame");
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
      serialize<StdAlignedVectorPythonVisitor<context::Frame>::vector_type>();
#endif
    }

  } // namespace python
} // namespace pinocchio
