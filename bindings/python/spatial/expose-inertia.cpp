// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/inertia.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/inertia.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeInertia()
    {
      // Expose Inertia class
      InertiaPythonVisitor<context::Inertia>::expose();
      StdAlignedVectorPythonVisitor<context::Inertia>::expose("StdVec_Inertia");
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
      serialize<StdAlignedVectorPythonVisitor<context::Inertia>::vector_type>();
#endif
    }

  } // namespace python
} // namespace pinocchio
