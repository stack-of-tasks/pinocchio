//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/symmetric3.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/symmetric3.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeSymmetric3()
    {
      Symmetric3PythonVisitor<context::Symmetric3>::expose();
      StdAlignedVectorPythonVisitor<context::Symmetric3>::expose("StdVec_Symmetric3");
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
      serialize<StdAlignedVectorPythonVisitor<context::Symmetric3>::vector_type>();
#endif // ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
    }

  } // namespace python
} // namespace pinocchio
