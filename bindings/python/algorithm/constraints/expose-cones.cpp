//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/serialization/aligned-vector.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/algorithm/constraints/coulomb-friction-cone.hpp"
// #include "pinocchio/bindings/python/serialization/serialization.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeCones()
    {
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
      CoulombFrictionConePythonVisitor<context::CoulombFrictionCone>::expose();
      StdVectorPythonVisitor<context::CoulombFrictionConeVector>::expose(
        "StdVec_CoulombFrictionCone");
      // #ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
      //       serialize<StdAlignedVectorPythonVisitor<context::CoulombFrictionCone>::vector_type>();
      // #endif

      DualCoulombFrictionConePythonVisitor<context::DualCoulombFrictionCone>::expose();
      StdVectorPythonVisitor<context::DualCoulombFrictionConeVector>::expose(
        "StdVec_DualCoulombFrictionCone");
// #ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
//       serialize<StdAlignedVectorPythonVisitor<context::DualCoulombFrictionCone>::vector_type>();
// #endif
#endif
    }

  } // namespace python
} // namespace pinocchio
