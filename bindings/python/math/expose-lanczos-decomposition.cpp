//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/math/lanczos-decomposition.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeLanczosDecomposition()
    {
#ifndef PINOCCHIO_PYTHON_SKIP_CASADI_UNSUPPORTED
      typedef LanczosDecompositionTpl<context::MatrixXs> LanczosDecomposition;
      LanczosDecompositionPythonVisitor<LanczosDecomposition>::expose();
#endif // PINOCCHIO_PYTHON_SKIP_CASADI_UNSUPPORTED
    }

  } // namespace python
} // namespace pinocchio
