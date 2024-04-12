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
      typedef LanczosDecompositionTpl<context::MatrixXs> LanczosDecomposition;
      LanczosDecompositionPythonVisitor<LanczosDecomposition>::expose();
    }
    
  } // namespace python
} // namespace pinocchio
