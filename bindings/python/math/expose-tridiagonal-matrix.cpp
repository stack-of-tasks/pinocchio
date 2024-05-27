//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/math/tridiagonal-matrix.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeTridiagonalMatrix()
    {
      typedef TridiagonalSymmetricMatrixTpl<context::Scalar, context::Options>
        TridiagonalSymmetricMatrix;
      TridiagonalSymmetricMatrixPythonVisitor<TridiagonalSymmetricMatrix>::expose();
    }

  } // namespace python
} // namespace pinocchio
