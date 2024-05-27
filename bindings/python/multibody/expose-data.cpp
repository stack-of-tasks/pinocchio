//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/data.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeData()
    {
      DataPythonVisitor<context::Data>::expose();
    }

  } // namespace python
} // namespace pinocchio
