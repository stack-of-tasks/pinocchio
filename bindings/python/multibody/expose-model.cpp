//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/model.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeModel()
    {
      ModelPythonVisitor::expose();
    }
    
  } // namespace python
} // namespace pinocchio
