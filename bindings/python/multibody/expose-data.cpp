//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/data.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeData()
    {
      DataPythonVisitor::expose();
    }
    
  } // namespace python
} // namespace pinocchio
