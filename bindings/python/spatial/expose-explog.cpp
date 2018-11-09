//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/explog.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeExplog()
    {
      ExplogPythonVisitor::expose();
    }
    
  } // namespace python
} // namespace pinocchio
