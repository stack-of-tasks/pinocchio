//
// Copyright (c) 2017-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/fcl/transform.hpp"

namespace pinocchio
{
  namespace python
  {
    void exposeFCL()
    {
      namespace bp = boost::python;
      bp::import("hppfcl");
      
      typedef ::hpp::fcl::Transform3f Transform3f;
      
      // Register implicit conversion SE3 <=> ::hpp::fcl::Transform3f
      bp::implicitly_convertible< SE3,Transform3f >();
      bp::implicitly_convertible< Transform3f,SE3 >();
    }
    
  } // namespace python
} // namespace pinocchio
