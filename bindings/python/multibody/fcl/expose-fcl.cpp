//
// Copyright (c) 2017-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/fcl/transform.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

#define HPP_FCL_SKIP_EIGEN_BOOST_SERIALIZATION
  #include <hpp/fcl/serialization/BVH_model.h>
#undef HPP_FCL_SKIP_EIGEN_BOOST_SERIALIZATION

namespace pinocchio
{
  namespace python
  {
    void exposeFCL()
    {
      namespace bp = boost::python;
      bp::import("hppfcl");
      
      using namespace ::hpp::fcl;
      
      // Register implicit conversion SE3 <=> ::hpp::fcl::Transform3f
      bp::implicitly_convertible< SE3,Transform3f >();
      bp::implicitly_convertible< Transform3f,SE3 >();
      
      // Expose serialization of basic geometries to binary buffers
      serialize< BVHModel<OBB> >();
      serialize< BVHModel<RSS> >();
      serialize< BVHModel<OBBRSS> >();
    }
    
  } // namespace python
} // namespace pinocchio
