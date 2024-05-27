//
// Copyright (c) 2017-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/collision/fcl/transform.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

#define HPP_FCL_SKIP_EIGEN_BOOST_SERIALIZATION
#include <hpp/fcl/serialization/BVH_model.h>
#include <hpp/fcl/serialization/hfield.h>
#include <hpp/fcl/serialization/geometric_shapes.h>
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
      bp::implicitly_convertible<SE3, Transform3f>();
      bp::implicitly_convertible<Transform3f, SE3>();

      // Expose serialization of basic geometries to binary buffers
      serialize<TriangleP>();
      serialize<Sphere>();
      serialize<Capsule>();
      serialize<Box>();
      serialize<Cone>();
      serialize<Cylinder>();
      serialize<Plane>();
      serialize<Halfspace>();
      serialize<BVHModel<OBB>>();
      serialize<BVHModel<RSS>>();
      serialize<BVHModel<OBBRSS>>();

      serialize<HeightField<OBBRSS>>();
      serialize<HeightField<AABB>>();
    }

  } // namespace python
} // namespace pinocchio
