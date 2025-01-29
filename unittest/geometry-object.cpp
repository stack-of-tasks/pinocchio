//
// Copyright (c) 2022 INRIA
//

#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry-object.hpp"

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_clone)
{
  coal::Sphere * sphere_ptr = new coal::Sphere(0.5);
  GeometryObject::CollisionGeometryPtr sphere_shared_ptr(sphere_ptr);
  GeometryObject geom_obj("sphere", 0, 0, SE3::Random(), sphere_shared_ptr);

  const GeometryObject geom_obj_clone = geom_obj.clone();
  BOOST_CHECK(geom_obj_clone == geom_obj);

  sphere_ptr->radius = 1.;
  BOOST_CHECK(geom_obj_clone != geom_obj);
}

BOOST_AUTO_TEST_SUITE_END()
