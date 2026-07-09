//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE serialization_multibody

#include <iostream>

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/delassus-operator.hpp"

#include "pinocchio/serialization.hpp"
#include "serialization.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

template<>
struct empty_contructor_algo<pinocchio::GeometryObject>
{
  static pinocchio::GeometryObject * run()
  {
    return new pinocchio::GeometryObject("", 0, 0, pinocchio::SE3::Identity(), nullptr);
  }
};

BOOST_AUTO_TEST_CASE(test_multibody_serialization)
{
  using namespace pinocchio;

  Frame frame("frame", 0, 0, SE3::Random(), SENSOR);
  generic_test(frame, TEST_SERIALIZATION_FOLDER "/Frame", "Frame");
}

BOOST_AUTO_TEST_CASE(test_collision_pair)
{
  using namespace pinocchio;

  CollisionPair collision_pair(1, 2);
  generic_test(collision_pair, TEST_SERIALIZATION_FOLDER "/CollisionPair", "CollisionPair");
}

BOOST_AUTO_TEST_CASE(test_model_item)
{
  using namespace pinocchio;

  typedef GeometryObject::Base GeometryObject_ModelItem;
  GeometryObject_ModelItem model_item("pinocchio", 1, 2, SE3::Random());
  generic_test(model_item, TEST_SERIALIZATION_FOLDER "/ModelItem", "ModelItem");
}

BOOST_AUTO_TEST_CASE(test_geometry_object)
{
  using namespace pinocchio;

  {
    GeometryObject geometry_object("nullptr", 1, 2, SE3::Random(), nullptr);
    generic_test(geometry_object, TEST_SERIALIZATION_FOLDER "/GeometryObject", "GeometryObject");
  }

#ifdef PINOCCHIO_WITH_COLLISION
  {
    coal::Box box(1., 2., 3.);
    generic_test(box, TEST_SERIALIZATION_FOLDER "/Box", "Box");
  }

  {
    typedef GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;
    CollisionGeometryPtr box_ptr = CollisionGeometryPtr(new coal::Box(1., 2., 3.));
    GeometryObject geometry_object("box", 1, 2, SE3::Random(), box_ptr);
    generic_test(geometry_object, TEST_SERIALIZATION_FOLDER "/GeometryObject", "GeometryObject");
  }
#endif
}

BOOST_AUTO_TEST_CASE(test_geometry_model_and_data_serialization)
{
  using namespace pinocchio;

  Model model;
  buildModels::humanoid(model);
  Data data(model);

  // Empty structures
  {
    GeometryModel geom_model;
    generic_test(geom_model, TEST_SERIALIZATION_FOLDER "/GeometryModel", "GeometryModel");

    GeometryData geom_data(geom_model);
    generic_test(geom_data, TEST_SERIALIZATION_FOLDER "/GeometryData", "GeometryData");
  }

#ifdef PINOCCHIO_WITH_COLLISION
  {
    pinocchio::GeometryModel geom_model;
    pinocchio::buildModels::humanoidGeometries(model, geom_model);
    // Append new objects
    {
      using namespace coal;
      BVHModel<OBBRSS> * bvh_ptr = new BVHModel<OBBRSS>();
      //      bvh_ptr->beginModel();
      //      bvh_ptr->addSubModel(p1, t1);
      //      bvh_ptr->endModel();

      GeometryObject obj_bvh(
        "bvh", 0, 0, SE3::Identity(), GeometryObject::CollisionGeometryPtr(bvh_ptr));
      geom_model.addGeometryObject(obj_bvh);

      const double min_altitude = -1.;
      const double x_dim = 1., y_dim = 2.;
      const Eigen::Index nx = 100, ny = 200;
      const Eigen::MatrixXd heights = Eigen::MatrixXd::Random(ny, nx);

      HeightField<OBBRSS> * hfield_ptr =
        new HeightField<OBBRSS>(x_dim, y_dim, heights, min_altitude);

      GeometryObject obj_hfield(
        "hfield", 0, 0, SE3::Identity(), GeometryObject::CollisionGeometryPtr(hfield_ptr));
      geom_model.addGeometryObject(obj_hfield);
    }
    generic_test(geom_model, TEST_SERIALIZATION_FOLDER "/GeometryModel", "GeometryModel");

    pinocchio::GeometryData geom_data(geom_model);
    const Eigen::VectorXd q = pinocchio::neutral(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

    generic_test(geom_data, TEST_SERIALIZATION_FOLDER "/GeometryData", "GeometryData");
  }
#endif // PINOCCHIO_WITH_COLLISION
}
