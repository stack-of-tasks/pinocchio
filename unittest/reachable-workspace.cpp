//
// Copyright (c) 2016-2023 CNRS INRIA
//

#include "pinocchio/extra/reachable-workspace.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include <coal/collision_object.h>
#endif // PINOCCHIO_WITH_HPP_FCL

/// @brief Create a spherical joint with a stick of length l attached to it
/// @param length length of the stick
/// @return pinochio model
static pinocchio::Model createSpherical(double length)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model modelR;
  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1);
  pos.translation() = SE3::LinearType(0, 0., 0.);

  Model::JointIndex idx;
  Eigen::VectorXd effortMax = Eigen::VectorXd::Constant(3, 0.2);
  Eigen::VectorXd velLim = Eigen::VectorXd::Constant(3, 0.5);
  Eigen::VectorXd posMin(4);
  posMin << 0, 0, 0.70710678, 0.70710678;
  Eigen::VectorXd posMax(4);
  posMax << 0.0, 0.0, 1.0, 0.0;
  idx = modelR.addJoint(0, JointModelSpherical(), pos, "s", effortMax, effortMax, posMin, posMax);
  modelR.appendBodyToJoint(idx, inertia);
  pos.translation() = SE3::LinearType(length, 0., 0.);
  Frame f("frame_test", idx, modelR.getFrameId("s"), pos, OP_FRAME);
  modelR.addFrame(f);

  return modelR;
}

/// @brief Create a fixture structure for boost test case. It will create a 3DOF robot with
/// prismatic joints and initialize all variables needed for tests of the reachable workspace.
struct robotCreationFixture
{
  robotCreationFixture()
  {
    using namespace pinocchio;
    length = 1;
    filename = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/3DOF_planar.urdf");

    pinocchio::urdf::buildModel(filename, model);
    SE3 pos(1);
    pos.translation() = SE3::LinearType(0, 0., 0.);
    Frame f("frame_test", 3, model.getFrameId("px"), pos, OP_FRAME);
    model.addFrame(f);

    q = Eigen::VectorXd::Zero(3);

    param.n_samples = 5;
    param.facet_dims = 2;

    time_horizon = 0.5;
    frame_name = model.nframes - 1;
  }
  // Robot model
  pinocchio::Model model;
  // Name of the urdf
  std::string filename;
  // Configuration vector for which the workspace will be computed
  Eigen::VectorXd q;
  // Parameter of the algorithm
  pinocchio::ReachableSetParams param;
  // Length of the articulation
  double length;
  // Time horizon for reachable workspace computation
  double time_horizon;
  // Frame for which the workspace will be computed
  int frame_name;
};

#ifdef PINOCCHIO_WITH_HPP_FCL
/// @brief Create an obstacle to add to the geometry model
/// @param distance where to put the object
/// @param dimension dimension of the box
static void
addObstacle(pinocchio::GeometryModel & geom_model, const double distance, const double dimension)
{
  std::shared_ptr<pinocchio::fcl::CollisionGeometry> geometry =
    std::make_shared<pinocchio::fcl::Box>(dimension, dimension, dimension);
  std::string geometry_object_name = "obstacle";
  pinocchio::SE3 geomPlacement(1);
  geomPlacement.translation() = pinocchio::SE3::LinearType(distance, 0, 0);

  pinocchio::Model::JointIndex idxJ = 0;
  pinocchio::Model::FrameIndex idxF = 0;

  pinocchio::GeometryObject geometry_object(
    geometry_object_name, idxJ, idxF, geomPlacement, geometry);
  geom_model.addGeometryObject(geometry_object);
}
#endif
BOOST_AUTO_TEST_SUITE(ReachableWorkspace)

/// @brief test generation combination function and compare to the output of the python function
/// `itertools.combinations`
BOOST_AUTO_TEST_CASE(test_combination_generation)
{
  int ndof = 6;
  int ncomb = 2;

  Eigen::VectorXi indices = Eigen::VectorXi::Zero(ncomb);

  int size = static_cast<int>(
    boost::math::factorial<double>(ndof) / boost::math::factorial<double>(ncomb)
    / boost::math::factorial<double>(ndof - ncomb));
  Eigen::MatrixXi res(size, ncomb);
  for (int k = 0; k < size; k++)
  {
    pinocchio::internal::generateCombination(ndof, ncomb, indices);

    res.row(k) = indices;
  }

  Eigen::MatrixXi trueRes(size, ncomb);
  trueRes << 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 1, 2, 1, 3, 1, 4, 1, 5, 2, 3, 2, 4, 2, 5, 3, 4, 3, 5, 4,
    5;
  BOOST_CHECK(res == trueRes);
}

/// @brief test of the cartesian product function and compare it to the python function
/// `itertools.product`
BOOST_AUTO_TEST_CASE(test_compute_product)
{
  int repeat = 5;

  Eigen::VectorXd temp(3);
  temp << 2, 1, 0;
  int n_ps = static_cast<int>(std::pow(temp.size(), repeat));

  Eigen::VectorXi indices = Eigen::VectorXi::Zero(repeat);

  Eigen::MatrixXd res(n_ps, repeat);
  Eigen::VectorXd temp_(repeat);
  for (int k = 0; k < n_ps; k++)
  {
    pinocchio::internal::productCombination(temp, repeat, indices, temp_);
    res.row(k) = temp_;
  }

  Eigen::MatrixXd trueRes(n_ps, repeat);
  trueRes << 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 0, 2, 2, 2, 1, 2, 2, 2, 2, 1, 1, 2, 2, 2, 1,
    0, 2, 2, 2, 0, 2, 2, 2, 2, 0, 1, 2, 2, 2, 0, 0, 2, 2, 1, 2, 2, 2, 2, 1, 2, 1, 2, 2, 1, 2, 0, 2,
    2, 1, 1, 2, 2, 2, 1, 1, 1, 2, 2, 1, 1, 0, 2, 2, 1, 0, 2, 2, 2, 1, 0, 1, 2, 2, 1, 0, 0, 2, 2, 0,
    2, 2, 2, 2, 0, 2, 1, 2, 2, 0, 2, 0, 2, 2, 0, 1, 2, 2, 2, 0, 1, 1, 2, 2, 0, 1, 0, 2, 2, 0, 0, 2,
    2, 2, 0, 0, 1, 2, 2, 0, 0, 0, 2, 1, 2, 2, 2, 2, 1, 2, 2, 1, 2, 1, 2, 2, 0, 2, 1, 2, 1, 2, 2, 1,
    2, 1, 1, 2, 1, 2, 1, 0, 2, 1, 2, 0, 2, 2, 1, 2, 0, 1, 2, 1, 2, 0, 0, 2, 1, 1, 2, 2, 2, 1, 1, 2,
    1, 2, 1, 1, 2, 0, 2, 1, 1, 1, 2, 2, 1, 1, 1, 1, 2, 1, 1, 1, 0, 2, 1, 1, 0, 2, 2, 1, 1, 0, 1, 2,
    1, 1, 0, 0, 2, 1, 0, 2, 2, 2, 1, 0, 2, 1, 2, 1, 0, 2, 0, 2, 1, 0, 1, 2, 2, 1, 0, 1, 1, 2, 1, 0,
    1, 0, 2, 1, 0, 0, 2, 2, 1, 0, 0, 1, 2, 1, 0, 0, 0, 2, 0, 2, 2, 2, 2, 0, 2, 2, 1, 2, 0, 2, 2, 0,
    2, 0, 2, 1, 2, 2, 0, 2, 1, 1, 2, 0, 2, 1, 0, 2, 0, 2, 0, 2, 2, 0, 2, 0, 1, 2, 0, 2, 0, 0, 2, 0,
    1, 2, 2, 2, 0, 1, 2, 1, 2, 0, 1, 2, 0, 2, 0, 1, 1, 2, 2, 0, 1, 1, 1, 2, 0, 1, 1, 0, 2, 0, 1, 0,
    2, 2, 0, 1, 0, 1, 2, 0, 1, 0, 0, 2, 0, 0, 2, 2, 2, 0, 0, 2, 1, 2, 0, 0, 2, 0, 2, 0, 0, 1, 2, 2,
    0, 0, 1, 1, 2, 0, 0, 1, 0, 2, 0, 0, 0, 2, 2, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 2, 2, 2, 2, 1, 2, 2,
    2, 1, 1, 2, 2, 2, 0, 1, 2, 2, 1, 2, 1, 2, 2, 1, 1, 1, 2, 2, 1, 0, 1, 2, 2, 0, 2, 1, 2, 2, 0, 1,
    1, 2, 2, 0, 0, 1, 2, 1, 2, 2, 1, 2, 1, 2, 1, 1, 2, 1, 2, 0, 1, 2, 1, 1, 2, 1, 2, 1, 1, 1, 1, 2,
    1, 1, 0, 1, 2, 1, 0, 2, 1, 2, 1, 0, 1, 1, 2, 1, 0, 0, 1, 2, 0, 2, 2, 1, 2, 0, 2, 1, 1, 2, 0, 2,
    0, 1, 2, 0, 1, 2, 1, 2, 0, 1, 1, 1, 2, 0, 1, 0, 1, 2, 0, 0, 2, 1, 2, 0, 0, 1, 1, 2, 0, 0, 0, 1,
    1, 2, 2, 2, 1, 1, 2, 2, 1, 1, 1, 2, 2, 0, 1, 1, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 2, 1, 0, 1, 1, 2,
    0, 2, 1, 1, 2, 0, 1, 1, 1, 2, 0, 0, 1, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 2, 0, 1, 1, 1, 1, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 2, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 2, 2, 1, 1,
    0, 2, 1, 1, 1, 0, 2, 0, 1, 1, 0, 1, 2, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 2, 1, 1, 0, 0,
    1, 1, 1, 0, 0, 0, 1, 0, 2, 2, 2, 1, 0, 2, 2, 1, 1, 0, 2, 2, 0, 1, 0, 2, 1, 2, 1, 0, 2, 1, 1, 1,
    0, 2, 1, 0, 1, 0, 2, 0, 2, 1, 0, 2, 0, 1, 1, 0, 2, 0, 0, 1, 0, 1, 2, 2, 1, 0, 1, 2, 1, 1, 0, 1,
    2, 0, 1, 0, 1, 1, 2, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 2, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0,
    1, 0, 0, 2, 2, 1, 0, 0, 2, 1, 1, 0, 0, 2, 0, 1, 0, 0, 1, 2, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0,
    0, 0, 2, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0, 2, 2, 2, 1, 0, 2, 2, 2, 0, 0, 2, 2, 1,
    2, 0, 2, 2, 1, 1, 0, 2, 2, 1, 0, 0, 2, 2, 0, 2, 0, 2, 2, 0, 1, 0, 2, 2, 0, 0, 0, 2, 1, 2, 2, 0,
    2, 1, 2, 1, 0, 2, 1, 2, 0, 0, 2, 1, 1, 2, 0, 2, 1, 1, 1, 0, 2, 1, 1, 0, 0, 2, 1, 0, 2, 0, 2, 1,
    0, 1, 0, 2, 1, 0, 0, 0, 2, 0, 2, 2, 0, 2, 0, 2, 1, 0, 2, 0, 2, 0, 0, 2, 0, 1, 2, 0, 2, 0, 1, 1,
    0, 2, 0, 1, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 1, 0, 2, 0, 0, 0, 0, 1, 2, 2, 2, 0, 1, 2, 2, 1, 0, 1,
    2, 2, 0, 0, 1, 2, 1, 2, 0, 1, 2, 1, 1, 0, 1, 2, 1, 0, 0, 1, 2, 0, 2, 0, 1, 2, 0, 1, 0, 1, 2, 0,
    0, 0, 1, 1, 2, 2, 0, 1, 1, 2, 1, 0, 1, 1, 2, 0, 0, 1, 1, 1, 2, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0,
    1, 1, 0, 2, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 2, 2, 0, 1, 0, 2, 1, 0, 1, 0, 2, 0, 0, 1, 0,
    1, 2, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 2, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 2, 2, 2,
    0, 0, 2, 2, 1, 0, 0, 2, 2, 0, 0, 0, 2, 1, 2, 0, 0, 2, 1, 1, 0, 0, 2, 1, 0, 0, 0, 2, 0, 2, 0, 0,
    2, 0, 1, 0, 0, 2, 0, 0, 0, 0, 1, 2, 2, 0, 0, 1, 2, 1, 0, 0, 1, 2, 0, 0, 0, 1, 1, 2, 0, 0, 1, 1,
    1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 2, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 2, 1, 0,
    0, 0, 2, 0, 0, 0, 0, 1, 2, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0;
  BOOST_CHECK(res == trueRes);
}

/// @brief test of the function computeJointVel, which mix 2 vectors according to a set of indexes
BOOST_AUTO_TEST_CASE(test_compute_vel_config)
{
  int ndof = 9;
  int c1 = 3;
  int c2 = 6;
  Eigen::VectorXd res1 = Eigen::VectorXd::Constant(c1, 2.);
  Eigen::VectorXd res2 = Eigen::VectorXd::Constant(c2, 4.);

  Eigen::VectorXi comb(c1);
  comb << 1, 5, 8;

  Eigen::VectorXd resTotal(ndof);

  pinocchio::internal::computeJointVel(res1, res2, comb, resTotal);

  Eigen::VectorXd resTrue(ndof);
  resTrue << 4, 2, 4, 4, 4, 2, 4, 4, 2;

  BOOST_CHECK(resTrue == resTotal);
}

/// @brief test of the vertex computation for a 2DOf planar robot. Verify that vertex are inside the
/// rectangle of the joint limits.
BOOST_FIXTURE_TEST_CASE(test_compute_vertex, robotCreationFixture)
{
  using namespace pinocchio;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd vertex;

  double constraint = 0.2;
  auto f_ = [this, &constraint](const Model & model, Data & data) -> bool {
    SE3 pos = data.oMf[frame_name];
    if (pos.translation()(0) < constraint)
      return false;
    else
      return true;
  };

  pinocchio::internal::computeVertex(model, q, time_horizon, frame_name, f_, vertex, param);

  BOOST_CHECK(vertex.rows() > 0);
  BOOST_CHECK(vertex.cols() > 0);
  for (int k = 0; k < vertex.cols(); k++)
  {
    BOOST_CHECK(vertex(0, k) <= length && vertex(0, k) >= constraint);
    BOOST_CHECK(vertex(1, k) <= length && vertex(1, k) >= 0);
    BOOST_CHECK(vertex(2, k) <= length && vertex(2, k) >= 0);
  }
}

#ifdef PINOCCHIO_WITH_HPP_FCL
/// @brief test of the vertex computation for a 2DOf planar robot with an obstacle in its workspace.
/// Verify that vertex are inside the rectangle of the joint limits and that faces are computed
BOOST_FIXTURE_TEST_CASE(test_reachable_workspace_with_collision, robotCreationFixture)
{
  using namespace pinocchio;

  double dims = 0.1;
  double distance = length - dims;

  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model);
  addObstacle(geom_model, distance, dims);

  geom_model.addAllCollisionPairs();

  ReachableSetResults res;

  pinocchio::reachableWorkspaceWithCollisionsHull(
    model, geom_model, q, time_horizon, frame_name, res, param);

  BOOST_CHECK(res.vertex.rows() > 0);
  BOOST_CHECK(res.faces.rows() > 0);

  for (int k = 0; k < res.vertex.cols(); k++)
  {
    BOOST_CHECK(res.vertex(0, k) <= length && res.vertex(0, k) >= 0);
    BOOST_CHECK(res.vertex(1, k) <= length && res.vertex(1, k) >= 0);
    BOOST_CHECK(res.vertex(2, k) <= length && res.vertex(2, k) >= 0);

    if (res.vertex(1, k) > dims / 2 && res.vertex(2, k) > dims / 2)
      BOOST_CHECK(res.vertex(0, k) < distance);
  }
}
#endif

/// @brief test of the full pipeline for reachable workspace computation
BOOST_AUTO_TEST_CASE(test_reachable_workspace)
{
  using namespace Eigen;
  using namespace pinocchio;

  // Load the urdf model
  Model model;
  pinocchio::buildModels::manipulator(model);

  Eigen::VectorXd q = (model.upperPositionLimit + model.lowerPositionLimit) / 2;
  ReachableSetResults res;

  ReachableSetParams param;
  param.n_samples = 5;
  param.facet_dims = 2;

  double time_horizon = 0.2;
  int frame_name = model.nframes - 1;

  pinocchio::reachableWorkspaceHull(model, q, time_horizon, frame_name, res, param);

  BOOST_CHECK(res.vertex.cols() > 0);
  BOOST_CHECK(res.faces.rows() > 0);
}

/// @brief test reachable algorithm in cas nq!= nv
BOOST_AUTO_TEST_CASE(test_spherical)
{
  double length = 0.4;
  pinocchio::Model modelR = createSpherical(length);

  Eigen::VectorXd q = (modelR.upperPositionLimit + modelR.lowerPositionLimit) / 2;
  pinocchio::normalize(modelR, q);

  pinocchio::ReachableSetResults res;

  pinocchio::ReachableSetParams param;
  param.n_samples = 5;
  param.facet_dims = 1;

  double time_horizon = 0.2;
  int frame_name = modelR.nframes - 1;

  pinocchio::reachableWorkspaceHull(modelR, q, time_horizon, frame_name, res, param);

  BOOST_CHECK(res.vertex.cols() > 0);
  BOOST_CHECK(res.faces.rows() > 0);
}

BOOST_AUTO_TEST_SUITE_END()
