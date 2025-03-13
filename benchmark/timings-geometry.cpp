//
// Copyright (c) 2015-2025 CNRS INRIA
//

#include "model-fixture.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#ifdef PINOCCHIO_WITH_HPP_FCL
  #include "pinocchio/collision/collision.hpp"
#endif // PINOCCHIO_WITH_HPP_FCL
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include <iostream>

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

struct GeometryFixture : benchmark::Fixture
{
  void SetUp(benchmark::State &)
  {
    model = MODEL;
    data = pinocchio::Data(model);
    geometry_model = GEOMETRY_MODEL;
    geometry_data = pinocchio::GeometryData(geometry_model);

    const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
    q = randomConfiguration(model, -qmax, qmax);
    v = Eigen::VectorXd::Random(model.nv);
    a = Eigen::VectorXd::Random(model.nv);
  }

  void TearDown(benchmark::State &)
  {
  }

  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::GeometryModel geometry_model;
  pinocchio::GeometryData geometry_data;

  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd a;

  static pinocchio::Model MODEL;
  static pinocchio::GeometryModel GEOMETRY_MODEL;

  static void GlobalSetUp(const ExtraArgs &)
  {
    std::string romeo_filename =
      PINOCCHIO_MODEL_DIR
      + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
    std::vector<std::string> package_dirs;
    package_dirs.push_back(PINOCCHIO_MODEL_DIR);

    pinocchio::urdf::buildModel(romeo_filename, pinocchio::JointModelFreeFlyer(), MODEL);
    pinocchio::urdf::buildGeom(
      MODEL, romeo_filename, pinocchio::COLLISION, GEOMETRY_MODEL, package_dirs);

    std::cout << "nq = " << MODEL.nq << std::endl;
    std::cout << "nv = " << MODEL.nv << std::endl;
    std::cout << "name = " << MODEL.name << std::endl;
    std::cout << "--" << std::endl;
  }
};
pinocchio::Model GeometryFixture::MODEL;
pinocchio::GeometryModel GeometryFixture::GEOMETRY_MODEL;

// FORWARD_KINEMATICS_Q

PINOCCHIO_DONT_INLINE static void forwardKinematicsQCall(
  const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model, data, q);
}
BENCHMARK_DEFINE_F(GeometryFixture, FORWARD_KINEMATICS_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    forwardKinematicsQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, FORWARD_KINEMATICS_Q)->Apply(CustomArguments);

// UPDATE_GEOMETRY_PLACEMENTS

PINOCCHIO_DONT_INLINE static void updateGeometryPlacementsCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::GeometryModel & geometry_model,
  pinocchio::GeometryData & geometry_data,
  const Eigen::VectorXd & q)
{
  pinocchio::updateGeometryPlacements(model, data, geometry_model, geometry_data, q);
}
BENCHMARK_DEFINE_F(GeometryFixture, UPDATE_GEOMETRY_PLACEMENTS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    updateGeometryPlacementsCall(model, data, geometry_model, geometry_data, q);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, UPDATE_GEOMETRY_PLACEMENTS)->Apply(CustomArguments);

#ifdef PINOCCHIO_WITH_HPP_FCL

struct CollisionFixture : GeometryFixture
{
  void SetUp(benchmark::State & st)
  {
    GeometryFixture::SetUp(st);
  }

  void TearDown(benchmark::State & st)
  {
    GeometryFixture::TearDown(st);
  }

  static void GlobalSetUp(const ExtraArgs & extra_args)
  {
    GeometryFixture::GlobalSetUp(extra_args);
    GEOMETRY_MODEL.addAllCollisionPairs();
  }
};

// UPDATE_PLACEMENTS_AND_COMPUTE_COLLISIONS

PINOCCHIO_DONT_INLINE static void computeCollisionCall(
  const pinocchio::GeometryModel & geometry_model,
  pinocchio::GeometryData & geometry_data,
  size_t index)
{
  computeCollision(geometry_model, geometry_data, index);
}
BENCHMARK_DEFINE_F(CollisionFixture, UPDATE_PLACEMENTS_AND_COMPUTE_COLLISIONS)(
  benchmark::State & st)
{
  for (auto _ : st)
  {
    updateGeometryPlacementsCall(model, data, geometry_model, geometry_data, q);
    for (size_t i = 0; i < geometry_model.collisionPairs.size(); ++i)
    {
      computeCollisionCall(geometry_model, geometry_data, i);
    }
  }
}
BENCHMARK_REGISTER_F(CollisionFixture, UPDATE_PLACEMENTS_AND_COMPUTE_COLLISIONS)
  ->Apply(CustomArguments);

// COMPUTE_COLLISIONS

PINOCCHIO_DONT_INLINE static void computeCollisionsCall(
  const pinocchio::GeometryModel & geometry_model, pinocchio::GeometryData & geometry_data)
{
  pinocchio::computeCollisions(geometry_model, geometry_data, true);
}
BENCHMARK_DEFINE_F(CollisionFixture, COMPUTE_COLLISIONS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeCollisionsCall(geometry_model, geometry_data);
  }
}
BENCHMARK_REGISTER_F(CollisionFixture, COMPUTE_COLLISIONS)->Apply(CustomArguments);

// COMPUTE_DISTANCES

PINOCCHIO_DONT_INLINE static void computeDistancesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::GeometryModel & geometry_model,
  pinocchio::GeometryData & geometry_data,
  const Eigen::VectorXd & q)
{
  pinocchio::computeDistances(model, data, geometry_model, geometry_data, q);
}
BENCHMARK_DEFINE_F(CollisionFixture, COMPUTE_DISTANCES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeDistancesCall(model, data, geometry_model, geometry_data, q);
  }
}
BENCHMARK_REGISTER_F(CollisionFixture, COMPUTE_DISTANCES)->Apply(CustomArguments);

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL

#ifdef PINOCCHIO_WITH_HPP_FCL
PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(CollisionFixture::GlobalSetUp);
#else
PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(GeometryFixture::GlobalSetUp);
#endif
