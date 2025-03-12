//
// Copyright (c) 2021-2025 INRIA
//

#include "model-fixture.hpp"

#include "pinocchio/algorithm/parallel/rnea.hpp"
#include "pinocchio/algorithm/parallel/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include "pinocchio/collision/tree-broadphase-manager.hpp"
  #include "pinocchio/collision/parallel/geometry.hpp"
  #include "pinocchio/collision/pool/fwd.hpp"
  #include "pinocchio/collision/parallel/broadphase.hpp"

  #include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
  #include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
  #include <hpp/fcl/broadphase/broadphase_SSaP.h>
  #include <hpp/fcl/broadphase/broadphase_SaP.h>
  #include <hpp/fcl/broadphase/broadphase_bruteforce.h>
  #include <hpp/fcl/broadphase/broadphase_interval_tree.h>
  #include <hpp/fcl/broadphase/broadphase_spatialhash.h>
  #include <hpp/fcl/mesh_loader/loader.h>
#endif

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <benchmark/benchmark.h>

#include <iostream>

typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

struct ParallelFixture : benchmark::Fixture
{
  void SetUp(benchmark::State & st)
  {
    const auto BATCH_SIZE = st.range(0);
    const auto NUM_THREADS = st.range(1);

    model = MODEL;
    data = pinocchio::Data(model);

    const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
    qs = Eigen::MatrixXd(model.nq, BATCH_SIZE);
    vs = Eigen::MatrixXd(model.nv, BATCH_SIZE);
    as = Eigen::MatrixXd(model.nv, BATCH_SIZE);
    taus = Eigen::MatrixXd(model.nv, BATCH_SIZE);
    res = Eigen::MatrixXd(model.nv, BATCH_SIZE);

    for (Eigen::DenseIndex i = 0; i < BATCH_SIZE; ++i)
    {
      qs.col(i) = randomConfiguration(model, -qmax, qmax);
      vs.col(i) = Eigen::VectorXd::Random(model.nv);
      as.col(i) = Eigen::VectorXd::Random(model.nv);
      taus.col(i) = Eigen::VectorXd::Random(model.nv);
    }

    pool = std::make_unique<pinocchio::ModelPool>(model, static_cast<size_t>(NUM_THREADS));
  }

  void TearDown(benchmark::State &)
  {
  }

  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::MatrixXd qs;
  Eigen::MatrixXd vs;
  Eigen::MatrixXd as;
  Eigen::MatrixXd taus;
  Eigen::MatrixXd res;
  std::unique_ptr<pinocchio::ModelPool> pool;

  static pinocchio::Model MODEL;

  static void GlobalSetUp(const ExtraArgs &)
  {
    const std::string filename =
      PINOCCHIO_MODEL_DIR
      + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");

    pinocchio::urdf::buildModel(
      filename,
      pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar, pinocchio::context::Options>(),
      MODEL);

    std::cout << "nq = " << MODEL.nq << std::endl;
    std::cout << "nv = " << MODEL.nv << std::endl;
    std::cout << "name = " << MODEL.name << std::endl;
    std::cout << "--" << std::endl;
  }
};

pinocchio::Model ParallelFixture::MODEL;

static void MonoThreadCustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.)->ArgsProduct({{256}, {1}})->ArgNames({"BATCH_SIZE", "NUM_THREADS"});
}

static void MultiThreadCustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.)
    ->ArgsProduct({{256}, benchmark::CreateRange(1, omp_get_max_threads(), 2)})
    ->ArgNames({"BATCH_SIZE", "NUM_THREADS"})
    ->UseRealTime();
}

// RNEA

PINOCCHIO_DONT_INLINE static void rneaCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::MatrixXd::ColXpr & q,
  const Eigen::MatrixXd::ColXpr & v,
  const Eigen::MatrixXd::ColXpr & a,
  const Eigen::MatrixXd::ColXpr & r)
{
  PINOCCHIO_EIGEN_CONST_CAST(Eigen::MatrixXd::ColXpr, r) = pinocchio::rnea(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(ParallelFixture, RNEA)(benchmark::State & st)
{
  const auto BATCH_SIZE = st.range(0);
  for (auto _ : st)
  {
    for (auto i = 0; i < BATCH_SIZE; ++i)
    {
      rneaCall(model, data, qs.col(i), vs.col(i), as.col(i), res.col(i));
    }
  }
}
BENCHMARK_REGISTER_F(ParallelFixture, RNEA)->Apply(MonoThreadCustomArguments);

// RNEA_IN_PARALLEL

PINOCCHIO_DONT_INLINE static void rneaInParallelCall(
  size_t num_threads,
  pinocchio::ModelPool & pool,
  const Eigen::MatrixXd & qs,
  const Eigen::MatrixXd & vs,
  const Eigen::MatrixXd & as,
  const Eigen::MatrixXd & res)
{
  pinocchio::rneaInParallel(num_threads, pool, qs, vs, as, res);
}
BENCHMARK_DEFINE_F(ParallelFixture, RNEA_IN_PARALLEL)(benchmark::State & st)
{
  const auto NUM_THREADS = st.range(1);
  for (auto _ : st)
  {
    rneaInParallelCall(static_cast<size_t>(NUM_THREADS), *pool, qs, vs, as, res);
  }
}
BENCHMARK_REGISTER_F(ParallelFixture, RNEA_IN_PARALLEL)->Apply(MultiThreadCustomArguments);

// ABA

PINOCCHIO_DONT_INLINE static void abaCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::MatrixXd::ColXpr & q,
  const Eigen::MatrixXd::ColXpr & v,
  const Eigen::MatrixXd::ColXpr & taus,
  const Eigen::MatrixXd::ColXpr & r)
{
  PINOCCHIO_EIGEN_CONST_CAST(Eigen::MatrixXd::ColXpr, r) =
    pinocchio::aba(model, data, q, v, taus, pinocchio::Convention::WORLD);
}
BENCHMARK_DEFINE_F(ParallelFixture, ABA)(benchmark::State & st)
{
  const auto BATCH_SIZE = st.range(0);
  for (auto _ : st)
  {
    for (auto i = 0; i < BATCH_SIZE; ++i)
    {
      abaCall(model, data, qs.col(i), vs.col(i), taus.col(i), res.col(i));
    }
  }
}
BENCHMARK_REGISTER_F(ParallelFixture, ABA)->Apply(MonoThreadCustomArguments);

// ABA_IN_PARALLEL

PINOCCHIO_DONT_INLINE static void abaInParallelCall(
  size_t num_threads,
  pinocchio::ModelPool & pool,
  const Eigen::MatrixXd & qs,
  const Eigen::MatrixXd & vs,
  const Eigen::MatrixXd & taus,
  const Eigen::MatrixXd & res)
{
  pinocchio::abaInParallel(num_threads, pool, qs, vs, taus, res);
}
BENCHMARK_DEFINE_F(ParallelFixture, ABA_IN_PARALLEL)(benchmark::State & st)
{
  const auto NUM_THREADS = st.range(1);
  for (auto _ : st)
  {
    abaInParallelCall(static_cast<size_t>(NUM_THREADS), *pool, qs, vs, taus, res);
  }
}
BENCHMARK_REGISTER_F(ParallelFixture, ABA_IN_PARALLEL)->Apply(MultiThreadCustomArguments);

#ifdef PINOCCHIO_WITH_HPP_FCL

struct GeometryFixture : ParallelFixture
{
  void SetUp(benchmark::State & st)
  {
    ParallelFixture::SetUp(st);

    geometry_model = GEOMETRY_MODEL;
    geometry_data = pinocchio::GeometryData(geometry_model);

    const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
    q = randomConfiguration(model, -qmax, qmax);
  }

  void TearDown(benchmark::State & st)
  {
    ParallelFixture::TearDown(st);
  }

  pinocchio::GeometryModel geometry_model;
  pinocchio::GeometryData geometry_data;
  Eigen::VectorXd q;

  static pinocchio::GeometryModel GEOMETRY_MODEL;

  static void GlobalSetUp(const ExtraArgs & extra_args)
  {
    ParallelFixture::GlobalSetUp(extra_args);

    const std::string filename =
      PINOCCHIO_MODEL_DIR
      + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");
    const std::string package_path = PINOCCHIO_MODEL_DIR;
    hpp::fcl::MeshLoaderPtr mesh_loader = std::make_shared<hpp::fcl::CachedMeshLoader>();
    const std::string srdf_filename =
      PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
    std::vector<std::string> package_paths(1, package_path);
    pinocchio::urdf::buildGeom(
      MODEL, filename, pinocchio::COLLISION, GEOMETRY_MODEL, package_paths, mesh_loader);

    GEOMETRY_MODEL.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(MODEL, GEOMETRY_MODEL, srdf_filename, false);

    // Count active collision pair
    pinocchio::GeometryData geometry_data(GEOMETRY_MODEL);
    int num_active_collision_pairs = 0;
    for (size_t k = 0; k < geometry_data.activeCollisionPairs.size(); ++k)
    {
      if (geometry_data.activeCollisionPairs[k])
        num_active_collision_pairs++;
    }
    std::cout << "active collision pairs = " << num_active_collision_pairs << std::endl;
    std::cout << "---" << std::endl;
  }
};
pinocchio::GeometryModel GeometryFixture::GEOMETRY_MODEL;

// COMPUTE_COLLISIONS

PINOCCHIO_DONT_INLINE static void computeCollisionsCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::GeometryModel & geometry_model,
  pinocchio::GeometryData & geometry_data,
  const Eigen::VectorXd & q)
{
  pinocchio::computeCollisions(model, data, geometry_model, geometry_data, q);
}
BENCHMARK_DEFINE_F(GeometryFixture, COMPUTE_COLLISIONS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeCollisionsCall(model, data, geometry_model, geometry_data, q);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, COMPUTE_COLLISIONS)->Apply(MonoThreadCustomArguments);

// COMPUTE_COLLISIONS_IN_PARALLEL

PINOCCHIO_DONT_INLINE static void computeCollisionsInParallelCall(
  size_t num_threads,
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::GeometryModel & geometry_model,
  pinocchio::GeometryData & geometry_data,
  const Eigen::VectorXd & q)
{
  pinocchio::computeCollisionsInParallel(
    num_threads, model, data, geometry_model, geometry_data, q);
}
BENCHMARK_DEFINE_F(GeometryFixture, COMPUTE_COLLISIONS_IN_PARALLEL)(benchmark::State & st)
{
  const auto NUM_THREADS = st.range(1);
  for (auto _ : st)
  {
    computeCollisionsInParallelCall(
      static_cast<size_t>(NUM_THREADS), model, data, geometry_model, geometry_data, q);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, COMPUTE_COLLISIONS_IN_PARALLEL)
  ->Apply(MultiThreadCustomArguments);

// COMPUTE_COLLISIONS_BATCH

PINOCCHIO_DONT_INLINE static void computeCollisionsBatchCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::GeometryModel & geometry_model,
  pinocchio::GeometryData & geometry_data,
  const Eigen::MatrixXd::ColXpr & q)
{
  pinocchio::computeCollisions(model, data, geometry_model, geometry_data, q);
}
BENCHMARK_DEFINE_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH)(benchmark::State & st)
{
  const auto BATCH_SIZE = st.range(0);
  for (auto _ : st)
  {
    for (auto i = 0; i < BATCH_SIZE; ++i)
    {
      computeCollisionsBatchCall(model, data, geometry_model, geometry_data, qs.col(i));
    }
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH)->Apply(MonoThreadCustomArguments);

// COMPUTE_COLLISIONS_BATCH_IN_PARALLEL

PINOCCHIO_DONT_INLINE static void computeCollisionsBatchInParallelCall(
  size_t num_threads, pinocchio::GeometryPool & pool, const Eigen::MatrixXd & qs, VectorXb & res)
{
  pinocchio::computeCollisionsInParallel(num_threads, pool, qs, res);
}
BENCHMARK_DEFINE_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH_IN_PARALLEL)(benchmark::State & st)
{
  const auto BATCH_SIZE = st.range(0);
  const auto NUM_THREADS = st.range(1);

  pinocchio::GeometryPool geometry_pool(model, geometry_model, static_cast<size_t>(NUM_THREADS));
  VectorXb collision_res(BATCH_SIZE);
  collision_res.fill(false);
  for (auto _ : st)
  {
    computeCollisionsBatchInParallelCall(
      static_cast<size_t>(NUM_THREADS), geometry_pool, qs, collision_res);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH_IN_PARALLEL)
  ->Apply(MultiThreadCustomArguments);

// COMPUTE_COLLISIONS_BATCH_IN_PARALLEL_WITH_BROADPHASE

PINOCCHIO_DONT_INLINE static void computeCollisionsBatchInParallelWithBroadPhaseCall(
  size_t num_threads,
  pinocchio::BroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager, double> & pool,
  const Eigen::MatrixXd & qs,
  VectorXb & res)
{
  pinocchio::computeCollisionsInParallel(num_threads, pool, qs, res);
}
BENCHMARK_DEFINE_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH_IN_PARALLEL_WITH_BROADPHASE)(
  benchmark::State & st)
{
  const auto BATCH_SIZE = st.range(0);
  const auto NUM_THREADS = st.range(1);

  pinocchio::BroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager, double> pool(
    model, geometry_model, static_cast<size_t>(NUM_THREADS));
  VectorXb collision_res(BATCH_SIZE);
  collision_res.fill(false);
  for (auto _ : st)
  {
    computeCollisionsBatchInParallelWithBroadPhaseCall(
      static_cast<size_t>(NUM_THREADS), pool, qs, collision_res);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH_IN_PARALLEL_WITH_BROADPHASE)
  ->Apply(MultiThreadCustomArguments);

// COMPUTE_COLLISIONS_BATCH_IN_PARALLEL_WITH_TREE_BROADPHASE

PINOCCHIO_DONT_INLINE static void computeCollisionsBatchInParallelWithTreeBroadPhaseCall(
  size_t num_threads,
  pinocchio::TreeBroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager, double> & pool,
  const Eigen::MatrixXd & qs,
  VectorXb & res)
{
  pinocchio::computeCollisionsInParallel(num_threads, pool, qs, res);
}
BENCHMARK_DEFINE_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH_IN_PARALLEL_WITH_TREE_BROADPHASE)(
  benchmark::State & st)
{
  const auto BATCH_SIZE = st.range(0);
  const auto NUM_THREADS = st.range(1);

  pinocchio::TreeBroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager, double> pool(
    model, geometry_model, static_cast<size_t>(NUM_THREADS));
  VectorXb collision_res(BATCH_SIZE);
  collision_res.fill(false);
  for (auto _ : st)
  {
    computeCollisionsBatchInParallelWithTreeBroadPhaseCall(
      static_cast<size_t>(NUM_THREADS), pool, qs, collision_res);
  }
}
BENCHMARK_REGISTER_F(GeometryFixture, COMPUTE_COLLISIONS_BATCH_IN_PARALLEL_WITH_TREE_BROADPHASE)
  ->Apply(MultiThreadCustomArguments);

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL

#ifdef PINOCCHIO_WITH_HPP_FCL
PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(GeometryFixture::GlobalSetUp);
#else
PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(ParallelFixture::GlobalSetUp);
#endif
