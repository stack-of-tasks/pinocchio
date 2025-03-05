//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <Eigen/src/Core/util/Macros.h>
#include <benchmark/benchmark.h>

#include <iostream>

using Scalar = double;
constexpr int Options = 0;
using JointCollectionDefault = pinocchio::JointCollectionDefaultTpl<Scalar, Options>;
using Model = pinocchio::ModelTpl<Scalar, Options, pinocchio::JointCollectionDefaultTpl>;
using Data = pinocchio::DataTpl<Scalar, Options, pinocchio::JointCollectionDefaultTpl>;
using JointModelFreeFlyer = pinocchio::JointModelFreeFlyerTpl<Scalar>;

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepUnaryVisit
  : fusion::
      JointUnaryVisitorBase<EmptyForwardStepUnaryVisit<Scalar, Options, JointCollectionTpl>, int>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    // Function should not be empty to allow EIGEN_DONT_INLINE to work
    template<typename JointModel>
    EIGEN_DONT_INLINE static int
    algo(const JointModelBase<JointModel> &, JointDataBase<typename JointModel::JointDataDerived> &)
    {
      return 0;
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  static void emptyForwardPassUnaryVisit(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepUnaryVisit<Scalar, Options, JointCollectionTpl> Algo;

    int sum = 0;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      sum += Algo::run(model.joints[i], data.joints[i]);
      benchmark::DoNotOptimize(sum);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepUnaryVisitNoData
  : fusion::JointUnaryVisitorBase<
      EmptyForwardStepUnaryVisitNoData<Scalar, Options, JointCollectionTpl>,
      int>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    // Function should not be empty to allow EIGEN_DONT_INLINE to work
    template<typename JointModel>
    EIGEN_DONT_INLINE static int algo(const JointModelBase<JointModel> &)
    {
      return 0;
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  static void emptyForwardPassUnaryVisitNoData(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    PINOCCHIO_UNUSED_VARIABLE(data);
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepUnaryVisitNoData<Scalar, Options, JointCollectionTpl> Algo;

    int sum = 0;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      sum += Algo::run(model.joints[i]);
      benchmark::DoNotOptimize(sum);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepBinaryVisit
  : fusion::
      JointBinaryVisitorBase<EmptyForwardStepBinaryVisit<Scalar, Options, JointCollectionTpl>, int>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    // Function should not be empty to allow EIGEN_DONT_INLINE to work
    template<typename JointModel1, typename JointModel2>
    EIGEN_DONT_INLINE static int algo(
      const JointModelBase<JointModel1> &,
      const JointModelBase<JointModel2> &,
      JointDataBase<typename JointModel1::JointDataDerived> &,
      JointDataBase<typename JointModel2::JointDataDerived> &)
    {
      return 0;
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  static void emptyForwardPassBinaryVisit(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepBinaryVisit<Scalar, Options, JointCollectionTpl> Algo;

    int sum = 0;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      sum += Algo::run(model.joints[i], model.joints[i], data.joints[i], data.joints[i]);
      benchmark::DoNotOptimize(sum);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepBinaryVisitNoData
  : fusion::JointBinaryVisitorBase<
      EmptyForwardStepBinaryVisitNoData<Scalar, Options, JointCollectionTpl>,
      int>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    // Function should not be empty to allow EIGEN_DONT_INLINE to work
    template<typename JointModel1, typename JointModel2>
    EIGEN_DONT_INLINE static int
    algo(const JointModelBase<JointModel1> &, const JointModelBase<JointModel2> &)
    {
      return 0;
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  static void emptyForwardPassBinaryVisitNoData(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    PINOCCHIO_UNUSED_VARIABLE(data);
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepBinaryVisitNoData<Scalar, Options, JointCollectionTpl> Algo;

    int sum = 0;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      sum += Algo::run(model.joints[i], model.joints[i]);
      benchmark::DoNotOptimize(sum);
    }
  }
} // namespace pinocchio

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

bool WITH_FF = true;
std::string MODEL_FILENAME(PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf"));

class ModelFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State &)
  {
    model = pinocchio::Model();
    if (MODEL_FILENAME == "HS")
      pinocchio::buildModels::humanoidRandom(model, true);
    else if (WITH_FF)
      pinocchio::urdf::buildModel(MODEL_FILENAME, JointModelFreeFlyer(), model);
    else
      pinocchio::urdf::buildModel(MODEL_FILENAME, model);

    data = pinocchio::Data(model);
    const Eigen::VectorXd qmax(Eigen::VectorXd::Ones(model.nq));
    q = randomConfiguration(model, -qmax, qmax);
    v = Eigen::VectorXd::Random(model.nv);
    a = Eigen::VectorXd::Random(model.nv);
    tau = Eigen::VectorXd::Random(model.nv);
  }

  void TearDown(benchmark::State &)
  {
  }

  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd a;
  Eigen::VectorXd tau;
};

// RNEA

EIGEN_DONT_INLINE static void rneaCall(
  const Model & model,
  Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::rnea(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(ModelFixture, RNEA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    rneaCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, RNEA)->Apply(CustomArguments);

// nonLinearEffects

EIGEN_DONT_INLINE static void nonLinearEffectsCall(
  const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::nonLinearEffects(model, data, q, v);
}
BENCHMARK_DEFINE_F(ModelFixture, NLE)(benchmark::State & st)
{
  for (auto _ : st)
  {
    nonLinearEffectsCall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, NLE)->Apply(CustomArguments);

// nonLinearEffects via RNEA

EIGEN_DONT_INLINE static void nonLinearEffectsViaRNEACall(
  const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::rnea(model, data, q, v, Eigen::VectorXd::Zero(model.nv));
}
BENCHMARK_DEFINE_F(ModelFixture, NLE_via_RNEA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    nonLinearEffectsViaRNEACall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, NLE_via_RNEA)->Apply(CustomArguments);

// CRBA Local

EIGEN_DONT_INLINE static void
crbaLocalCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::LOCAL);
}
BENCHMARK_DEFINE_F(ModelFixture, CRBA_LOCAL)(benchmark::State & st)
{
  for (auto _ : st)
  {
    crbaLocalCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, CRBA_LOCAL)->Apply(CustomArguments);

// CRBA World

EIGEN_DONT_INLINE static void
crbaWorldCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
}
BENCHMARK_DEFINE_F(ModelFixture, CRBA_WORLD)(benchmark::State & st)
{
  for (auto _ : st)
  {
    crbaWorldCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, CRBA_WORLD)->Apply(CustomArguments);

// computeAllTerms

EIGEN_DONT_INLINE static void computeAllTermsCall(
  const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::computeAllTerms(model, data, q, v);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_ALL_TERMS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeAllTermsCall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_ALL_TERMS)->Apply(CustomArguments);

// choleskyDecompose

EIGEN_DONT_INLINE static void choleskyDecomposeCall(const Model & model, Data & data)
{
  pinocchio::cholesky::decompose(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, CHOLESKY_DECOMPOSE)(benchmark::State & st)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  for (auto _ : st)
  {
    choleskyDecomposeCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, CHOLESKY_DECOMPOSE)->Apply(CustomArguments);

// Dense choleskyDecompose

EIGEN_DONT_INLINE static void
denseCholeksyDecomposeCall(Eigen::LDLT<Eigen::MatrixXd> & M_ldlt, const Data & data)
{
  M_ldlt.compute(data.M);
}
BENCHMARK_DEFINE_F(ModelFixture, DENSE_CHOLESKY_DECOMPOSE)(benchmark::State & st)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  Eigen::LDLT<Eigen::MatrixXd> M_ldlt(data.M);
  for (auto _ : st)
  {
    denseCholeksyDecomposeCall(M_ldlt, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, DENSE_CHOLESKY_DECOMPOSE)->Apply(CustomArguments);

// computeJointJacobians

EIGEN_DONT_INLINE static void
computeJointJacobiansCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::computeJointJacobians(model, data, q);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_JOINT_JACOBIANS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeJointJacobiansCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_JOINT_JACOBIANS)->Apply(CustomArguments);

// computeJointJacobiansTimeVariation

EIGEN_DONT_INLINE static void computeJointJacobiansTimeVariationCall(
  const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_JOINT_JACOBIANS_TIME_VARIATION)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeJointJacobiansTimeVariationCall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_JOINT_JACOBIANS_TIME_VARIATION)->Apply(CustomArguments);

// jacobianCenterOfMass

EIGEN_DONT_INLINE static void
jacobianCenterOfMassCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::jacobianCenterOfMass(model, data, q, true);
}
BENCHMARK_DEFINE_F(ModelFixture, JACOBIAN_CENTER_OF_MASS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    jacobianCenterOfMassCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, JACOBIAN_CENTER_OF_MASS)->Apply(CustomArguments);

// centerOfMass

EIGEN_DONT_INLINE static void centerOfMassCall(
  const Model & model,
  Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::centerOfMass(model, data, q, v, a, true);
}
BENCHMARK_DEFINE_F(ModelFixture, CENTER_OF_MASS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    centerOfMassCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, CENTER_OF_MASS)->Apply(CustomArguments);

// forwardKinematicsQ

EIGEN_DONT_INLINE static void
forwardKinematicsQCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model, data, q);
}
BENCHMARK_DEFINE_F(ModelFixture, FORWARD_KINEMATICS_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    forwardKinematicsQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, FORWARD_KINEMATICS_Q)->Apply(CustomArguments);

// forwardKinematicsQV

EIGEN_DONT_INLINE static void forwardKinematicsQVCall(
  const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::forwardKinematics(model, data, q, v);
}
BENCHMARK_DEFINE_F(ModelFixture, FORWARD_KINEMATICS_Q_V)(benchmark::State & st)
{
  for (auto _ : st)
  {
    forwardKinematicsQVCall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, FORWARD_KINEMATICS_Q_V)->Apply(CustomArguments);

// forwardKinematicsQVA

EIGEN_DONT_INLINE static void forwardKinematicsQVACall(
  const Model & model,
  Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::forwardKinematics(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(ModelFixture, FORWARD_KINEMATICS_Q_V_A)(benchmark::State & st)
{
  for (auto _ : st)
  {
    forwardKinematicsQVACall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, FORWARD_KINEMATICS_Q_V_A)->Apply(CustomArguments);

// framesForwardKinematics

EIGEN_DONT_INLINE static void
framesForwardKinematicsCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::framesForwardKinematics(model, data, q);
}
BENCHMARK_DEFINE_F(ModelFixture, FRAME_FORWARD_KINEMATICS)(benchmark::State & st)
{
  for (auto _ : st)
  {
    framesForwardKinematicsCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, FRAME_FORWARD_KINEMATICS)->Apply(CustomArguments);

// updateFramePlacements

EIGEN_DONT_INLINE static void updateFramePlacementsCall(const Model & model, Data & data)
{
  pinocchio::updateFramePlacements(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, UPDATE_FRAME_PLACEMENTS)(benchmark::State & st)
{
  forwardKinematics(model, data, q);
  for (auto _ : st)
  {
    updateFramePlacementsCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, UPDATE_FRAME_PLACEMENTS)->Apply(CustomArguments);

// CCRBA

EIGEN_DONT_INLINE static void
ccrbaCall(const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::ccrba(model, data, q, v);
}
BENCHMARK_DEFINE_F(ModelFixture, CCRBA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    ccrbaCall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, CCRBA)->Apply(CustomArguments);

// ABA Local

EIGEN_DONT_INLINE static void abaLocalCall(
  const Model & model,
  Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::aba(model, data, q, v, a, pinocchio::Convention::LOCAL);
}
BENCHMARK_DEFINE_F(ModelFixture, ABA_LOCAL)(benchmark::State & st)
{
  for (auto _ : st)
  {
    abaLocalCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, ABA_LOCAL)->Apply(CustomArguments);

// ABA World

EIGEN_DONT_INLINE static void abaWorldCall(
  const Model & model,
  Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::aba(model, data, q, v, a, pinocchio::Convention::WORLD);
}
BENCHMARK_DEFINE_F(ModelFixture, ABA_WORLD)(benchmark::State & st)
{
  for (auto _ : st)
  {
    abaWorldCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, ABA_WORLD)->Apply(CustomArguments);

// computeCoriolisMatrix

EIGEN_DONT_INLINE static void computeCoriolisMatrixCall(
  const Model & model, Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::computeCoriolisMatrix(model, data, q, v);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_CORIOLIS_MATRIX)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeCoriolisMatrixCall(model, data, q, v);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_CORIOLIS_MATRIX)->Apply(CustomArguments);

// computeMinverseQ

EIGEN_DONT_INLINE static void
computeMinverseQCall(const Model & model, Data & data, const Eigen::VectorXd & q)
{
  pinocchio::computeMinverse(model, data, q);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_M_INVERSE_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeMinverseQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_M_INVERSE_Q)->Apply(CustomArguments);

// computeMinverse

EIGEN_DONT_INLINE static void computeMinverseCall(const Model & model, Data & data)
{
  pinocchio::computeMinverse(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_M_INVERSE)(benchmark::State & st)
{
  pinocchio::aba(model, data, q, v, a, pinocchio::Convention::WORLD);
  for (auto _ : st)
  {
    computeMinverseCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_M_INVERSE)->Apply(CustomArguments);

// emptyForwardPassUnaryVisit

EIGEN_DONT_INLINE static void emptyForwardPassUnaryVisitCall(const Model & model, Data & data)
{
  pinocchio::emptyForwardPassUnaryVisit(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, EMPTY_FORWARD_PASS_UNARY_VISIT)(benchmark::State & st)
{
  for (auto _ : st)
  {
    emptyForwardPassUnaryVisitCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, EMPTY_FORWARD_PASS_UNARY_VISIT)->Apply(CustomArguments);

// emptyForwardPassUnaryVisitNoData

EIGEN_DONT_INLINE static void emptyForwardPassUnaryVisitNoDataCall(const Model & model, Data & data)
{
  pinocchio::emptyForwardPassUnaryVisitNoData(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, EMPTY_FORWARD_PASS_UNARY_VISIT_NO_DATA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    emptyForwardPassUnaryVisitNoDataCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, EMPTY_FORWARD_PASS_UNARY_VISIT_NO_DATA)->Apply(CustomArguments);

// emptyForwardPassBinaryVisit

EIGEN_DONT_INLINE static void emptyForwardPassBinaryVisitCall(const Model & model, Data & data)
{
  pinocchio::emptyForwardPassBinaryVisit(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, EMPTY_FORWARD_PASS_BINARY_VISIT)(benchmark::State & st)
{
  for (auto _ : st)
  {
    emptyForwardPassBinaryVisitCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, EMPTY_FORWARD_PASS_BINARY_VISIT)->Apply(CustomArguments);

// emptyForwardPassBinaryVisitNoData

EIGEN_DONT_INLINE static void
emptyForwardPassBinaryVisitNoDataCall(const Model & model, Data & data)
{
  pinocchio::emptyForwardPassBinaryVisitNoData(model, data);
}
BENCHMARK_DEFINE_F(ModelFixture, EMPTY_FORWARD_PASS_BINARY_VISIT_NO_DATA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    emptyForwardPassBinaryVisitNoDataCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, EMPTY_FORWARD_PASS_BINARY_VISIT_NO_DATA)->Apply(CustomArguments);

BENCHMARK_MAIN();
