//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6x;

template<typename JointModel>
void test_constraint_mimic(const JointModelBase<JointModel> & jmodel)
{
  typedef typename traits<JointModel>::JointDerived Joint;
  typedef typename traits<Joint>::Constraint_t ConstraintType;
  typedef typename traits<Joint>::JointDataDerived JointData;
  typedef ScaledJointMotionSubspaceTpl<double, 0, JointModel::NVExtended> ScaledConstraint;
  typedef JointMotionSubspaceTpl<Eigen::Dynamic, double, 0> ConstraintRef;

  JointData jdata = jmodel.createData();

  const double scaling_factor = 2.;
  ConstraintRef constraint_ref(jdata.S.matrix()), constraint_ref_shared(jdata.S.matrix());
  ScaledConstraint scaled_constraint(constraint_ref_shared, scaling_factor);

  BOOST_CHECK(constraint_ref.nv() == scaled_constraint.nv());

  typedef typename JointModel::TangentVector_t TangentVector_t;
  TangentVector_t v = TangentVector_t::Random();

  Motion m = scaled_constraint * v;
  Motion m_ref = scaling_factor * (Motion)(constraint_ref * v);

  BOOST_CHECK(m.isApprox(m_ref));

  {
    SE3 M = SE3::Random();
    typename ScaledConstraint::DenseBase S = M.act(scaled_constraint);
    typename ScaledConstraint::DenseBase S_ref = scaling_factor * M.act(constraint_ref);

    BOOST_CHECK(S.isApprox(S_ref));
  }

  {
    typename ScaledConstraint::DenseBase S = scaled_constraint.matrix();
    typename ScaledConstraint::DenseBase S_ref = scaling_factor * constraint_ref.matrix();

    BOOST_CHECK(S.isApprox(S_ref));
  }

  {
    Motion v = Motion::Random();
    typename ScaledConstraint::DenseBase S = v.cross(scaled_constraint);
    typename ScaledConstraint::DenseBase S_ref = scaling_factor * v.cross(constraint_ref);

    BOOST_CHECK(S.isApprox(S_ref));
  }

  // Test transpose operations
  {
    const Eigen::DenseIndex dim = ScaledConstraint::MaxDim;
    const Matrix6x Fin = Matrix6x::Random(6, dim);
    Eigen::MatrixXd Fout = scaled_constraint.transpose() * Fin;
    Eigen::MatrixXd Fout_ref = scaling_factor * (constraint_ref.transpose() * Fin);
    BOOST_CHECK(Fout.isApprox(Fout_ref));

    Force force_in(Force::Random());
    Eigen::MatrixXd Stf = (scaled_constraint.transpose() * force_in);
    Eigen::MatrixXd Stf_ref = scaling_factor * (constraint_ref.transpose() * force_in);
    BOOST_CHECK(Stf_ref.isApprox(Stf));
  }

  // CRBA Y*S
  {
    Inertia Y = Inertia::Random();
    Eigen::MatrixXd YS = Y * scaled_constraint;
    Eigen::MatrixXd YS_ref = scaling_factor * (Y * constraint_ref);

    BOOST_CHECK(YS.isApprox(YS_ref));
  }
}

struct TestJointConstraint
{

  template<typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(1, 0, 0, 0);

    test_constraint_mimic(jmodel);
  }

  void operator()(const JointModelBase<JointModelRevoluteUnaligned> &) const
  {
    JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(1, 0, 0, 0);

    test_constraint_mimic(jmodel);
  }

  void operator()(const JointModelBase<JointModelPrismaticUnaligned> &) const
  {
    JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(1, 0, 0, 0);

    test_constraint_mimic(jmodel);
  }
};

BOOST_AUTO_TEST_CASE(test_constraint)
{
  using namespace pinocchio;
  typedef boost::variant<
    JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned, JointModelPX,
    JointModelPY, JointModelPZ, JointModelPrismaticUnaligned>
    Variant;

  boost::mpl::for_each<Variant::types>(TestJointConstraint());
}

template<typename JointModel, typename MimicConfigurationTransform, bool MimicIdentity>
void test_joint_mimic(const JointModelBase<JointModel> & jmodel)
{
  typedef typename traits<JointModel>::JointDerived Joint;
  typedef typename traits<Joint>::JointDataDerived JointData;

  JointData jdata = jmodel.createData();

  const double scaling_factor = MimicIdentity ? 1. : 2.5;
  const double offset = MimicIdentity ? 0 : 0.75;

  // test constructor
  JointModelMimic jmodel_mimic(jmodel.derived(), scaling_factor, offset);
  JointDataMimic jdata_mimic = jmodel_mimic.createData();

  // Non-const ref accessors trigger asserts, usefull const ref to call const ref accessors...
  const JointDataMimic & jdata_mimic_const_ref{jdata_mimic};

  BOOST_CHECK(jmodel_mimic.nq() == 0);
  BOOST_CHECK(jmodel_mimic.nv() == 0);

  BOOST_CHECK(jmodel_mimic.idx_q() == jmodel.idx_q());
  BOOST_CHECK(jmodel_mimic.idx_v() == jmodel.idx_v());

  typedef typename JointModel::ConfigVector_t ConfigVectorType;
  typedef typename LieGroup<JointModel>::type LieGroupType;
  ConfigVectorType q0 =
    LieGroupType().randomConfiguration(-ConfigVectorType::Ones(), ConfigVectorType::Ones());
  ConfigVectorType q0_mimic;
  MimicConfigurationTransform::run(q0, scaling_factor, offset, q0_mimic);

  jmodel.calc(jdata, q0_mimic);
  jmodel_mimic.calc(jdata_mimic, q0);

  BOOST_CHECK(((SE3)jdata.M).isApprox((SE3)jdata_mimic.M()));
  BOOST_CHECK((scaling_factor * jdata.S.matrix()).isApprox(jdata_mimic.S.matrix()));

  typedef typename JointModel::TangentVector_t TangentVectorType;

  q0 = LieGroupType().randomConfiguration(-ConfigVectorType::Ones(), ConfigVectorType::Ones());
  MimicConfigurationTransform::run(q0, scaling_factor, offset, q0_mimic);
  TangentVectorType v0 = TangentVectorType::Random();
  TangentVectorType v0_mimic = v0 * scaling_factor;
  jmodel.calc(jdata, q0_mimic, v0_mimic);
  jmodel_mimic.calc(jdata_mimic, q0, v0);

  BOOST_CHECK(((SE3)jdata.M).isApprox((SE3)jdata_mimic.M()));
  BOOST_CHECK((scaling_factor * jdata.S.matrix()).isApprox(jdata_mimic.S.matrix()));
  BOOST_CHECK(((Motion)jdata.v).isApprox((Motion)jdata_mimic.v()));
}

template<typename MimicConfigurationTransform, bool MimicIdentity>
struct TestJointMimic
{

  template<typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(1, 0, 0, 0);

    test_joint_mimic<JointModel, MimicConfigurationTransform, MimicIdentity>(jmodel);
  }

  void operator()(const JointModelBase<JointModelRevoluteUnaligned> &) const
  {
    JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(1, 0, 0, 0);

    test_joint_mimic<JointModelRevoluteUnaligned, MimicConfigurationTransform, MimicIdentity>(
      jmodel);
  }

  void operator()(const JointModelBase<JointModelPrismaticUnaligned> &) const
  {
    JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(1, 0, 0, 0);

    test_joint_mimic<JointModelPrismaticUnaligned, MimicConfigurationTransform, MimicIdentity>(
      jmodel);
  }
};

BOOST_AUTO_TEST_CASE(test_joint)
{
  using namespace pinocchio;
  typedef boost::variant<
    JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned, JointModelPX,
    JointModelPY, JointModelPZ, JointModelPrismaticUnaligned>
    VariantLinear;

  typedef boost::variant<JointModelRUBX, JointModelRUBY, JointModelRUBZ> VariantUnboundedRevolute;

  // Test specific transforms for non trivial affine values
  boost::mpl::for_each<VariantLinear::types>(TestJointMimic<LinearAffineTransform, false>());
  boost::mpl::for_each<VariantUnboundedRevolute::types>(
    TestJointMimic<UnboundedRevoluteAffineTransform, false>());
}

BOOST_AUTO_TEST_CASE(test_transform_linear_affine)
{
  typedef JointModelRX::ConfigVector_t ConfigVectorType;
  double scaling = 1., offset = 0.;

  ConfigVectorType q0 = ConfigVectorType::Random();
  ConfigVectorType q1;
  LinearAffineTransform::run(q0, scaling, offset, q1);
  BOOST_CHECK(q0 == q1);

  scaling = 2.5;
  offset = 1.5;
  LinearAffineTransform::run(ConfigVectorType::Zero(), scaling, offset, q1);
  BOOST_CHECK(q1 == ConfigVectorType::Constant(offset));

  LinearAffineTransform::run(q0, scaling, offset, q1);
  BOOST_CHECK((scaling * q0 + ConfigVectorType::Ones() * offset) == q1);
}

BOOST_AUTO_TEST_CASE(test_transform_linear_revolute_unbounded)
{
  typedef JointModelRUBX::ConfigVector_t ConfigVectorType;
  double scaling = 1., offset = 0.;

  ConfigVectorType q0 = ConfigVectorType::Random().normalized();
  ConfigVectorType q1;
  UnboundedRevoluteAffineTransform::run(q0, scaling, offset, q1);
  BOOST_CHECK(q0.isApprox(q1));

  scaling = 2.5;
  offset = 1.5;
  UnboundedRevoluteAffineTransform::run(q0, scaling, offset, q1);
  const double theta = atan2(q0[1], q0[0]);
  BOOST_CHECK(
    q1
    == ConfigVectorType(math::cos(theta * scaling + offset), math::sin(theta * scaling + offset)));
}

BOOST_AUTO_TEST_CASE(test_joint_generic_cast)
{
  JointModelRX jmodel_ref;
  jmodel_ref.setIndexes(1, 2, 3, 3);

  JointModelMimic jmodel(jmodel_ref, 2., 1.);
  jmodel.setIndexes(2, -1, -1, 3);

  BOOST_CHECK(jmodel.idx_q() == jmodel_ref.idx_q());
  BOOST_CHECK(jmodel.idx_v() == jmodel_ref.idx_v());

  JointModel jmodel_generic(jmodel);
  jmodel_generic.setIndexes(2, -2, -2, 3);

  BOOST_CHECK(jmodel_generic.id() == jmodel.id());
}
BOOST_AUTO_TEST_SUITE_END()
