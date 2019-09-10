//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;

template<typename JointModel>
void test_constraint_mimic(const JointModelBase<JointModel> & jmodel)
{
  typedef typename traits<JointModel>::JointDerived Joint;
  typedef typename traits<Joint>::Constraint_t ConstraintType;
  typedef typename traits<Joint>::JointDataDerived JointData;
  typedef ScaledConstraint<ConstraintType> ScaledConstraintType;
  
  JointData jdata = jmodel.createData();
  
  const double scaling_factor = 2.;
  ConstraintType constraint_ref(jdata.S), constraint_ref_shared(jdata.S);
  ScaledConstraintType scaled_constraint(constraint_ref_shared,scaling_factor);
  
  BOOST_CHECK(constraint_ref.nv() == scaled_constraint.nv());
  
  typedef typename JointModel::TangentVector_t TangentVector_t;
  TangentVector_t v = TangentVector_t::Random();
  
  Motion m = scaled_constraint * v;
  Motion m_ref = scaling_factor * (Motion)(constraint_ref * v);
  
  BOOST_CHECK(m.isApprox(m_ref));
  
  {
    SE3 M = SE3::Random();
    typename ScaledConstraintType::DenseBase S = M.act(scaled_constraint);
    typename ScaledConstraintType::DenseBase S_ref = scaling_factor * M.act(constraint_ref);
    
    BOOST_CHECK(S.isApprox(S_ref));
  }
  
  {
    typename ScaledConstraintType::DenseBase S = scaled_constraint.matrix();
    typename ScaledConstraintType::DenseBase S_ref = scaling_factor * constraint_ref.matrix();
    
    BOOST_CHECK(S.isApprox(S_ref));
  }
  
  {
    Motion v = Motion::Random();
    typename ScaledConstraintType::DenseBase S = v.cross(scaled_constraint);
    typename ScaledConstraintType::DenseBase S_ref = scaling_factor * v.cross(constraint_ref);
    
    BOOST_CHECK(S.isApprox(S_ref));
  }
  
  // Test transpose operations
  {
    const Eigen::DenseIndex dim = 20;
    const Matrix6x Fin = Matrix6x::Random(6,dim);
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
  
  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(0,0,0);
    
    test_constraint_mimic(jmodel);
  }
  
  void operator()(const JointModelBase<JointModelRevoluteUnaligned> &) const
  {
    JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_constraint_mimic(jmodel);
  }

  void operator()(const JointModelBase<JointModelPrismaticUnaligned> &) const
  {
    JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_constraint_mimic(jmodel);
  }
  
};

BOOST_AUTO_TEST_CASE(test_constraint)
{
  using namespace pinocchio;
  typedef boost::variant<
  JointModelRX, JointModelRY, JointModelRZ
  , JointModelRevoluteUnaligned
  , JointModelPX, JointModelPY, JointModelPZ
  , JointModelPrismaticUnaligned
  , JointModelRUBX, JointModelRUBY, JointModelRUBZ
  > Variant;
  
  boost::mpl::for_each<Variant::types>(TestJointConstraint());
}

template<typename JointModel>
void test_joint_mimic(const JointModelBase<JointModel> & jmodel)
{
  typedef typename traits<JointModel>::JointDerived Joint;
  typedef typename traits<Joint>::JointDataDerived JointData;
  
  JointData jdata = jmodel.createData();
  
  const double scaling_factor = 1.;
  const double offset = 0.;
  
  typedef JointMimic<Joint> JointMimicType;
  typedef typename traits<JointMimicType>::JointModelDerived JointModelMimicType;
  typedef typename traits<JointMimicType>::JointDataDerived JointDataMimicType;
  
  // test constructor
  JointModelMimicType jmodel_mimic(jmodel.derived(),scaling_factor,offset);
  JointDataMimicType jdata_mimic = jmodel_mimic.createData();
  
  BOOST_CHECK(jmodel_mimic.nq() == 0);
  BOOST_CHECK(jmodel_mimic.nv() == 0);
  
  BOOST_CHECK(jmodel_mimic.idx_q() == jmodel.idx_q());
  BOOST_CHECK(jmodel_mimic.idx_v() == jmodel.idx_v());
  
  BOOST_CHECK(jmodel_mimic.idx_q() == 0);
  BOOST_CHECK(jmodel_mimic.idx_v() == 0);
  
  typedef typename JointModelMimicType::ConfigVector_t ConfigVectorType;
  typedef typename LieGroup<JointModel>::type LieGroupType;
  ConfigVectorType q0 = LieGroupType().randomConfiguration(-ConfigVectorType::Ones(),ConfigVectorType::Ones());
  
  jmodel.calc(jdata,q0);
  jmodel_mimic.calc(jdata_mimic,q0);
  
  BOOST_CHECK(((SE3)jdata.M).isApprox((SE3)jdata_mimic.M()));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdata_mimic.S.matrix()));

  typedef typename JointModelMimicType::TangentVector_t TangentVectorType;
  
  q0 = LieGroupType().randomConfiguration(-ConfigVectorType::Ones(),ConfigVectorType::Ones());
  TangentVectorType v0 = TangentVectorType::Random();
  jmodel.calc(jdata,q0,v0);
  jmodel_mimic.calc(jdata_mimic,q0,v0);
  
  BOOST_CHECK(((SE3)jdata.M).isApprox((SE3)jdata_mimic.M()));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdata_mimic.S.matrix()));
  BOOST_CHECK(((Motion)jdata.v).isApprox((Motion)jdata_mimic.v()));
}

struct TestJointMimic
{
  
  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(0,0,0);
    
    test_joint_mimic(jmodel);
  }
  
  void operator()(const JointModelBase<JointModelRevoluteUnaligned> &) const
  {
    JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_mimic(jmodel);
  }

  void operator()(const JointModelBase<JointModelPrismaticUnaligned> &) const
  {
    JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_mimic(jmodel);
  }
  
};

BOOST_AUTO_TEST_CASE(test_joint)
{
  using namespace pinocchio;
  typedef boost::variant<
  JointModelRX, JointModelRY, JointModelRZ
  , JointModelRevoluteUnaligned
  , JointModelPX, JointModelPY, JointModelPZ
  , JointModelPrismaticUnaligned
  , JointModelRUBX, JointModelRUBY, JointModelRUBZ
  > Variant;
  
  boost::mpl::for_each<Variant::types>(TestJointMimic());
}

BOOST_AUTO_TEST_CASE(test_transform_linear_affine)
{
  typedef JointModelRX::ConfigVector_t ConfigVectorType;
  double scaling = 1., offset = 0.;
  
  ConfigVectorType q0 = ConfigVectorType::Random();
  ConfigVectorType q1;
  LinearAffineTransform::run(q0,scaling,offset,q1);
  BOOST_CHECK(q0 == q1);
  
  offset = 2.;
  LinearAffineTransform::run(ConfigVectorType::Zero(),scaling,offset,q1);
  BOOST_CHECK(q1 == ConfigVectorType::Constant(offset));
}

BOOST_AUTO_TEST_CASE(test_transform_linear_revolute)
{
  typedef JointModelRUBX::ConfigVector_t ConfigVectorType;
  double scaling = 1., offset = 0.;
  
  ConfigVectorType q0 = ConfigVectorType::Random().normalized();
  ConfigVectorType q1;
  UnboundedRevoluteAffineTransform::run(q0,scaling,offset,q1);
  BOOST_CHECK(q0.isApprox(q1));
  
  offset = 2.;
  UnboundedRevoluteAffineTransform::run(ConfigVectorType::Zero(),scaling,offset,q1);
  BOOST_CHECK(q1 == ConfigVectorType(math::cos(offset),math::sin(offset)));
}

BOOST_AUTO_TEST_CASE(test_joint_generic_cast)
{
  JointModelRX jmodel_ref;
  jmodel_ref.setIndexes(1,2,3);
  
  JointModelMimic<JointModelRX> jmodel(jmodel_ref,2.,1.);
  jmodel.setIndexes(1,-1,-1);
  
  BOOST_CHECK(jmodel.id() == jmodel_ref.id());
  BOOST_CHECK(jmodel.idx_q() == jmodel_ref.idx_q());
  BOOST_CHECK(jmodel.idx_v() == jmodel_ref.idx_v());
  
  JointModel jmodel_generic(jmodel);
  jmodel_generic.setIndexes(1,-2,-2);
  
  BOOST_CHECK(jmodel_generic.id() == jmodel_ref.id());
}
BOOST_AUTO_TEST_SUITE_END()
