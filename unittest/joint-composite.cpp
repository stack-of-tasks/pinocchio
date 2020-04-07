//
// Copyright (c) 2016 CNRS
//

#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;
using namespace Eigen;

template<typename JointModel>
void test_joint_methods(const JointModelBase<JointModel> & jmodel, JointModelComposite & jmodel_composite);

template<typename JointModel>
void test_joint_methods(const JointModelBase<JointModel> & jmodel)
{
  JointModelComposite jmodel_composite(jmodel);
  test_joint_methods(jmodel,jmodel_composite);
}

template<typename JointModel>
void test_joint_methods(const JointModelBase<JointModel> & jmodel, JointModelComposite & jmodel_composite)
{
  typedef typename JointModelBase<JointModel>::JointDataDerived JointData;
  
  JointData jdata = jmodel.createData();
  JointDataComposite jdata_composite = jmodel_composite.createData();
  
  jmodel_composite.setIndexes(jmodel.id(), jmodel.idx_q(), jmodel.idx_v());
  
  typedef typename JointModel::ConfigVector_t ConfigVector_t;
  typedef typename JointModel::TangentVector_t TangentVector_t;
  typedef typename LieGroup<JointModel>::type LieGroupType;
  
  typedef Eigen::Matrix<typename ConfigVector_t::Scalar, Eigen::Dynamic, 1, ConfigVector_t::Options> DynConfigVectorType;
  typedef DynConfigVectorType DynTangentVectorType;
  
  ConfigVector_t ql(ConfigVector_t::Constant(jmodel.nq(),-M_PI));
  ConfigVector_t qu(ConfigVector_t::Constant(jmodel.nq(),M_PI));
  DynConfigVectorType q = LieGroupType().randomConfiguration(ql,qu);

  BOOST_CHECK(jmodel.nv() == jmodel_composite.nv());
  BOOST_CHECK(jmodel.nq() == jmodel_composite.nq());
  
  jmodel.calc(jdata,q);
  jmodel_composite.calc(jdata_composite,q);
  
  BOOST_CHECK(jdata_composite.M.isApprox((SE3)jdata.M));
  BOOST_CHECK(jdata_composite.S.matrix().isApprox(jdata.S.matrix()));
  
  q = LieGroupType().randomConfiguration(ql,qu);
  DynTangentVectorType v = TangentVector_t::Random(jmodel.nv());
  jmodel.calc(jdata,q,v);
  jmodel_composite.calc(jdata_composite,q,v);
  
  BOOST_CHECK(jdata_composite.M.isApprox((SE3)jdata.M));
  BOOST_CHECK(jdata_composite.S.matrix().isApprox(jdata.S.matrix()));
  BOOST_CHECK(jdata_composite.v.isApprox((Motion)jdata.v));
  BOOST_CHECK(jdata_composite.c.isApprox((Motion)jdata.c));
  
  // TODO: Not yet checked
//  {
//    VectorXd q1(jmodel.random());
//    jmodel.normalize(q1);
//    VectorXd q2(jmodel.random());
//    jmodel.normalize(q2);
//    VectorXd v(VectorXd::Random(jmodel.nv()));
//    
//    BOOST_CHECK(jmodel_composite.integrate(q1,v).isApprox(jmodel.integrate(q1,v)));
//
//    TangentVector_t v1 = jmodel_composite.difference(q1,q2);
//    TangentVector_t v2 = jmodel.difference(q1,q2);
//
//    BOOST_CHECK(v1.isApprox(v2));
//    
//    const double alpha = 0.2;
//    BOOST_CHECK(jmodel_composite.interpolate(q1,q2,alpha).isApprox(jmodel.interpolate(q1,q2,alpha)));
//    BOOST_CHECK(math::fabs(jmodel_composite.distance(q1,q2)-jmodel.distance(q1,q2))<= NumTraits<double>::dummy_precision());
//  }
  
  Inertia::Matrix6 I1(Inertia::Random().matrix());
  Inertia::Matrix6 I2 = I1;

  const Eigen::VectorXd armature = Eigen::VectorXd::Random(jmodel.nv()) + Eigen::VectorXd::Ones(jmodel.nv());
  jmodel.calc_aba(jdata,armature,I1,true);
  jmodel_composite.calc_aba(jdata_composite,armature,I2,true);

  double prec = 1e-10; // higher tolerance to errors due to possible numerical imprecisions
  
  BOOST_CHECK(jdata.U.isApprox(jdata_composite.U,prec));
  BOOST_CHECK(jdata.Dinv.isApprox(jdata_composite.Dinv,prec));
  BOOST_CHECK(jdata.UDinv.isApprox(jdata_composite.UDinv,prec));

  // Checking the inertia was correctly updated
  // We use isApprox as usual, except for the freeflyer,
  // where the correct result is exacly zero and isApprox would fail.
  // Only for this single case, we use the infinity norm of the difference
  if(jmodel.shortname() == "JointModelFreeFlyer")
    BOOST_CHECK((I1-I2).lpNorm<Eigen::Infinity>() < prec);
  else
    BOOST_CHECK(I1.isApprox(I2,prec));
}

struct TestJointComposite{

  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);    
  }

//  void operator()(const JointModelBase<JointModelComposite> &) const
//  {
//    JointModelComposite jmodel_composite;
//    jmodel_composite.addJoint(pinocchio::JointModelRX());
//    jmodel_composite.addJoint(pinocchio::JointModelRY());
//    jmodel_composite.setIndexes(0,0,0);
//
//    test_joint_methods(jmodel_composite);
//  }

  void operator()(const JointModelBase<JointModelRevoluteUnaligned> &) const
  {
    JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

  void operator()(const JointModelBase<JointModelPrismaticUnaligned> &) const
  {
    JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

};

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_basic)
{
  typedef boost::variant< JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned
  , JointModelSpherical, JointModelSphericalZYX
  , JointModelPX, JointModelPY, JointModelPZ
  , JointModelPrismaticUnaligned
  , JointModelFreeFlyer
  , JointModelPlanar
  , JointModelTranslation
  , JointModelRUBX, JointModelRUBY, JointModelRUBZ
  > Variant;
  
  boost::mpl::for_each<Variant::types>(TestJointComposite());
}

BOOST_AUTO_TEST_CASE(chain)
{
  JointModelComposite jmodel_composite;
  jmodel_composite.addJoint(JointModelRZ()).addJoint(JointModelRY(),SE3::Random()).addJoint(JointModelRX());
  BOOST_CHECK_MESSAGE( jmodel_composite.nq() == 3, "Chain did not work");
  BOOST_CHECK_MESSAGE( jmodel_composite.nv() == 3, "Chain did not work");
  BOOST_CHECK_MESSAGE( jmodel_composite.njoints == 3, "Chain did not work");
}

BOOST_AUTO_TEST_CASE(vsZYX)
{
  JointModelSphericalZYX jmodel_spherical;
  jmodel_spherical.setIndexes(0,0,0);
  
  JointModelComposite jmodel_composite((JointModelRZ()));
  jmodel_composite.addJoint(JointModelRY());
  jmodel_composite.addJoint(JointModelRX());
  
  test_joint_methods(jmodel_spherical, jmodel_composite);
}

BOOST_AUTO_TEST_CASE(vsTranslation)
{
  JointModelTranslation jmodel_translation;
  jmodel_translation.setIndexes(0,0,0);
  
  JointModelComposite jmodel_composite((JointModelPX()));
  jmodel_composite.addJoint(JointModelPY());
  jmodel_composite.addJoint(JointModelPZ());
  
  test_joint_methods(jmodel_translation, jmodel_composite);
}

BOOST_AUTO_TEST_CASE (test_recursive_variant)
{
  /// Create joint composite with two joints,
  JointModelComposite jmodel_composite_two_rx((JointModelRX()));
  jmodel_composite_two_rx.addJoint(JointModelRY());

  /// Create Joint composite with three joints, and add a composite in it, to test the recursive_wrapper
  JointModelComposite jmodel_composite_recursive((JointModelFreeFlyer()));
  jmodel_composite_recursive.addJoint(JointModelPlanar());
  jmodel_composite_recursive.addJoint(jmodel_composite_two_rx);
}


BOOST_AUTO_TEST_CASE(test_copy)
{
  JointModelComposite jmodel_composite_planar((JointModelPX()));
  jmodel_composite_planar.addJoint(JointModelPY());
  jmodel_composite_planar.addJoint(JointModelRZ());
  jmodel_composite_planar.setIndexes(0,0,0);

  JointDataComposite jdata_composite_planar = jmodel_composite_planar.createData();

  VectorXd q1(VectorXd::Random(3));
  VectorXd q1_dot(VectorXd::Random(3));

  JointModelComposite model_copy = jmodel_composite_planar;
  JointDataComposite data_copy = model_copy.createData();
  
  BOOST_CHECK_MESSAGE( model_copy.nq() == jmodel_composite_planar.nq(), "Test Copy Composite, nq are differents");
  BOOST_CHECK_MESSAGE( model_copy.nv() == jmodel_composite_planar.nv(), "Test Copy Composite, nv are differents");

  jmodel_composite_planar.calc(jdata_composite_planar,q1, q1_dot);
  model_copy.calc(data_copy,q1, q1_dot);

}

BOOST_AUTO_TEST_CASE(test_kinematics)
{
  Model model;
  JointModelComposite jmodel_composite;

  SE3 config=SE3::Random();
  JointIndex parent=0;

  for(int i=0; i<10; i++)
  {
    parent = model.addJoint(parent, JointModelRX(), config, "joint");
    jmodel_composite.addJoint(JointModelRX(),config);

    config.setRandom();
  }

  Data data(model);

  Model model_c;
  model_c.addJoint(0,jmodel_composite,SE3::Identity(),"joint");

  Data data_c(model_c);

  BOOST_CHECK(model.nv == model_c.nv);
  BOOST_CHECK(model.nq == model_c.nq);
  
  VectorXd q(VectorXd::Random(model.nv));
  forwardKinematics(model,data,q);
  forwardKinematics(model_c,data_c,q);
  
  BOOST_CHECK(data.oMi.back().isApprox(data_c.oMi.back()));
  
  q.setRandom(model.nq);
  VectorXd v(VectorXd::Random(model.nv));
  forwardKinematics(model,data,q,v);
  forwardKinematics(model_c,data_c,q,v);
  
  BOOST_CHECK(data.oMi.back().isApprox(data_c.oMi.back()));
  BOOST_CHECK(data.v.back().isApprox(data_c.v.back()));

  q.setRandom(model.nq);
  v.setRandom(model.nv);
  VectorXd a(VectorXd::Random(model.nv));
  forwardKinematics(model,data,q,v,a);
  forwardKinematics(model_c,data_c,q,v,a);

  BOOST_CHECK(data.oMi.back().isApprox(data_c.oMi.back()));
  BOOST_CHECK(data.v.back().isApprox(data_c.v.back()));
  BOOST_CHECK(data.a.back().isApprox(data_c.a.back()));
}

BOOST_AUTO_TEST_SUITE_END ()

