//
// Copyright (c) 2016-2020 CNRS INRIA
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

template<typename JointModel>
void test_joint_methods(const pinocchio::JointModelBase<JointModel> & jmodel)
{
  typedef typename pinocchio::JointModelBase<JointModel>::JointDataDerived JointData;
  typedef typename JointModel::ConfigVector_t ConfigVector_t;
  typedef typename pinocchio::LieGroup<JointModel>::type LieGroupType;

  JointData jdata = jmodel.createData();

  ConfigVector_t ql(ConfigVector_t::Constant(jmodel.nq(),-M_PI));
  ConfigVector_t qu(ConfigVector_t::Constant(jmodel.nq(),M_PI));

  ConfigVector_t q = LieGroupType().randomConfiguration(ql,qu);
  pinocchio::Inertia::Matrix6 I(pinocchio::Inertia::Random().matrix());
  pinocchio::Inertia::Matrix6 I_check = I;

  jmodel.calc(jdata,q);
  jmodel.calc_aba(jdata,I,true);

  Eigen::MatrixXd S = jdata.S.matrix();
  Eigen::MatrixXd U_check = I_check*S;
  Eigen::MatrixXd D_check = S.transpose()*U_check;
  Eigen::MatrixXd Dinv_check = D_check.inverse();
  Eigen::MatrixXd UDinv_check = U_check*Dinv_check;
  Eigen::MatrixXd update_check = U_check*Dinv_check*U_check.transpose();
  I_check -= update_check;

  BOOST_CHECK(jdata.U.isApprox(U_check));
  BOOST_CHECK(jdata.Dinv.isApprox(Dinv_check));
  BOOST_CHECK(jdata.UDinv.isApprox(UDinv_check));

  // Checking the inertia was correctly updated
  // We use isApprox as usual, except for the freeflyer,
  // where the correct result is exacly zero and isApprox would fail.
  // Only for this single case, we use the infinity norm of the difference
  if(jmodel.shortname() == "JointModelFreeFlyer")
    BOOST_CHECK((I-I_check).lpNorm<Eigen::Infinity>() < Eigen::NumTraits<double>::dummy_precision());
  else
    BOOST_CHECK(I.isApprox(I_check));
}

struct TestJointMethods{

  template <typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

  void operator()(const pinocchio::JointModelBase<pinocchio::JointModelComposite> &) const
  {
    pinocchio::JointModelComposite jmodel_composite;
    jmodel_composite.addJoint(pinocchio::JointModelRX());
    jmodel_composite.addJoint(pinocchio::JointModelRY());
    jmodel_composite.setIndexes(0,0,0);

    //TODO: correct LieGroup
    //test_joint_methods(jmodel_composite);

  }

  void operator()(const pinocchio::JointModelBase<pinocchio::JointModelRevoluteUnaligned> &) const
  {
    pinocchio::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

  void operator()(const pinocchio::JointModelBase<pinocchio::JointModelPrismaticUnaligned> &) const
  {
    pinocchio::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

};

BOOST_AUTO_TEST_CASE( test_joint_basic )
{
  using namespace pinocchio;

  typedef boost::variant< JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned
  , JointModelSpherical, JointModelSphericalZYX
  , JointModelPX, JointModelPY, JointModelPZ
  , JointModelPrismaticUnaligned
  , JointModelFreeFlyer
  , JointModelPlanar
  , JointModelTranslation
  , JointModelRUBX, JointModelRUBY, JointModelRUBZ
  > Variant;

  boost::mpl::for_each<Variant::types>(TestJointMethods());
}

BOOST_AUTO_TEST_CASE ( test_aba_simple )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model; buildModels::humanoidRandom(model);
  
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.segment<4>(3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  
  tau = rnea(model, data_ref, q, v, a);
  aba(model, data, q, v, tau);
  
  for(size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data_ref.liMi[k].isApprox(data.liMi[k]));
    BOOST_CHECK(data_ref.v[k].isApprox(data.v[k]));
  }
  
  BOOST_CHECK(data.ddq.isApprox(a, 1e-12));
  
}

BOOST_AUTO_TEST_CASE ( test_aba_with_fext )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model; buildModels::humanoidRandom(model);
  
  pinocchio::Data data(model);
  
  VectorXd q = VectorXd::Random(model.nq);
  q.segment<4>(3).normalize();
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  PINOCCHIO_ALIGNED_STD_VECTOR(Force) fext(model.joints.size(), Force::Random());
  
  crba(model, data, q);
  computeJointJacobians(model, data, q);
  nonLinearEffects(model, data, q, v);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  

  VectorXd tau = data.M * a + data.nle;
  Data::Matrix6x J = Data::Matrix6x::Zero(6, model.nv);
  for(Model::Index i=1;i<(Model::Index)model.njoints;++i) {
    getJointJacobian(model, data, i, LOCAL, J);
    tau -= J.transpose()*fext[i].toVector();
    J.setZero();
  }
  aba(model, data, q, v, tau, fext);
  
  BOOST_CHECK(data.ddq.isApprox(a, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_aba_vs_rnea )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model; buildModels::humanoidRandom(model);
  
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  
  crba(model, data_ref, q);
  nonLinearEffects(model, data_ref, q, v);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  tau = data_ref.M * a + data_ref.nle;
  aba(model, data, q, v, tau);
  
  VectorXd tau_ref = rnea(model, data_ref, q, v, a);
  BOOST_CHECK(tau_ref.isApprox(tau, 1e-12));
  
  
  BOOST_CHECK(data.ddq.isApprox(a, 1e-12));
  
}

BOOST_AUTO_TEST_CASE ( test_computeMinverse )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  buildModels::humanoidRandom(model);
  model.gravity.setZero();
  
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);

  crba(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  MatrixXd Minv_ref(data_ref.M.inverse());

  computeMinverse(model, data, q);

  
  BOOST_CHECK(data.Minv.topRows<6>().isApprox(Minv_ref.topRows<6>()));
  
  data.Minv.triangularView<Eigen::StrictlyLower>()
  = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  
  BOOST_CHECK(data.Minv.isApprox(Minv_ref));
  
//  std::cout << "Minv:\n" << data.Minv.block<10,10>(0,0) << std::endl;
//  std::cout << "Minv_ref:\n" << Minv_ref.block<10,10>(0,0) << std::endl;
//
//  std::cout << "Minv:\n" << data.Minv.bottomRows<10>() << std::endl;
//  std::cout << "Minv_ref:\n" << Minv_ref.bottomRows<10>() << std::endl;
  
}

BOOST_AUTO_TEST_CASE(test_multiple_calls)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data1(model), data2(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  computeMinverse(model,data1,q);
  data2 = data1;
  
  for(int k = 0; k < 20; ++k)
  {
    computeMinverse(model,data1,q);
  }
  
  BOOST_CHECK(data1.Minv.isApprox(data2.Minv));
}

BOOST_AUTO_TEST_SUITE_END ()
