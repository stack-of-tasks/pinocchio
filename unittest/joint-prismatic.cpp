//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename D>
void addJointAndBody(Model & model,
                     const JointModelBase<D> & jmodel,
                     const Model::JointIndex parent_id,
                     const SE3 & joint_placement,
                     const std::string & joint_name,
                     const Inertia & Y)
{
  Model::JointIndex idx;
  
  idx = model.addJoint(parent_id,jmodel,joint_placement,joint_name);
  model.appendBodyToJoint(idx,Y);
}

BOOST_AUTO_TEST_SUITE( JointPrismatic )
  
BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformPrismaticTpl<double,0,0> TransformX;
  typedef TransformPrismaticTpl<double,0,1> TransformY;
  typedef TransformPrismaticTpl<double,0,2> TransformZ;
  
  typedef SE3::Vector3 Vector3;
  
  const double displacement = 0.2;
  SE3 Mplain, Mrand(SE3::Random());
  
  TransformX Mx(displacement);
  Mplain = Mx;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3(displacement,0,0)));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mx));
  
  TransformY My(displacement);
  Mplain = My;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3(0,displacement,0)));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*My));
  
  TransformZ Mz(displacement);
  Mplain = Mz;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3(0,0,displacement)));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mz));
  
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionPrismaticTpl<double,0,0> mp_x(2.);
  Motion mp_dense_x(mp_x);
  
  BOOST_CHECK(M.act(mp_x).isApprox(M.act(mp_dense_x)));
  BOOST_CHECK(M.actInv(mp_x).isApprox(M.actInv(mp_dense_x)));
  
  BOOST_CHECK(v.cross(mp_x).isApprox(v.cross(mp_dense_x)));
  
  MotionPrismaticTpl<double,0,1> mp_y(2.);
  Motion mp_dense_y(mp_y);
  
  BOOST_CHECK(M.act(mp_y).isApprox(M.act(mp_dense_y)));
  BOOST_CHECK(M.actInv(mp_y).isApprox(M.actInv(mp_dense_y)));
  
  BOOST_CHECK(v.cross(mp_y).isApprox(v.cross(mp_dense_y)));
  
  MotionPrismaticTpl<double,0,2> mp_z(2.);
  Motion mp_dense_z(mp_z);
  
  BOOST_CHECK(M.act(mp_z).isApprox(M.act(mp_dense_z)));
  BOOST_CHECK(M.actInv(mp_z).isApprox(M.actInv(mp_dense_z)));
  
  BOOST_CHECK(v.cross(mp_z).isApprox(v.cross(mp_dense_z)));
}

BOOST_AUTO_TEST_CASE( test_kinematics )
{
  using namespace pinocchio;


  Motion expected_v_J(Motion::Zero());
  Motion expected_c_J(Motion::Zero());

  SE3 expected_configuration(SE3::Identity());

  JointDataPX joint_data;
  JointModelPX joint_model;

  joint_model.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));
  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));

  // -------
  q << 0. ;
  q_dot << 0.;

  joint_model.calc(joint_data, q, q_dot);

  BOOST_CHECK(expected_configuration.rotation().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK(expected_configuration.translation().isApprox(joint_data.M.translation(), 1e-12));
  BOOST_CHECK(expected_v_J.toVector().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK(expected_c_J.isApprox((Motion) joint_data.c, 1e-12));

  // -------
  q << 1.;
  q_dot << 1.;

  joint_model.calc(joint_data, q, q_dot);

  expected_configuration.translation() << 1, 0, 0;

  expected_v_J.linear() << 1., 0., 0.;

  BOOST_CHECK(expected_configuration.rotation().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK(expected_configuration.translation().isApprox(joint_data.M.translation(), 1e-12));
  BOOST_CHECK(expected_v_J.toVector().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK(expected_c_J.isApprox((Motion) joint_data.c, 1e-12));
}

BOOST_AUTO_TEST_CASE( test_rnea )
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model model;
  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());

  addJointAndBody(model,JointModelPX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data(model);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(model.nq));
  Eigen::VectorXd v(Eigen::VectorXd::Zero(model.nv));
  Eigen::VectorXd a(Eigen::VectorXd::Zero(model.nv));

  rnea(model, data, q, v, a);

  Eigen::VectorXd tau_expected(Eigen::VectorXd::Zero(model.nq));
  tau_expected  << 0;

  BOOST_CHECK(tau_expected.isApprox(data.tau, 1e-14));

  // -----
  q = Eigen::VectorXd::Ones(model.nq);
  v = Eigen::VectorXd::Ones(model.nv);
  a = Eigen::VectorXd::Ones(model.nv);

  rnea(model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK(tau_expected.isApprox(data.tau, 1e-12));

  q << 3;
  v = Eigen::VectorXd::Ones(model.nv);
  a = Eigen::VectorXd::Ones(model.nv);

  rnea(model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK(tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE( test_crba )
{
  using namespace pinocchio;
  using namespace std;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model model;
  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());

  addJointAndBody(model,JointModelPX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data(model);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(model.nq));
  Eigen::MatrixXd M_expected(model.nv,model.nv);

  crba(model, data, q);
  M_expected << 1.0;

  BOOST_CHECK(M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones(model.nq);

  crba(model, data, q);

  BOOST_CHECK(M_expected.isApprox(data.M, 1e-12));

  q << 3;

  crba(model, data, q);
  
  BOOST_CHECK(M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(JointPrismaticUnaligned)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionPrismaticUnaligned mp(MotionPrismaticUnaligned::Vector3(1.,2.,3.),6.);
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}

BOOST_AUTO_TEST_CASE(vsPX)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelPX, modelPrismaticUnaligned;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  JointModelPrismaticUnaligned joint_model_PU(axis);
  
  addJointAndBody(modelPX,JointModelPX(),0,pos,"px",inertia);
  addJointAndBody(modelPrismaticUnaligned,joint_model_PU,0,pos,"prismatic-unaligned",inertia);

  Data dataPX(modelPX);
  Data dataPrismaticUnaligned(modelPrismaticUnaligned);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelPX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelPX.nv);
  Eigen::VectorXd tauPX = Eigen::VectorXd::Ones(modelPX.nv);
  Eigen::VectorXd tauPrismaticUnaligned = Eigen::VectorXd::Ones(modelPrismaticUnaligned.nv);
  Eigen::VectorXd aPX = Eigen::VectorXd::Ones(modelPX.nv);
  Eigen::VectorXd aPrismaticUnaligned(aPX);
  
  forwardKinematics(modelPX, dataPX, q, v);
  forwardKinematics(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  computeAllTerms(modelPX, dataPX, q, v);
  computeAllTerms(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  BOOST_CHECK(dataPrismaticUnaligned.oMi[1].isApprox(dataPX.oMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.liMi[1].isApprox(dataPX.liMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.Ycrb[1].matrix().isApprox(dataPX.Ycrb[1].matrix()));
  BOOST_CHECK(dataPrismaticUnaligned.f[1].toVector().isApprox(dataPX.f[1].toVector()));
  
  BOOST_CHECK(dataPrismaticUnaligned.nle.isApprox(dataPX.nle));
  BOOST_CHECK(dataPrismaticUnaligned.com[0].isApprox(dataPX.com[0]));

  // InverseDynamics == rnea
  tauPX = rnea(modelPX, dataPX, q, v, aPX);
  tauPrismaticUnaligned = rnea(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v, aPrismaticUnaligned);

  BOOST_CHECK(tauPX.isApprox(tauPrismaticUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPX = aba(modelPX,dataPX, q, v, tauPX);
  Eigen::VectorXd aAbaPrismaticUnaligned = aba(modelPrismaticUnaligned,dataPrismaticUnaligned, q, v, tauPrismaticUnaligned);

  BOOST_CHECK(aAbaPX.isApprox(aAbaPrismaticUnaligned));

  // crba
  crba(modelPX, dataPX,q);
  crba(modelPrismaticUnaligned, dataPrismaticUnaligned, q);

  BOOST_CHECK(dataPX.M.isApprox(dataPrismaticUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;jacobianPX.resize(6,1); jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;jacobianPrismaticUnaligned.resize(6,1);jacobianPrismaticUnaligned.setZero();
  computeJointJacobians(modelPX, dataPX, q);
  computeJointJacobians(modelPrismaticUnaligned, dataPrismaticUnaligned, q);
  getJointJacobian(modelPX, dataPX, 1, LOCAL, jacobianPX);
  getJointJacobian(modelPrismaticUnaligned, dataPrismaticUnaligned, 1, LOCAL, jacobianPrismaticUnaligned);

  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));
}

BOOST_AUTO_TEST_SUITE_END()
