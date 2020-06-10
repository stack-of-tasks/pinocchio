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

BOOST_AUTO_TEST_SUITE(JointSpherical)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionSpherical mp(MotionSpherical::Vector3(1.,2.,3.));
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}

BOOST_AUTO_TEST_CASE(vsFreeFlyer)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef SE3::Matrix3 Matrix3;

  Model modelSpherical, modelFreeflyer;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelSpherical,JointModelSpherical(),0,pos,"spherical",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,pos,"free-flyer",inertia);
  
  Data dataSpherical(modelSpherical);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelSpherical.nq);q.normalize();
  VectorFF qff; qff << 0, 0, 0, q[0], q[1], q[2], q[3];
  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelSpherical.nv);
  Vector6 vff; vff << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd tauSpherical = Eigen::VectorXd::Ones(modelSpherical.nv);
  Eigen::VectorXd tauff; tauff.resize(7); tauff << 0,0,0,1,1,1,1;
  Eigen::VectorXd aSpherical = Eigen::VectorXd::Ones(modelSpherical.nv);
  Eigen::VectorXd aff(vff);
  
  forwardKinematics(modelSpherical, dataSpherical, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelSpherical, dataSpherical, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataSpherical.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataSpherical.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataSpherical.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataSpherical.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[3],
                                                         dataFreeFlyer.nle[4],
                                                         dataFreeFlyer.nle[5]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataSpherical.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataSpherical.com[0]));

  // InverseDynamics == rnea
  tauSpherical = rnea(modelSpherical, dataSpherical, q, v, aSpherical);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(3), tauff(4), tauff(5);
  BOOST_CHECK(tauSpherical.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaSpherical = aba(modelSpherical,dataSpherical, q, v, tauSpherical);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[3],
                                    aAbaFreeFlyer[4],
                                    aAbaFreeFlyer[5]
                                    ;
  BOOST_CHECK(aAbaSpherical.isApprox(a_expected));

  // crba
  crba(modelSpherical, dataSpherical,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected(dataFreeFlyer.M.bottomRightCorner<3,3>());

  BOOST_CHECK(dataSpherical.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJointJacobians(modelSpherical, dataSpherical, q);
  computeJointJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJointJacobian(modelSpherical, dataSpherical, 1, LOCAL, jacobian_planar);
  getJointJacobian(modelFreeflyer, dataFreeFlyer, 1, LOCAL, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(3),
                                                                      jacobian_ff.col(4),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));

}
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(JointSphericalZYX)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionSpherical mp(MotionSpherical::Vector3(1.,2.,3.));
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}

BOOST_AUTO_TEST_CASE(vsFreeFlyer)
{
  // WARNIG : Dynamic algorithm's results cannot be compared to FreeFlyer's ones because
  // of the representation of the rotation and the ConstraintSubspace difference.
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef SE3::Matrix3 Matrix3;

  Model modelSphericalZYX, modelFreeflyer;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelSphericalZYX,JointModelSphericalZYX(),0,pos,"spherical-zyx",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,pos,"free-flyer",inertia);

  Data dataSphericalZYX(modelSphericalZYX);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::AngleAxisd rollAngle(1, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(1, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_sph = rollAngle * yawAngle * pitchAngle;
  
  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelSphericalZYX.nq);
  VectorFF qff; qff << 0, 0, 0, q_sph.x(), q_sph.y(), q_sph.z(), q_sph.w();
  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelSphericalZYX.nv);
  Vector6 vff; vff << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd tauSpherical = Eigen::VectorXd::Ones(modelSphericalZYX.nv);
  Eigen::VectorXd tauff; tauff.resize(6); tauff << 0,0,0,1,1,1;
  Eigen::VectorXd aSpherical = Eigen::VectorXd::Ones(modelSphericalZYX.nv);
  Eigen::VectorXd aff(vff);
  
  forwardKinematics(modelSphericalZYX, dataSphericalZYX, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelSphericalZYX, dataSphericalZYX, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataSphericalZYX.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataSphericalZYX.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataSphericalZYX.Ycrb[1].matrix()));

  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataSphericalZYX.com[0]));
}

BOOST_AUTO_TEST_CASE(test_rnea)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model model;
  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());

  addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  rnea(model, data, q, v, a);
  Vector3 tau_expected(0., -4.905, 0.);

  BOOST_CHECK(tau_expected.isApprox(data.tau, 1e-14));

  q = Eigen::VectorXd::Ones(model.nq);
  v = Eigen::VectorXd::Ones(model.nv);
  a = Eigen::VectorXd::Ones(model.nv);

  rnea(model, data, q, v, a);
  tau_expected << -0.53611600195085, -0.74621832606188, -0.38177329067604;

  BOOST_CHECK(tau_expected.isApprox(data.tau, 1e-12));

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones(model.nv);
  a = Eigen::VectorXd::Ones(model.nv);

  rnea(model, data, q, v, a);
  tau_expected << 0.73934458094049,  2.7804530848031, 0.50684940972146;

  BOOST_CHECK(tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE(test_crba)
{
  using namespace pinocchio;
  using namespace std;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model model;
  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());

  addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data(model);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(model.nq));
  Eigen::MatrixXd M_expected(model.nv,model.nv);

  crba(model, data, q);
  M_expected <<
  1.25,    0,    0,
  0, 1.25,    0,
  0,    0,    1;

  BOOST_CHECK(M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones(model.nq);

  crba(model, data, q);
  M_expected <<
  1.0729816454316, -5.5511151231258e-17,     -0.8414709848079,
  -5.5511151231258e-17,                 1.25,                    0,
  -0.8414709848079,                    0,                    1;

  BOOST_CHECK(M_expected.isApprox(data.M, 1e-12));

  q << 3, 2, 1;

  crba(model, data, q);
  M_expected <<
  1.043294547392, 2.7755575615629e-17,   -0.90929742682568,
  0,                1.25,                   0,
  -0.90929742682568,                   0,                  1;

  BOOST_CHECK(M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END()
