//
// Copyright (c) 2022 CNRS INRIA
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

template<typename D>
void addJointAndBody(
  Model & model,
  const JointModelBase<D> & jmodel,
  const Model::JointIndex parent_id,
  const SE3 & joint_placement,
  const std::string & joint_name,
  const Inertia & Y)
{
  Model::JointIndex idx;

  idx = model.addJoint(parent_id, jmodel, joint_placement, joint_name);
  model.appendBodyToJoint(idx, Y);
}

BOOST_AUTO_TEST_SUITE(JointHelical)

BOOST_AUTO_TEST_CASE(vsPXRX)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model modelHX, modelPXRX;

  Inertia inertia(1., Vector3(0., 0., 0.0), Matrix3::Identity());
  // Important to have the same mass for both systems, otherwise COM position not the same
  Inertia inertia_zero_mass(0., Vector3(0.0, 0.0, 0.0), Matrix3::Identity());
  const double h = 0.4;

  JointModelHX joint_model_HX(h);
  addJointAndBody(modelHX, joint_model_HX, 0, SE3::Identity(), "helical x", inertia);

  JointModelPX joint_model_PX;
  JointModelRX joint_model_RX;
  addJointAndBody(modelPXRX, joint_model_PX, 0, SE3::Identity(), "prismatic x", inertia);
  addJointAndBody(modelPXRX, joint_model_RX, 1, SE3::Identity(), "revolute x", inertia_zero_mass);

  Data dataHX(modelHX);
  Data dataPXRX(modelPXRX);

  // Set the prismatic joint to corresponding displacement, velocit and acceleration
  Eigen::VectorXd q_hx = Eigen::VectorXd::Ones(modelHX.nq);     // dim 1
  Eigen::VectorXd q_PXRX = Eigen::VectorXd::Ones(modelPXRX.nq); // dim 2
  q_PXRX(0) = q_hx(0) * h;

  Eigen::VectorXd v_hx = Eigen::VectorXd::Ones(modelHX.nv);
  Eigen::VectorXd v_PXRX = Eigen::VectorXd::Ones(modelPXRX.nv);
  v_PXRX(0) = v_hx(0) * h;

  Eigen::VectorXd tauHX = Eigen::VectorXd::Ones(modelHX.nv);
  Eigen::VectorXd tauPXRX = Eigen::VectorXd::Ones(modelPXRX.nv);
  Eigen::VectorXd aHX = Eigen::VectorXd::Ones(modelHX.nv);
  Eigen::VectorXd aPXRX = Eigen::VectorXd::Ones(modelPXRX.nv);
  aPXRX(0) = aHX(0) * h * h;

  forwardKinematics(modelHX, dataHX, q_hx, v_hx);
  forwardKinematics(modelPXRX, dataPXRX, q_PXRX, v_PXRX);

  computeAllTerms(modelHX, dataHX, q_hx, v_hx);
  computeAllTerms(modelPXRX, dataPXRX, q_PXRX, v_PXRX);

  BOOST_CHECK(dataPXRX.oMi[2].isApprox(dataHX.oMi[1]));
  BOOST_CHECK((dataPXRX.liMi[2] * dataPXRX.liMi[1]).isApprox(dataHX.liMi[1]));
  BOOST_CHECK(dataPXRX.Ycrb[2].matrix().isApprox(dataHX.Ycrb[1].matrix()));
  BOOST_CHECK((dataPXRX.liMi[2].actInv(dataPXRX.f[1])).toVector().isApprox(dataHX.f[1].toVector()));
  BOOST_CHECK(
    (Eigen::Matrix<double, 1, 1>(dataPXRX.nle.dot(Eigen::VectorXd::Ones(2)))).isApprox(dataHX.nle));
  BOOST_CHECK(dataPXRX.com[0].isApprox(dataHX.com[0]));

  // InverseDynamics == rnea
  tauHX = rnea(modelHX, dataHX, q_hx, v_hx, aHX);
  tauPXRX = rnea(modelPXRX, dataPXRX, q_PXRX, v_PXRX, aPXRX);
  BOOST_CHECK(tauHX.isApprox(Eigen::Matrix<double, 1, 1>(tauPXRX.dot(Eigen::VectorXd::Ones(2)))));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaHX = aba(modelHX, dataHX, q_hx, v_hx, tauHX, Convention::WORLD);
  Eigen::VectorXd aAbaPXRX = aba(modelPXRX, dataPXRX, q_PXRX, v_PXRX, tauPXRX, Convention::WORLD);

  BOOST_CHECK(aAbaHX.isApprox(aHX));
  BOOST_CHECK(aAbaPXRX.isApprox(aPXRX));
  BOOST_CHECK(aAbaPXRX.isApprox(Eigen::Matrix<double, 2, 1>(aHX(0) * h * h, aHX(0))));

  aAbaHX = aba(modelHX, dataHX, q_hx, v_hx, tauHX, Convention::LOCAL);
  aAbaPXRX = aba(modelPXRX, dataPXRX, q_PXRX, v_PXRX, tauPXRX, Convention::LOCAL);

  BOOST_CHECK(aAbaHX.isApprox(aHX));
  BOOST_CHECK(aAbaPXRX.isApprox(aPXRX));
  BOOST_CHECK(aAbaPXRX.isApprox(Eigen::Matrix<double, 2, 1>(aHX(0) * h * h, aHX(0))));

  // crba
  crba(modelHX, dataHX, q_hx, Convention::WORLD);
  crba(modelPXRX, dataPXRX, q_PXRX, Convention::WORLD);

  tauHX = dataHX.M * aHX;
  tauPXRX = dataPXRX.M * aPXRX;

  BOOST_CHECK(tauHX.isApprox(Eigen::Matrix<double, 1, 1>(tauPXRX.dot(Eigen::VectorXd::Ones(2)))));

  crba(modelHX, dataHX, q_hx, Convention::LOCAL);
  crba(modelPXRX, dataPXRX, q_PXRX, Convention::LOCAL);

  tauHX = dataHX.M * aHX;
  tauPXRX = dataPXRX.M * aPXRX;

  BOOST_CHECK(tauHX.isApprox(Eigen::Matrix<double, 1, 1>(tauPXRX.dot(Eigen::VectorXd::Ones(2)))));

  // Jacobian
  computeJointJacobians(modelHX, dataHX, q_hx);
  computeJointJacobians(modelPXRX, dataPXRX, q_PXRX);
  Eigen::VectorXd v_body_hx = dataHX.J * v_hx;
  Eigen::VectorXd v_body_PXRX = dataPXRX.J * v_PXRX;
  BOOST_CHECK(v_body_hx.isApprox(v_body_PXRX));
}

BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformHelicalTpl<double, 0, 0> TransformX;
  typedef TransformHelicalTpl<double, 0, 1> TransformY;
  typedef TransformHelicalTpl<double, 0, 2> TransformZ;

  typedef SE3::Vector3 Vector3;

  const double alpha = 0.2, h = 0.1;
  double sin_alpha, cos_alpha;
  SINCOS(alpha, &sin_alpha, &cos_alpha);
  SE3 Mplain, Mrand(SE3::Random());

  TransformX Mx(sin_alpha, cos_alpha, alpha * h);
  Mplain = Mx;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3::UnitX() * alpha * h));
  BOOST_CHECK(
    Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha, Vector3::UnitX()).toRotationMatrix()));
  BOOST_CHECK((Mrand * Mplain).isApprox(Mrand * Mx));

  TransformY My(sin_alpha, cos_alpha, alpha * h);
  Mplain = My;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3::UnitY() * alpha * h));
  BOOST_CHECK(
    Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha, Vector3::UnitY()).toRotationMatrix()));
  BOOST_CHECK((Mrand * Mplain).isApprox(Mrand * My));

  TransformZ Mz(sin_alpha, cos_alpha, alpha * h);
  Mplain = Mz;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3::UnitZ() * alpha * h));
  BOOST_CHECK(
    Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha, Vector3::UnitZ()).toRotationMatrix()));
  BOOST_CHECK((Mrand * Mplain).isApprox(Mrand * Mz));

  SE3 M(SE3::Random());
  Motion v(Motion::Random());

  MotionHelicalTpl<double, 0, 0> mh_x(2., h);
  Motion mh_dense_x(mh_x);

  BOOST_CHECK(M.act(mh_x).isApprox(M.act(mh_dense_x)));
  BOOST_CHECK(M.actInv(mh_x).isApprox(M.actInv(mh_dense_x)));

  BOOST_CHECK(v.cross(mh_x).isApprox(v.cross(mh_dense_x)));

  MotionHelicalTpl<double, 0, 1> mh_y(2., h);
  Motion mh_dense_y(mh_y);

  BOOST_CHECK(M.act(mh_y).isApprox(M.act(mh_dense_y)));
  BOOST_CHECK(M.actInv(mh_y).isApprox(M.actInv(mh_dense_y)));

  BOOST_CHECK(v.cross(mh_y).isApprox(v.cross(mh_dense_y)));

  MotionHelicalTpl<double, 0, 2> mh_z(2., h);
  Motion mh_dense_z(mh_z);

  BOOST_CHECK(M.act(mh_z).isApprox(M.act(mh_dense_z)));
  BOOST_CHECK(M.actInv(mh_z).isApprox(M.actInv(mh_dense_z)));

  BOOST_CHECK(v.cross(mh_z).isApprox(v.cross(mh_dense_z)));
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(JointHelicalUnaligned)

BOOST_AUTO_TEST_CASE(vsHX)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Vector3 axis;
  axis << 1.0, 0.0, 0.0;
  const double h = 0.2;

  Model modelHX, modelHelicalUnaligned;

  Inertia inertia(1., Vector3(0.0, 0., 0.0), Matrix3::Identity());
  SE3 pos(1);
  pos.translation() = SE3::LinearType(1., 0., 0.);

  JointModelHelicalUnaligned joint_model_HU(axis, h);

  addJointAndBody(modelHX, JointModelHX(h), 0, pos, "HX", inertia);
  addJointAndBody(modelHelicalUnaligned, joint_model_HU, 0, pos, "Helical-unaligned", inertia);

  Data dataHX(modelHX);
  Data dataHelicalUnaligned(modelHelicalUnaligned);

  Eigen::VectorXd q = 3 * Eigen::VectorXd::Ones(modelHX.nq);
  Eigen::VectorXd v = 30 * Eigen::VectorXd::Ones(modelHX.nv);
  Eigen::VectorXd tauHX = Eigen::VectorXd::Ones(modelHX.nv);
  Eigen::VectorXd tauHelicalUnaligned = Eigen::VectorXd::Ones(modelHelicalUnaligned.nv);
  Eigen::VectorXd aHX = 5 * Eigen::VectorXd::Ones(modelHX.nv);
  Eigen::VectorXd aHelicalUnaligned(aHX);

  forwardKinematics(modelHX, dataHX, q, v);
  forwardKinematics(modelHelicalUnaligned, dataHelicalUnaligned, q, v);

  computeAllTerms(modelHX, dataHX, q, v);
  computeAllTerms(modelHelicalUnaligned, dataHelicalUnaligned, q, v);

  BOOST_CHECK(dataHelicalUnaligned.oMi[1].isApprox(dataHX.oMi[1]));
  BOOST_CHECK(dataHelicalUnaligned.liMi[1].isApprox(dataHX.liMi[1]));
  BOOST_CHECK(dataHelicalUnaligned.Ycrb[1].matrix().isApprox(dataHX.Ycrb[1].matrix()));
  BOOST_CHECK(dataHelicalUnaligned.f[1].toVector().isApprox(dataHX.f[1].toVector()));

  BOOST_CHECK(dataHelicalUnaligned.nle.isApprox(dataHX.nle));
  BOOST_CHECK(dataHelicalUnaligned.com[0].isApprox(dataHX.com[0]));

  // InverseDynamics == rnea
  tauHX = rnea(modelHX, dataHX, q, v, aHX);
  tauHelicalUnaligned = rnea(modelHelicalUnaligned, dataHelicalUnaligned, q, v, aHelicalUnaligned);

  BOOST_CHECK(tauHX.isApprox(tauHelicalUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaHX = aba(modelHX, dataHX, q, v, tauHX, Convention::WORLD);
  Eigen::VectorXd aAbaHelicalUnaligned =
    aba(modelHelicalUnaligned, dataHelicalUnaligned, q, v, tauHelicalUnaligned, Convention::WORLD);

  BOOST_CHECK(aAbaHX.isApprox(aAbaHelicalUnaligned));

  // crba
  crba(modelHX, dataHX, q, Convention::WORLD);
  crba(modelHelicalUnaligned, dataHelicalUnaligned, q, Convention::WORLD);

  BOOST_CHECK(dataHX.M.isApprox(dataHelicalUnaligned.M));

  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;
  jacobianPX.resize(6, 1);
  jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;
  jacobianPrismaticUnaligned.resize(6, 1);
  jacobianPrismaticUnaligned.setZero();
  computeJointJacobians(modelHX, dataHX, q);
  computeJointJacobians(modelHelicalUnaligned, dataHelicalUnaligned, q);
  getJointJacobian(modelHX, dataHX, 1, LOCAL, jacobianPX);
  getJointJacobian(
    modelHelicalUnaligned, dataHelicalUnaligned, 1, LOCAL, jacobianPrismaticUnaligned);

  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));
}

BOOST_AUTO_TEST_SUITE_END()
