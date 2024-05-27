//
// Copyright (c) 2023 INRIA
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
using namespace Eigen;

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

BOOST_AUTO_TEST_SUITE(JointUniversal)

BOOST_AUTO_TEST_CASE(vsRXRY)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Vector3 axis1;
  axis1 << 1.0, 0.0, 0.0;
  Vector3 axis2;
  axis2 << 0.0, 1.0, 0.0;

  Model modelUniversal, modelRXRY;
  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());

  JointModelUniversal joint_model_U(axis1, axis2);
  addJointAndBody(modelUniversal, joint_model_U, 0, SE3::Identity(), "universal", inertia);

  JointModelComposite joint_model_RXRY;
  joint_model_RXRY.addJoint(JointModelRX());
  joint_model_RXRY.addJoint(JointModelRY());
  addJointAndBody(modelRXRY, joint_model_RXRY, 0, SE3::Identity(), "rxry", inertia);

  Data dataUniversal(modelUniversal);
  Data dataRXRY(modelRXRY);

  BOOST_CHECK(modelUniversal.nv == modelRXRY.nv);
  BOOST_CHECK(modelUniversal.nq == modelRXRY.nq);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelRXRY.nq);

  forwardKinematics(modelRXRY, dataRXRY, q);
  forwardKinematics(modelUniversal, dataUniversal, q);

  BOOST_CHECK(dataUniversal.oMi.back().isApprox(dataRXRY.oMi.back()));
  BOOST_CHECK(dataUniversal.liMi.back().isApprox(dataRXRY.liMi.back()));
  BOOST_CHECK(dataUniversal.Ycrb.back().matrix().isApprox(dataRXRY.Ycrb.back().matrix()));

  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelRXRY.nv);
  forwardKinematics(modelRXRY, dataRXRY, q, v);
  forwardKinematics(modelUniversal, dataUniversal, q, v);

  BOOST_CHECK(dataUniversal.oMi.back().isApprox(dataRXRY.oMi.back()));
  BOOST_CHECK(dataUniversal.liMi.back().isApprox(dataRXRY.liMi.back()));
  BOOST_CHECK(dataUniversal.Ycrb.back().matrix().isApprox(dataRXRY.Ycrb.back().matrix()));

  computeAllTerms(modelRXRY, dataRXRY, q, v);
  computeAllTerms(modelUniversal, dataUniversal, q, v);

  BOOST_CHECK(dataUniversal.com.back().isApprox(dataRXRY.com.back()));
  BOOST_CHECK(dataUniversal.nle.isApprox(dataRXRY.nle));
  BOOST_CHECK(dataUniversal.f.back().toVector().isApprox(dataRXRY.f.back().toVector()));

  // InverseDynamics == rnea
  Eigen::VectorXd a = Eigen::VectorXd::Ones(modelRXRY.nv);

  Eigen::VectorXd tauRXRY = rnea(modelRXRY, dataRXRY, q, v, a);
  Eigen::VectorXd tauUniversal = rnea(modelUniversal, dataUniversal, q, v, a);

  BOOST_CHECK(tauUniversal.isApprox(tauRXRY));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRXRY = aba(modelRXRY, dataRXRY, q, v, tauRXRY, Convention::WORLD);
  Eigen::VectorXd aAbaUniversal =
    aba(modelUniversal, dataUniversal, q, v, tauUniversal, Convention::WORLD);

  BOOST_CHECK(aAbaUniversal.isApprox(aAbaRXRY));

  // CRBA
  crba(modelRXRY, dataRXRY, q, Convention::WORLD);
  crba(modelUniversal, dataUniversal, q, Convention::WORLD);

  BOOST_CHECK(dataUniversal.M.isApprox(dataRXRY.M));

  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRXRY;
  jacobianRXRY.resize(6, 2);
  jacobianRXRY.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianUniversal;
  jacobianUniversal.resize(6, 2);
  jacobianUniversal.setZero();

  computeJointJacobians(modelRXRY, dataRXRY, q);
  computeJointJacobians(modelUniversal, dataUniversal, q);
  getJointJacobian(modelRXRY, dataRXRY, 1, LOCAL, jacobianRXRY);
  getJointJacobian(modelUniversal, dataUniversal, 1, LOCAL, jacobianUniversal);

  BOOST_CHECK(jacobianUniversal.isApprox(jacobianRXRY));
}

BOOST_AUTO_TEST_CASE(vsRandomAxis)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Vector3 axis1;
  axis1 << 0., 0., 1.;
  Vector3 axis2;
  axis2 << -1., 0., 0.;

  Model modelUniversal, modelRandomAxis;
  Inertia inertia = Inertia::Random();

  JointModelUniversal joint_model_U(axis1, axis2);
  addJointAndBody(modelUniversal, joint_model_U, 0, SE3::Identity(), "universal", inertia);

  JointModelComposite joint_model_RandomAxis;
  joint_model_RandomAxis.addJoint(JointModelRevoluteUnaligned(axis1));
  joint_model_RandomAxis.addJoint(JointModelRevoluteUnaligned(axis2));
  addJointAndBody(
    modelRandomAxis, joint_model_RandomAxis, 0, SE3::Identity(), "random_axis", inertia);

  Data dataUniversal(modelUniversal);
  Data dataRandomAxis(modelRandomAxis);

  BOOST_CHECK(modelUniversal.nv == modelRandomAxis.nv);
  BOOST_CHECK(modelUniversal.nq == modelRandomAxis.nq);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelRandomAxis.nq);

  forwardKinematics(modelRandomAxis, dataRandomAxis, q);
  forwardKinematics(modelUniversal, dataUniversal, q);

  BOOST_CHECK(dataUniversal.oMi.back().isApprox(dataRandomAxis.oMi.back()));
  BOOST_CHECK(dataUniversal.liMi.back().isApprox(dataRandomAxis.liMi.back()));
  BOOST_CHECK(dataUniversal.Ycrb.back().matrix().isApprox(dataRandomAxis.Ycrb.back().matrix()));

  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelRandomAxis.nv);
  forwardKinematics(modelRandomAxis, dataRandomAxis, q, v);
  forwardKinematics(modelUniversal, dataUniversal, q, v);

  BOOST_CHECK(dataUniversal.oMi.back().isApprox(dataRandomAxis.oMi.back()));
  BOOST_CHECK(dataUniversal.liMi.back().isApprox(dataRandomAxis.liMi.back()));
  BOOST_CHECK(dataUniversal.Ycrb.back().matrix().isApprox(dataRandomAxis.Ycrb.back().matrix()));

  computeAllTerms(modelRandomAxis, dataRandomAxis, q, v);
  computeAllTerms(modelUniversal, dataUniversal, q, v);

  BOOST_CHECK(dataUniversal.com.back().isApprox(dataRandomAxis.com.back()));
  BOOST_CHECK(dataUniversal.nle.isApprox(dataRandomAxis.nle));
  BOOST_CHECK(dataUniversal.f.back().toVector().isApprox(dataRandomAxis.f.back().toVector()));

  // InverseDynamics == rnea
  Eigen::VectorXd a = Eigen::VectorXd::Ones(modelRandomAxis.nv);

  Eigen::VectorXd tauRandomAxis = rnea(modelRandomAxis, dataRandomAxis, q, v, a);
  Eigen::VectorXd tauUniversal = rnea(modelUniversal, dataUniversal, q, v, a);

  BOOST_CHECK(tauUniversal.isApprox(tauRandomAxis));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRandomAxis =
    aba(modelRandomAxis, dataRandomAxis, q, v, tauRandomAxis, Convention::WORLD);
  Eigen::VectorXd aAbaUniversal =
    aba(modelUniversal, dataUniversal, q, v, tauUniversal, Convention::WORLD);

  BOOST_CHECK(aAbaUniversal.isApprox(aAbaRandomAxis));

  // CRBA
  crba(modelRandomAxis, dataRandomAxis, q, Convention::WORLD);
  crba(modelUniversal, dataUniversal, q, Convention::WORLD);

  BOOST_CHECK(dataUniversal.M.isApprox(dataRandomAxis.M));

  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRandomAxis;
  jacobianRandomAxis.resize(6, 2);
  jacobianRandomAxis.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianUniversal;
  jacobianUniversal.resize(6, 2);
  jacobianUniversal.setZero();

  computeJointJacobians(modelRandomAxis, dataRandomAxis, q);
  computeJointJacobians(modelUniversal, dataUniversal, q);
  getJointJacobian(modelRandomAxis, dataRandomAxis, 1, LOCAL, jacobianRandomAxis);
  getJointJacobian(modelUniversal, dataUniversal, 1, LOCAL, jacobianUniversal);

  BOOST_CHECK(jacobianUniversal.isApprox(jacobianRandomAxis));
}

BOOST_AUTO_TEST_SUITE_END()
