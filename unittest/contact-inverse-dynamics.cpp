//
// Copyright (c) 2019-2023 INRIA
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/contact-inverse-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <optional>

#define KP 10
#define KD 10

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

/// \brief Computes motions in the world frame
pinocchio::Motion computeAcceleration(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::JointIndex & joint_id,
  pinocchio::ReferenceFrame reference_frame,
  const pinocchio::ContactType type,
  const pinocchio::SE3 & placement = pinocchio::SE3::Identity())
{
  PINOCCHIO_UNUSED_VARIABLE(model);
  using namespace pinocchio;
  Motion res(Motion::Zero());

  const Data::SE3 & oMi = data.oMi[joint_id];
  const Data::SE3 & iMc = placement;
  const Data::SE3 oMc = oMi * iMc;

  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);

  switch (reference_frame)
  {
  case WORLD:
    if (type == CONTACT_3D)
      classicAcceleration(ov, oa, res.linear());
    else
      res.linear() = oa.linear();
    res.angular() = oa.angular();
    break;
  case LOCAL_WORLD_ALIGNED:
    if (type == CONTACT_3D)
      res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id], data.a[joint_id], iMc);
    else
      res.linear() = oMc.rotation() * (iMc.actInv(data.a[joint_id])).linear();
    res.angular() = oMi.rotation() * data.a[joint_id].angular();
    break;
  case LOCAL:
    if (type == CONTACT_3D)
      classicAcceleration(data.v[joint_id], data.a[joint_id], iMc, res.linear());
    else
      res.linear() = (iMc.actInv(data.a[joint_id])).linear();
    res.angular() = iMc.rotation().transpose() * data.a[joint_id].angular();
    break;
  default:
    break;
  }

  return res;
}

BOOST_AUTO_TEST_CASE(test_contact_inverse_dynamics_3D)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)
    RigidConstraintModelVector;
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) RigidConstraintDataVector;
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(CoulombFrictionCone) CoulombFrictionConeVector;

  RigidConstraintModelVector contact_models;
  RigidConstraintDataVector contact_datas;
  CoulombFrictionConeVector cones;
  RigidConstraintModel ci_RF(CONTACT_3D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  cones.push_back(CoulombFrictionCone(0.4));
  RigidConstraintModel ci_LF(CONTACT_3D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  cones.push_back(CoulombFrictionCone(0.4));

  RigidConstraintDataVector contact_datas_ref(contact_datas);

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  Eigen::MatrixXd J_ref(constraint_dim, model.nv);
  J_ref.setZero();

  double dt = 1e-3;
  Eigen::VectorXd R = Eigen::VectorXd::Zero(constraint_dim);
  Eigen::VectorXd constraint_correction = Eigen::VectorXd::Zero(constraint_dim);
  boost::optional<Eigen::VectorXd> lambda_guess = boost::optional<Eigen::VectorXd>(boost::none);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  ProximalSettings prox_settings(1e-12, 1e-6, 1);
  initConstraintDynamics(model, data_ref, contact_models);
  contactInverseDynamics(
    model, data_ref, q, v, a, dt, contact_models, contact_datas, cones, R, constraint_correction,
    prox_settings, lambda_guess);
  //   constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas_ref,
  //   prox_settings_cd); forwardKinematics(model, data_ref, q, v, v*0);

  //   Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  //   getJointJacobian(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,Jtmp);
  //   J_ref.middleRows<3>(0) = Jtmp.middleRows<3>(Motion::LINEAR);
  //   Jtmp.setZero(); getJointJacobian(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,Jtmp);
  //   J_ref.middleRows<3>(3) = Jtmp.middleRows<3>(Motion::LINEAR);

  //   Eigen::VectorXd gamma(constraint_dim);

  //   gamma.segment<3>(0) =
  //   computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type).linear();
  //   gamma.segment<3>(3) =
  //   computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type).linear();

  //   BOOST_CHECK((J_ref*data_ref.ddq + gamma).isZero());

  //   Data data_constrained_dyn(model);

  // PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  // PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  //   forwardDynamics(model,data_constrained_dyn,q,v,tau,J_ref,gamma,0.);
  // PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  //   BOOST_CHECK((J_ref*data_constrained_dyn.ddq + gamma).isZero());

  //   ProximalSettings prox_settings;
  //   prox_settings.max_iter = 10;
  //   prox_settings.mu = 1e8;
  //   contactABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  //   BOOST_CHECK((J_ref*data.ddq + gamma).isZero());

  //   // Call the algorithm a second time
  //   Data data2(model);
  //   ProximalSettings prox_settings2;
  //   contactABA(model, data2, q, v, tau, contact_models, contact_datas, prox_settings2);

  //   BOOST_CHECK(prox_settings2.iter == 0);
}

BOOST_AUTO_TEST_SUITE_END()
