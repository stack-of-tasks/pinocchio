//
// Copyright (c) 2020-2021 CNRS INRIA
//

#include <iostream>

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_sparse_contact_dynamics_derivatives_no_contact)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  // Contact models and data
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) empty_contact_data;

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,empty_contact_models);
  contactDynamics(model,data,q,v,tau,empty_contact_models,empty_contact_data,prox_settings);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model,data,empty_contact_models,empty_contact_data);

  // Reference values
  computeABADerivatives(model, data_ref, q, v, tau);
  forwardKinematics(model, data_ref, q, v, data_ref.ddq);
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));

  for(size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data_ref.oMi[k].isApprox(data.oMi[k]));
    BOOST_CHECK(data_ref.ov[k].isApprox(data.ov[k]));
    BOOST_CHECK(data_ref.v[k].isApprox(data.v[k]));
    BOOST_CHECK(data_ref.a[k].isApprox(data.a[k]));
    BOOST_CHECK(data_ref.oa[k].isApprox(data.oa[k]));
  }

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));
  BOOST_CHECK(data_ref.dVdq.isApprox(data.dVdq));
  BOOST_CHECK(data_ref.J.isApprox(data.J));
  BOOST_CHECK(data_ref.dAdq.isApprox(data.dAdq));
  BOOST_CHECK(data_ref.dAdv.isApprox(data.dAdv));
  BOOST_CHECK(data_ref.dFdq.isApprox(data.dFdq));
  BOOST_CHECK(data_ref.dFdv.isApprox(data.dFdv));

  BOOST_CHECK(data_ref.dtau_dq.isApprox(data.dtau_dq));
  BOOST_CHECK(data_ref.dtau_dv.isApprox(data.dtau_dv));

  BOOST_CHECK(data_ref.ddq_dq.isApprox(data.ddq_dq));
  BOOST_CHECK(data_ref.ddq_dv.isApprox(data.ddq_dv));
  BOOST_CHECK(data_ref.Minv.isApprox(data.ddq_dtau));
}

BOOST_AUTO_TEST_CASE(test_sparse_contact_dynamics_derivatives)
{
  using namespace Eigen;
  using namespace pinocchio;
  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_LF(CONTACT_6D,model,LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  RigidContactModel ci_RF(CONTACT_3D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();

  contact_models.push_back(ci_LF); contact_data.push_back(RigidContactData(ci_LF));
  contact_models.push_back(ci_RF); contact_data.push_back(RigidContactData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model,data,contact_models,contact_data);

  // Reference values
  crba(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  container::aligned_vector<Force> fext((size_t)model.njoints,Force::Zero());
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_data[k];
    fext[cmodel.joint1_id] = cmodel.joint1_placement.act(cdata.contact_force);

    BOOST_CHECK(cdata.oMc1.isApprox(data_ref.oMi[cmodel.joint1_id] * cmodel.joint1_placement));
  }

  computeABADerivatives(model, data_ref, q, v, tau, fext);
  forwardKinematics(model,data_ref,q,v);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dFdq.isApprox(data_ref.dFdq));
  BOOST_CHECK(data.dFdv.isApprox(data_ref.dFdv));

  MatrixXd
    vLF_partial_dq(MatrixXd::Zero(6, model.nv)),
    aLF_partial_dq(MatrixXd::Zero(6, model.nv)),
    aLF_partial_dv(MatrixXd::Zero(6, model.nv)),
    aLF_partial_da(MatrixXd::Zero(6, model.nv));

  MatrixXd
    vRF_partial_dq(MatrixXd::Zero(6, model.nv)),
    aRF_partial_dq(MatrixXd::Zero(6, model.nv)),
    aRF_partial_dv(MatrixXd::Zero(6, model.nv)),
    aRF_partial_da(MatrixXd::Zero(6, model.nv));

  getFrameAccelerationDerivatives(model, data_ref,
                                  LF_id,
                                  ci_LF.joint1_placement,
                                  LOCAL,
                                  vLF_partial_dq,
                                  aLF_partial_dq,
                                  aLF_partial_dv,
                                  aLF_partial_da);
  getFrameAccelerationDerivatives(model, data_ref,
                                  RF_id,
                                  ci_RF.joint1_placement,
                                  LOCAL,
                                  vRF_partial_dq,
                                  aRF_partial_dq,
                                  aRF_partial_dv,
                                  aRF_partial_da);

  MatrixXd Jc(constraint_dim, model.nv);
  Jc << aLF_partial_da,
    aRF_partial_da.topRows<3>();

  MatrixXd K(model.nv+constraint_dim, model.nv+constraint_dim);
  K << data_ref.M, Jc.transpose(),
    Jc, MatrixXd::Zero(constraint_dim,constraint_dim);
  const MatrixXd Kinv = K.inverse();

  MatrixXd osim((Jc * data_ref.M.inverse() * Jc.transpose()).inverse());
  BOOST_CHECK(data.osim.isApprox(osim));

  MatrixXd ac_partial_dq(constraint_dim, model.nv);

  BOOST_CHECK(data.ov[RF_id].isApprox(data_ref.oMi[RF_id].act(data_ref.v[RF_id])));
  aRF_partial_dq.topRows<3>() +=
    cross(ci_RF.joint1_placement.actInv(data_ref.v[RF_id]).angular(),
          vRF_partial_dq.topRows<3>())
    - cross(ci_RF.joint1_placement.actInv(data_ref.v[RF_id]).linear(),
            vRF_partial_dq.bottomRows<3>());

  BOOST_CHECK(data.ov[LF_id].isApprox(data_ref.oMi[LF_id].act(data_ref.v[LF_id])));

  ac_partial_dq << aLF_partial_dq,
    aRF_partial_dq.topRows<3>();

  MatrixXd dac_dq = ac_partial_dq - Jc * data_ref.Minv*data_ref.dtau_dq;

  BOOST_CHECK(data.dac_dq.isApprox(dac_dq,1e-8));
  BOOST_CHECK(Kinv.bottomLeftCorner(constraint_dim, model.nv).isApprox(osim*Jc*data_ref.Minv));

  MatrixXd df_dq = Kinv.bottomLeftCorner(constraint_dim, model.nv)* data_ref.dtau_dq +
    Kinv.bottomRightCorner(constraint_dim,constraint_dim)*ac_partial_dq;

  MatrixXd ddq_dq = data_ref.Minv * (-data_ref.dtau_dq +  Jc.transpose() * df_dq);

  BOOST_CHECK(df_dq.isApprox(data.dlambda_dq));
  BOOST_CHECK(ddq_dq.isApprox(data.ddq_dq));
}

pinocchio::Motion computeAcceleration(const pinocchio::Model & model,
                                      const pinocchio::Data & data,
                                      const pinocchio::JointIndex & joint_id,
                                      const pinocchio::ReferenceFrame reference_frame,
                                      const pinocchio::ContactType contact_type,
                                      const pinocchio::SE3 & placement = pinocchio::SE3::Identity())
{
  PINOCCHIO_UNUSED_VARIABLE(model);
  using namespace pinocchio;
  Motion res(Motion::Zero());

  const Data::SE3 & oMi = data.oMi[joint_id];
  const Data::SE3 oMc = oMi * placement;

  const Data::SE3 & iMc = placement;
  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);

  switch (reference_frame)
  {
    case WORLD:
      if(contact_type == CONTACT_6D)
        return oa;
      classicAcceleration(ov,oa,res.linear());
      res.angular() = oa.angular();
      break;
    case LOCAL_WORLD_ALIGNED:
      if(contact_type == CONTACT_6D)
      {
        res.linear() = oMc.rotation() * iMc.actInv(data.a[joint_id]).linear();
        res.angular() = oMi.rotation() * data.a[joint_id].angular();
      }
      else
      {
        res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id],data.a[joint_id],iMc);
        res.angular() = oMi.rotation() * data.a[joint_id].angular();
      }
      break;
    case LOCAL:
      if(contact_type == CONTACT_6D)
        return oMc.actInv(oa);
      classicAcceleration(data.v[joint_id],data.a[joint_id],iMc,res.linear());
      res.angular() = iMc.rotation().transpose() * data.a[joint_id].angular();
      break;
    default:
      break;
  }

  return res;
}

pinocchio::Motion getContactAcceleration(const Model & model,
                                         const Data & data,
                                         const RigidContactModel & cmodel)
{
  return computeAcceleration(model,data,cmodel.joint1_id,cmodel.reference_frame,cmodel.type,cmodel.joint1_placement);
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_6D_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_LF(CONTACT_6D,model,LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();

  contact_models.push_back(ci_LF); contact_data.push_back(RigidContactData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactData)
createData(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactModel) & contact_models)
{
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactData) contact_datas;
  for(size_t k = 0; k < contact_models.size(); ++k)
    contact_datas.push_back(pinocchio::RigidContactData(contact_models[k]));
  
  return contact_datas;
}

BOOST_AUTO_TEST_CASE(test_correction_6D)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  const double mu = 0.;
  ProximalSettings prox_settings(1e-12,mu,1);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  
  RigidContactModel ci_RF(CONTACT_6D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.joint2_placement.setRandom();
  ci_RF.corrector.Kp = 10.;
  ci_RF.corrector.Kd = 2. * sqrt(ci_RF.corrector.Kp);
  contact_models.push_back(ci_RF);
  
  RigidContactModel ci_LF(CONTACT_3D,model,LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.joint2_placement.setRandom();
  ci_LF.corrector.Kp = 10.;
  ci_LF.corrector.Kd = 2. * sqrt(ci_LF.corrector.Kp);
  contact_models.push_back(ci_LF);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas = createData(contact_models);
  initContactDynamics(model,data,contact_models);
  const Eigen::VectorXd ddq0 = contactDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
  Eigen::MatrixXd ddq_dq(model.nv,model.nv), ddq_dv(model.nv,model.nv), ddq_dtau(model.nv,model.nv);
  Eigen::MatrixXd dlambda_dq(constraint_dim,model.nv), dlambda_dv(constraint_dim,model.nv), dlambda_dtau(constraint_dim,model.nv);
  
  computeContactDynamicsDerivatives(model,data,contact_models,contact_datas,
                                    ddq_dq,ddq_dv,ddq_dtau,
                                    dlambda_dq,dlambda_dv,dlambda_dtau);
  computeForwardKinematicsDerivatives(model,data,q,v,0*v);
  
  Data::Matrix6x dv_RF_dq_L(Data::Matrix6x::Zero(6,model.nv));
  Data::Matrix6x dv_RF_dv_L(Data::Matrix6x::Zero(6,model.nv));
  getFrameVelocityDerivatives(model,data,ci_RF.joint1_id,ci_RF.joint1_placement,ci_RF.reference_frame,dv_RF_dq_L,dv_RF_dv_L);
  
  Data::Matrix6x dv_LF_dq_L(Data::Matrix6x::Zero(6,model.nv));
  Data::Matrix6x dv_LF_dv_L(Data::Matrix6x::Zero(6,model.nv));
  getFrameVelocityDerivatives(model,data,ci_LF.joint1_id,ci_LF.joint1_placement,ci_LF.reference_frame,dv_LF_dq_L,dv_LF_dv_L);
  
  const double eps = 1e-8;
  Data::Matrix6x dacc_corrector_RF_dq(6,model.nv); dacc_corrector_RF_dq.setZero();
  Data::Matrix6x dacc_corrector_RF_dv(6,model.nv); dacc_corrector_RF_dv.setZero();
  Data::Matrix3x dacc_corrector_LF_dq(3,model.nv); dacc_corrector_LF_dq.setZero();
  Data::Matrix3x dacc_corrector_LF_dv(3,model.nv); dacc_corrector_LF_dv.setZero();

  {
    const SE3::Matrix6 Jlog = Jlog6(contact_datas[0].c1Mc2.inverse());
    dacc_corrector_RF_dq =  - ci_RF.corrector.Kp * Jlog * dv_RF_dv_L;
    dacc_corrector_RF_dq += - ci_RF.corrector.Kd * dv_RF_dq_L;
    
    dacc_corrector_RF_dv = - ci_RF.corrector.Kd * dv_RF_dv_L;
    BOOST_CHECK(dv_RF_dv_L.isApprox(data.contact_chol.matrix().topRightCorner(6,model.nv)));
  }
  
  {
    dacc_corrector_LF_dq =  - ci_LF.corrector.Kp * dv_LF_dv_L.topRows<3>();
    dacc_corrector_LF_dq += - ci_LF.corrector.Kd * dv_LF_dq_L.topRows<3>();
    for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
    {
      dacc_corrector_LF_dq.col(k) += ci_LF.corrector.Kp * dv_LF_dv_L.col(k).tail<3>().cross(contact_datas[1].contact_placement_error.linear());
    }
    
    dacc_corrector_LF_dv = - ci_LF.corrector.Kd * dv_LF_dv_L.topRows<3>();
    BOOST_CHECK(dv_LF_dv_L.topRows<3>().isApprox(data.contact_chol.matrix().topRightCorner(9,model.nv).bottomRows<3>()));
  }
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_fd = createData(contact_models);
  initContactDynamics(model,data_fd,contact_models);
  
  Data::Matrix6x dacc_corrector_RF_dq_fd(6,model.nv);
  Data::Matrix3x dacc_corrector_LF_dq_fd(3,model.nv);
  
  Eigen::MatrixXd ddq_dq_fd(model.nv,model.nv), ddq_dv_fd(model.nv,model.nv), ddq_dtau_fd(model.nv,model.nv);
  Eigen::MatrixXd dlambda_dq_fd(constraint_dim,model.nv), dlambda_dv_fd(constraint_dim,model.nv), dlambda_dtau_fd(constraint_dim,model.nv);
  
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd v_eps = Eigen::VectorXd::Zero(model.nv);
    v_eps[k] = eps;
    const Eigen::VectorXd q_plus = integrate(model,q,v_eps);
    
    Eigen::VectorXd ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_datas_fd,prox_settings);
    dacc_corrector_RF_dq_fd.col(k) = (contact_datas_fd[0].contact_acceleration_error - contact_datas[0].contact_acceleration_error).toVector() / eps;
    dacc_corrector_LF_dq_fd.col(k) = (contact_datas_fd[1].contact_acceleration_error.linear() - contact_datas[1].contact_acceleration_error.linear()) / eps;
    
    ddq_dq_fd.col(k) = (ddq_plus - ddq0)/eps;
  }
  
  BOOST_CHECK(ddq_dq_fd.isApprox(ddq_dq,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_RF_dq.isApprox(dacc_corrector_RF_dq_fd,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dq.isApprox(dacc_corrector_LF_dq_fd,sqrt(eps)));
  // std::cout << "dacc_corrector_RF_dq:\n" << dacc_corrector_RF_dq << std::endl;
  // std::cout << "dacc_corrector_RF_dq_fd:\n" << dacc_corrector_RF_dq_fd << std::endl;
  
  Data::Matrix6x dacc_corrector_RF_dv_fd(6,model.nv);
  Data::Matrix3x dacc_corrector_LF_dv_fd(3,model.nv);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd v_plus(v);
    v_plus[k] += eps;
    
    Eigen::VectorXd ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_datas_fd,prox_settings);
    dacc_corrector_RF_dv_fd.col(k) = (contact_datas_fd[0].contact_acceleration_error - contact_datas[0].contact_acceleration_error).toVector() / eps;
    dacc_corrector_LF_dv_fd.col(k) = (contact_datas_fd[1].contact_acceleration_error.linear() - contact_datas[1].contact_acceleration_error.linear()) / eps;
    
    ddq_dv_fd.col(k) = (ddq_plus - ddq0)/eps;
  }
  BOOST_CHECK(ddq_dv_fd.isApprox(ddq_dv,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_RF_dv.isApprox(dacc_corrector_RF_dv_fd,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dv.isApprox(dacc_corrector_LF_dv_fd,sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_3D_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_RF(CONTACT_3D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF); contact_data.push_back(RigidContactData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}


BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_3D_loop_closure_j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidContactModel ci_closure (CONTACT_3D, model,  0, SE3::Identity(),
                                RA_id, SE3::Random(), LOCAL);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidContactData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}


BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_6D_loop_closure_j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidContactModel ci_closure (CONTACT_6D, model, 0, SE3::Identity(),
                                RA_id, SE3::Random(), LOCAL);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidContactData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_6D_loop_closure_j1j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidContactModel ci_closure (CONTACT_6D, model, LA_id, SE3::Random(),
                                RA_id, SE3::Random(), LOCAL);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidContactData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));
  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_WORL_ALIGNED_6D_loop_closure_j1j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidContactModel ci_closure (CONTACT_6D, model, LA_id, SE3::Random(),
                                RA_id, SE3::Random(), LOCAL_WORLD_ALIGNED);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidContactData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}


BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_3D_loop_closure_j1j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidContactModel ci_closure (CONTACT_3D, model, LA_id, SE3::Random(),
                                RA_id, SE3::Random(), LOCAL);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidContactData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_WORLD_ALIGNED_3D_loop_closure_j1j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidContactModel ci_closure (CONTACT_3D, model, LA_id, SE3::Random(),
                                RA_id, SE3::Random(), LOCAL_WORLD_ALIGNED);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidContactData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}


BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_LOCAL_WORLD_ALIGNED_6D_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_LF(CONTACT_6D,model,LF_id,LOCAL_WORLD_ALIGNED);

  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF); contact_data.push_back(RigidContactData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE ( test_contact_dynamics_derivatives_LOCAL_WORLD_ALIGNED_3D_fd )
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_RF(CONTACT_3D,model,RF_id,LOCAL_WORLD_ALIGNED);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF); contact_data.push_back(RigidContactData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_mix_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);
  const std::string RH = "rarm6_joint";
  const Model::JointIndex RH_id = model.getJointId(RH);
  const std::string LH = "larm6_joint";
  const Model::JointIndex LH_id = model.getJointId(LH);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_LF(CONTACT_6D,model,LF_id,LOCAL_WORLD_ALIGNED);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF); contact_data.push_back(RigidContactData(ci_LF));
  RigidContactModel ci_RF(CONTACT_6D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF); contact_data.push_back(RigidContactData(ci_RF));
  
  RigidContactModel ci_LH(CONTACT_3D,model,LH_id,LOCAL_WORLD_ALIGNED);
  ci_LH.joint1_placement.setRandom();
  contact_models.push_back(ci_LH); contact_data.push_back(RigidContactData(ci_LH));
  RigidContactModel ci_RH(CONTACT_3D,model,RH_id,LOCAL);
  ci_RH.joint1_placement.setRandom();
  contact_models.push_back(ci_RH); contact_data.push_back(RigidContactData(ci_RH));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initContactDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);
  
  const Eigen::MatrixXd Jc = data.dac_da;
  const Eigen::MatrixXd Jc_ref = data.contact_chol.matrix().topRightCorner(constraint_dim,model.nv);
  
  BOOST_CHECK(Jc.isApprox(Jc_ref));
  
  const Eigen::MatrixXd JMinv = Jc * data.Minv;
  const Eigen::MatrixXd dac_dq = data.dac_dq + JMinv * data.dtau_dq;
  
  Eigen::MatrixXd dac_dq_fd(constraint_dim,model.nv);
  
  Eigen::VectorXd contact_acc0(constraint_dim);
  Eigen::DenseIndex row_id = 0;
  
  forwardKinematics(model,data,q,v,data.ddq);
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const Eigen::DenseIndex size = cmodel.size();
    
    const Motion contact_acc = getContactAcceleration(model,data,cmodel);
    
    if(cmodel.type == CONTACT_3D)
      contact_acc0.segment<3>(row_id) = contact_acc.linear();
    else
      contact_acc0.segment<6>(row_id) = contact_acc.toVector();
    
    row_id += size;
  }
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    
    Eigen::VectorXd contact_acc_plus(constraint_dim);
    Eigen::DenseIndex row_id = 0;
    forwardKinematics(model,data_fd,q_plus,v,data.ddq);
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const Eigen::DenseIndex size = cmodel.size();
      
      const Motion contact_acc = getContactAcceleration(model,data_fd,cmodel);
      
      if(cmodel.type == CONTACT_3D)
        contact_acc_plus.segment<3>(row_id) = contact_acc.linear();
      else
        contact_acc_plus.segment<6>(row_id) = contact_acc.toVector();
      
      row_id += size;
    }
    
    dac_dq_fd.col(k) = (contact_acc_plus - contact_acc0)/alpha;
    
    v_eps[k] = 0.;
  }
  
  // std::cout << "dac_dq_fd:\n" << dac_dq_fd << std::endl;
  // std::cout << "dac_dq:\n" << data.dac_da << std::endl;
  // std::cout << "dac_dq:\n" << dac_dq_fd<< std::endl;

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));
  
  // std::cout << "lambda_partial_dq_fd:\n" << lambda_partial_dq_fd << std::endl;
  // std::cout << "dlambda_dq:\n" << data.dlambda_dq << std::endl;

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }
  
  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_contact_dynamics_derivatives_dirty_data)
{
  // Verify that a dirty data doesn't affect the results of the contact dynamics derivs
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data_dirty(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_LF(CONTACT_6D,model,LF_id,LOCAL);
  RigidContactModel ci_RF(CONTACT_3D,model,RF_id,LOCAL);

  contact_models.push_back(ci_LF); contact_data.push_back(RigidContactData(ci_LF));
  contact_models.push_back(ci_RF); contact_data.push_back(RigidContactData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  initContactDynamics(model,data_dirty,contact_models);
  contactDynamics(model,data_dirty,q,v,tau,contact_models,contact_data,prox_settings);
  computeContactDynamicsDerivatives(model, data_dirty, contact_models, contact_data);

  // Reuse the same data with new configurations
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  contactDynamics(model,data_dirty,q,v,tau,contact_models,contact_data,prox_settings);
  computeContactDynamicsDerivatives(model, data_dirty, contact_models, contact_data);

  //Test with fresh data
  Data data_fresh(model);
  initContactDynamics(model,data_fresh,contact_models);
  contactDynamics(model,data_fresh,q,v,tau,contact_models,contact_data,prox_settings);
  computeContactDynamicsDerivatives(model, data_fresh, contact_models, contact_data);
  const double alpha = 1e-12;

  BOOST_CHECK(data_dirty.ddq_dq.isApprox(data_fresh.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(data_dirty.ddq_dv.isApprox(data_fresh.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(data_dirty.ddq_dtau.isApprox(data_fresh.ddq_dtau,sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dq.isApprox(data_fresh.dlambda_dq,sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dv.isApprox(data_fresh.dlambda_dv,sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dtau.isApprox(data_fresh.dlambda_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_SUITE_END ()
