//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

/// \brief Computes forces in the world frame
pinocchio::Motion computeFrameAcc(const pinocchio::Model & model,
                                  pinocchio::Data & data,
                                  const pinocchio::Model::FrameIndex & frame_id,
                                  pinocchio::ReferenceFrame reference_frame,
                                  const pinocchio::SE3 & placement = pinocchio::SE3::Identity())
{
  using namespace pinocchio;
  const Model::JointIndex& joint_id = model.frames[frame_id].parent;
  Motion res(Motion::Zero());
  
  updateFramePlacement(model,data,frame_id);
  
  const Data::SE3 & oMf = data.oMf[frame_id];
  const Data::SE3 oMc = oMf * placement;
  const Data::SE3 & oMi = data.oMi[joint_id];
  
//  const Data::SE3 iMf = oMi.actInv(oMf);
  const Data::SE3 iMc = oMi.actInv(oMc);
  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);
  
  switch (reference_frame)
  {
    case WORLD:
      classicAcceleration(ov,oa,res.linear());
      res.angular() = oa.angular();
      break;
    case LOCAL_WORLD_ALIGNED:
      res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id],data.a[joint_id],iMc);
      res.angular() = oMi.rotation() * data.a[joint_id].angular();
      break;
    case LOCAL:
      classicAcceleration(data.v[joint_id],data.a[joint_id],iMc,res.linear());
      res.angular() = iMc.rotation().transpose() * data.a[joint_id].angular();
      break;
    default:
      break;
  }
  
  return res;
}

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_empty)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  
  const double mu0 = 0.;

  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  initContactDynamics(model,data,contact_models);
  impulseDynamics(model,data,q,v,contact_models,contact_datas,mu0,0.);
  
  data.M.triangularView<Eigen::StrictlyLower>() =
  data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data.dq_after.isApprox(v));
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
//  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
//  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),WORLD);
  ci_RF.placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
  contact_models.push_back(ci_LF);
  ci_LF.placement.setRandom();
  contact_datas.push_back(RigidContactData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  updateFramePlacements(model,data_ref);
  getFrameJacobian(model,data_ref,model.getFrameId(RF),WORLD,J_ref.middleRows<6>(0));
  getFrameJacobian(model,data_ref,model.getFrameId(LF),WORLD,J_ref.middleRows<6>(6));
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = computeFrameAcc(model,data_ref,model.getFrameId(RF),ci_RF.reference_frame,ci_RF.placement).toVector();
  rhs_ref.segment<6>(6) = computeFrameAcc(model,data_ref,model.getFrameId(LF),ci_LF.reference_frame,ci_LF.placement).toVector();

  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());
  
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);
  
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  data.M.triangularView<Eigen::StrictlyLower>() =
  data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Data data_ag(model);
  ccrba(model,data_ag,q,v);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data.Ag.isApprox(data_ag.Ag));
  BOOST_CHECK(data.nle.isApprox(data_ref.nle));
  
  for(Model::JointIndex k = 1; k < model.joints.size(); ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.liMi[k].isApprox(data_ref.liMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.oMi[k].act(data_ref.v[k])));
    BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oMi[k].act(data_ref.a_gf[k])));
  }
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_datas[k];
    
    switch(cmodel.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D_LOCAL)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),LOCAL);
  ci_RF.placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
//  ci_LF.placement.setRandom();
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  updateFramePlacements(model,data_ref);
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  getFrameJacobian(model,data_ref,
                   ci_RF.frame_id,ci_RF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(0) = ci_RF.placement.inverse().toActionMatrix() * Jtmp;
  
  Jtmp.setZero();
  getFrameJacobian(model,data_ref,
                   ci_LF.frame_id,ci_LF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(6) = Jtmp;
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  rhs_ref.segment<6>(0) = computeFrameAcc(model,data_ref,ci_RF.frame_id,ci_RF.reference_frame,ci_RF.placement).toVector();
  rhs_ref.segment<6>(6) = computeFrameAcc(model,data_ref,ci_LF.frame_id,ci_LF.reference_frame,ci_LF.placement).toVector();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());
  
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);
  
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_datas[k];
    
    switch(cmodel.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D_3D)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),WORLD);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_3D,model.getFrameId(LF),WORLD);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  updateFramePlacements(model,data_ref);
  getFrameJacobian(model,data_ref,model.getFrameId(RF),WORLD,J_ref.middleRows<6>(0));
  Data::Matrix6x J_LF(6,model.nv); J_LF.setZero();
  getFrameJacobian(model,data_ref,model.getFrameId(LF),WORLD,J_LF);
  J_ref.middleRows<3>(6) = J_LF.middleRows<3>(Motion::LINEAR);
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = computeFrameAcc(model,data_ref,model.getFrameId(RF),ci_RF.reference_frame).toVector();
  rhs_ref.segment<3>(6) = computeFrameAcc(model,data_ref,model.getFrameId(LF),ci_LF.reference_frame).linear();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_datas[k];
    
    switch(cmodel.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D_LOCAL_WORLD_ALIGNED)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  updateFramePlacements(model,data_ref);
  getFrameJacobian(model,data_ref,
                   ci_RF.frame_id,ci_RF.reference_frame,
                   J_ref.middleRows<6>(0));
  getFrameJacobian(model,data_ref,
                   ci_LF.frame_id,ci_LF.reference_frame,
                   J_ref.middleRows<6>(6));
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = computeFrameAcc(model,data_ref,ci_RF.frame_id,ci_RF.reference_frame).toVector();
  rhs_ref.segment<6>(6) = computeFrameAcc(model,data_ref,ci_LF.frame_id,ci_LF.reference_frame).toVector();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_datas[k];
    
    switch(cmodel.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_CASE(test_diagonal_inertia)
{
  using namespace pinocchio;

  const double mu = 1e2;
  const Inertia diagonal6_inertia(mu,Inertia::Vector3::Zero(),
                                  Symmetric3(mu,0,mu,0,0,mu));
  const Inertia::Matrix6 diagonal6_inertia_mat = diagonal6_inertia.matrix();
  BOOST_CHECK(diagonal6_inertia_mat.block(Inertia::LINEAR,Inertia::ANGULAR,3,3).isZero());
  BOOST_CHECK(diagonal6_inertia_mat.block(Inertia::ANGULAR,Inertia::LINEAR,3,3).isZero());
  
  const SE3 M = SE3::Random();
//  const Inertia::Matrix3 RtRmu = mu * M.rotation().transpose()*M.rotation();
  const Inertia::Matrix3 RtRmu = mu * Inertia::Matrix3::Identity();
  Inertia I6_translate(mu,M.translation(),Symmetric3(RtRmu));
  
  const Inertia I6_ref = M.act(diagonal6_inertia);
  BOOST_CHECK(I6_translate.isApprox(I6_ref));
  
  const Inertia diagonal3_inertia(mu,Inertia::Vector3::Zero(),
                                  Symmetric3(0,0,0,0,0,0));
  const Inertia::Matrix6 diagonal3_inertia_mat = diagonal3_inertia.matrix();
  BOOST_CHECK(diagonal3_inertia_mat.block(Inertia::LINEAR,Inertia::ANGULAR,3,3).isZero());
  BOOST_CHECK(diagonal3_inertia_mat.block(Inertia::ANGULAR,Inertia::LINEAR,3,3).isZero());
  BOOST_CHECK(diagonal3_inertia_mat.block(Inertia::ANGULAR,Inertia::ANGULAR,3,3).isZero());
  
  Inertia I3_translate(mu,
                       M.translation(),
                       Symmetric3(0,0,0,0,0,0));
  
  const Inertia I3_ref = M.act(diagonal3_inertia);
  BOOST_CHECK(I3_translate.isApprox(I3_ref));
}


BOOST_AUTO_TEST_SUITE_END ()
