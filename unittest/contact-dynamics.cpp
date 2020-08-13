//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(contact_models)
{
  using namespace pinocchio;
  
  // Check default constructor
  RigidContactModel cmodel1;
  BOOST_CHECK(cmodel1.type == CONTACT_UNDEFINED);
  BOOST_CHECK(cmodel1.size() == 0);
  
  // Check complete constructor
  SE3 M(SE3::Random());
  RigidContactModel cmodel2(CONTACT_3D,0);
  BOOST_CHECK(cmodel2.type == CONTACT_3D);
  BOOST_CHECK(cmodel2.frame_id == 0);
  BOOST_CHECK(cmodel2.size() == 3);
  
  // Check contructor with two arguments
  RigidContactModel cmodel2prime(CONTACT_3D,0);
  BOOST_CHECK(cmodel2prime.type == CONTACT_3D);
  BOOST_CHECK(cmodel2prime.frame_id == 0);
  BOOST_CHECK(cmodel2prime.size() == 3);
  
  // Check default copy constructor
  RigidContactModel cmodel3(cmodel2);
  BOOST_CHECK(cmodel3 == cmodel2);
  
  // Check complete constructor 6D
  RigidContactModel cmodel4(CONTACT_6D,0);
  BOOST_CHECK(cmodel4.type == CONTACT_6D);
  BOOST_CHECK(cmodel4.frame_id == 0);
  BOOST_CHECK(cmodel4.size() == 6);
}

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

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_empty)
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
  
  const double mu0 = 0.;

  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv,model.nv);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);
  
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
  aba(model,data_ref,q,v,tau);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_double_init)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data1(model), data2(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";

  // Contact models and data
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_empty;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_empty;
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_6D;
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_6D6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_6D6D;
  
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),WORLD);
  contact_models_6D.push_back(ci_RF);
  contact_datas_6D.push_back(RigidContactData(ci_RF));
  contact_models_6D6D.push_back(ci_RF);
  contact_datas_6D6D.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
  contact_models_6D6D.push_back(ci_LF);
  contact_datas_6D6D.push_back(RigidContactData(ci_LF));
  
  initContactDynamics(model,data1,contact_models_empty);
  BOOST_CHECK(data1.contact_chol.size() == (model.nv + 0));
  contactDynamics(model,data1,q,v,tau,contact_models_empty,contact_datas_empty);
  BOOST_CHECK(!hasNaN(data1.ddq));
  
  initContactDynamics(model,data1,contact_models_6D);
  BOOST_CHECK(data1.contact_chol.size() == (model.nv + 1*6));
  contactDynamics(model,data1,q,v,tau,contact_models_6D,contact_datas_6D);
  BOOST_CHECK(!hasNaN(data1.ddq));
  
  initContactDynamics(model,data1,contact_models_6D6D);
  BOOST_CHECK(data1.contact_chol.size() == (model.nv + 2*6));
  contactDynamics(model,data1,q,v,tau,contact_models_6D6D,contact_datas_6D6D);
  BOOST_CHECK(!hasNaN(data1.ddq));
  
  initContactDynamics(model,data2,contact_models_6D6D);
  initContactDynamics(model,data2,contact_models_6D);
  initContactDynamics(model,data2,contact_models_empty);
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
  getFrameJacobian(model,data_ref,model.getFrameId(RF),WORLD,J_ref.middleRows<6>(0));
  getFrameJacobian(model,data_ref,model.getFrameId(LF),WORLD,J_ref.middleRows<6>(6));
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = computeFrameAcc(model,data_ref,model.getFrameId(RF),ci_RF.reference_frame).toVector();
  rhs_ref.segment<6>(6) = computeFrameAcc(model,data_ref,model.getFrameId(LF),ci_LF.reference_frame).toVector();

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
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),WORLD);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
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
  J_ref.middleRows<6>(0) = Jtmp;
  
  Jtmp.setZero();
  getFrameJacobian(model,data_ref,
                   ci_LF.frame_id,ci_LF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(6) = Jtmp;
  
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

BOOST_AUTO_TEST_CASE(test_contact_ABA_with_armature)
{
  using namespace pinocchio;
  using namespace Eigen;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.rotorInertia = 100. * (Model::VectorXs::Random(model.nv) + Model::VectorXs::Constant(model.nv,1.));
  model.rotorGearRatio.fill(100);
  
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) RigidContactModelVector;
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) RigidContactDataVector;
  const RigidContactModelVector empty_rigid_contact_models;
  RigidContactDataVector empty_rigid_contact_data;
  
  const Data::VectorXs a = contactABA(model,data,q,v,tau,empty_rigid_contact_models,empty_rigid_contact_data);
  const Data::VectorXs tau_ref = rnea(model,data_ref,q,v,a);
  
  BOOST_CHECK(tau.isApprox(tau_ref));
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

BOOST_AUTO_TEST_CASE(test_contact_ABA_6D)
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
//  const Frame & RF_frame = model.frames[model.getFrameId(RF)];
//  Frame RF_contact_frame("RF_contact_frame",
//                         RF_frame.parent,model.getFrameId(RF),
//                         SE3::Random(),OP_FRAME);
//  model.addFrame(RF_contact_frame);
  
  const std::string LF = "lleg6_joint";
//  const Frame & LF_frame = model.frames[model.getFrameId(LF)];
//  Frame LF_contact_frame("LF_contact_frame",
//                         LF_frame.parent,model.getFrameId(RF),
//                         SE3::Random(),OP_FRAME);
//  model.addFrame(LF_contact_frame);
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) RigidContactModelVector;
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) RigidContactDataVector;
  
  const RigidContactModelVector empty_contact_models;
  RigidContactDataVector empty_contact_data;
  
  contactABA(model, data, q, v, tau, empty_contact_models, empty_contact_data);
  forwardKinematics(model, data_ref, q, v, 0*v);
  for(JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    BOOST_CHECK(data.liMi[joint_id].isApprox(data_ref.liMi[joint_id]));
    BOOST_CHECK(data.oMi[joint_id].isApprox(data_ref.oMi[joint_id]));
    BOOST_CHECK(data.ov[joint_id].isApprox(data_ref.oMi[joint_id].act(data_ref.v[joint_id])));
    BOOST_CHECK(data.oa_drift[joint_id].isApprox(data_ref.oMi[joint_id].act(data_ref.a[joint_id])));
  }
  
  computeJointJacobians(model,data_ref,q);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  aba(model, data_ref, q, v, tau);
  
  for(JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    const Data::SE3 & oMi = data.oMi[joint_id];
    Eigen::MatrixXd U_ref = oMi.toDualActionMatrix() * data_ref.joints[joint_id].U();
    BOOST_CHECK(data.joints[joint_id].U().isApprox(U_ref));
    Eigen::MatrixXd StYS_ref = data_ref.joints[joint_id].S().matrix().transpose() * data_ref.joints[joint_id].U();
    BOOST_CHECK(data.joints[joint_id].StU().isApprox(StYS_ref));
    const Data::Matrix6 oYaba_ref = oMi.toDualActionMatrix() * data_ref.Yaba[joint_id] * oMi.inverse().toActionMatrix();
    BOOST_CHECK(data.oYaba[joint_id].isApprox(oYaba_ref));
    BOOST_CHECK(data.oa_augmented[joint_id].isApprox(model.gravity+data_ref.oMi[joint_id].act(data_ref.a[joint_id])));
  }
  
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  // Test second call
  contactABA(model, data, q, v, tau, empty_contact_models, empty_contact_data);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  RigidContactModelVector contact_models; RigidContactDataVector contact_datas;
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  RigidContactDataVector contact_datas_ref(contact_datas);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  initContactDynamics(model, data_ref, contact_models);
  contactDynamics(model, data_ref, q, v, tau, contact_models, contact_datas_ref);
  forwardKinematics(model, data_ref, q, v, v*0);
  
  updateFramePlacements(model,data_ref);
  Data::Matrix6x Jtmp(6,model.nv);
  
  Jtmp.setZero();
  getFrameJacobian(model,data_ref,
                   ci_RF.frame_id,ci_RF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(0) = Jtmp;
  
  Jtmp.setZero();
  getFrameJacobian(model,data_ref,
                   ci_LF.frame_id,ci_LF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(6) = Jtmp;
  
  Eigen::VectorXd gamma(constraint_dim);
  
  gamma.segment<6>(0) = computeFrameAcc(model,data_ref,ci_RF.frame_id,ci_RF.reference_frame).toVector();
  gamma.segment<6>(6) = computeFrameAcc(model,data_ref,ci_LF.frame_id,ci_LF.reference_frame).toVector();
  
  BOOST_CHECK((J_ref*data_ref.ddq + gamma).isZero());
  
  Data data_constrained_dyn(model);
  forwardDynamics(model,data_constrained_dyn,q,v,tau,J_ref,gamma,mu0);
  BOOST_CHECK((J_ref*data_constrained_dyn.ddq + gamma).isZero());
  
  ProximalSettings prox_settings;
  prox_settings.max_iter = 10;
  prox_settings.mu = 1e8;
  const double mu = prox_settings.mu;
  contactABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  
  BOOST_CHECK((J_ref*data.ddq + gamma).isZero());
  
  forwardKinematics(model, data_ref, q, v, 0*v);
  for(JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    BOOST_CHECK(data.oa_drift[joint_id].isApprox(data_ref.oMi[joint_id].act(data_ref.a[joint_id])));
  }
       
  aba(model,data_ref,q,v,0*v);
  for(size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
  {
    const RigidContactModel & cmodel = contact_models[contact_id];
    const RigidContactData & cdata = contact_datas[contact_id];

    const RigidContactModel::FrameIndex & frame_id = cmodel.frame_id;
    const Model::Frame & frame = model.frames[frame_id];
    const Model::JointIndex & joint_id = frame.parent;
    
    // Check contact placement
    const SE3 & iMc = frame.placement;
    const SE3 oMc = data_ref.oMi[joint_id] * iMc;
    //BOOST_CHECK(cdata.contact_placement.isApprox(oMc));
    
    // Check contact velocity
    const Motion contact_velocity_ref = iMc.actInv(data_ref.v[joint_id]);
    BOOST_CHECK(cdata.contact_velocity.isApprox(contact_velocity_ref));
    
    // Check contact inertia
    Symmetric3 S(Symmetric3::Zero());
    if(cmodel.type == CONTACT_6D)
      S.setDiagonal(Symmetric3::Vector3::Constant(mu));
    
    const Inertia contact_inertia(mu,data_ref.oMf[frame_id].translation(),S);
    
    Inertia::Matrix6 contact_inertia_ref = Inertia::Matrix6::Zero();
    
    if(cmodel.type == CONTACT_6D)
      contact_inertia_ref.diagonal().fill(mu);
    else
      contact_inertia_ref.diagonal().head<3>().fill(mu);
    contact_inertia_ref = oMc.toDualActionMatrix() * contact_inertia_ref * oMc.toActionMatrixInverse();
    BOOST_CHECK(contact_inertia_ref.isApprox(contact_inertia.matrix()));
    
    Inertia::Matrix6 Yaba_ref
    = data_ref.oMi[joint_id].toDualActionMatrix() * model.inertias[joint_id].matrix() * data_ref.oMi[joint_id].toActionMatrixInverse()
    + contact_inertia_ref;
    
    const JointModel & jmodel = model.joints[joint_id];
    const JointData & jdata = data.joints[joint_id];
//    const JointData & jdata_ref = data_ref.joints[joint_id];
    
    const MatrixXd U_ref = Yaba_ref * data_ref.J.middleCols(jmodel.idx_v(),jmodel.nv());
    const MatrixXd D_ref = data_ref.J.middleCols(jmodel.idx_v(),jmodel.nv()).transpose() * U_ref;
    const MatrixXd Dinv_ref = D_ref.inverse();
    const MatrixXd UDinv_ref = U_ref * Dinv_ref;
    BOOST_CHECK(jdata.U().isApprox(U_ref));
    BOOST_CHECK(jdata.StU().isApprox(D_ref));
    BOOST_CHECK(jdata.Dinv().isApprox(Dinv_ref));
    BOOST_CHECK(jdata.UDinv().isApprox(UDinv_ref));
    
    Yaba_ref -= UDinv_ref * U_ref.transpose();
    
    BOOST_CHECK(data.oYaba[joint_id].isApprox(Yaba_ref));
    
  }
  
  // Call the algorithm a second time
  Data data2(model);
  ProximalSettings prox_settings2;
  contactABA(model, data2, q, v, tau, contact_models, contact_datas, prox_settings2);
  
  BOOST_CHECK(prox_settings2.iter == 0);
  
}

BOOST_AUTO_TEST_CASE(test_contact_ABA_3D)
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
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) RigidContactModelVector;
  typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) RigidContactDataVector;
  
  RigidContactModelVector contact_models; RigidContactDataVector contact_datas;
  RigidContactModel ci_RF(CONTACT_3D,model.getFrameId(RF),LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_3D,model.getFrameId(LF),LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  RigidContactDataVector contact_datas_ref(contact_datas);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  initContactDynamics(model, data_ref, contact_models);
  contactDynamics(model, data_ref, q, v, tau, contact_models, contact_datas_ref);
  forwardKinematics(model, data_ref, q, v, v*0);
  
  updateFramePlacements(model,data_ref);
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  getFrameJacobian(model,data_ref,ci_RF.frame_id,ci_RF.reference_frame,Jtmp);
  J_ref.middleRows<3>(0) = Jtmp.middleRows<3>(Motion::LINEAR);
  Jtmp.setZero(); getFrameJacobian(model,data_ref,ci_LF.frame_id,ci_LF.reference_frame,Jtmp);
  J_ref.middleRows<3>(3) = Jtmp.middleRows<3>(Motion::LINEAR);
  
  Eigen::VectorXd gamma(constraint_dim);
  
  gamma.segment<3>(0) = computeFrameAcc(model,data_ref,ci_RF.frame_id,ci_RF.reference_frame).linear();
  gamma.segment<3>(3) = computeFrameAcc(model,data_ref,ci_LF.frame_id,ci_LF.reference_frame).linear();
  
  BOOST_CHECK((J_ref*data_ref.ddq + gamma).isZero());
  
  Data data_constrained_dyn(model);
  forwardDynamics(model,data_constrained_dyn,q,v,tau,J_ref,gamma,0.);
  BOOST_CHECK((J_ref*data_constrained_dyn.ddq + gamma).isZero());
  
  ProximalSettings prox_settings;
  prox_settings.max_iter = 10;
  prox_settings.mu = 1e8;
  contactABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  
  BOOST_CHECK((J_ref*data.ddq + gamma).isZero());
  
  // Call the algorithm a second time
  Data data2(model);
  ProximalSettings prox_settings2;
  contactABA(model, data2, q, v, tau, contact_models, contact_datas, prox_settings2);
  
  BOOST_CHECK(prox_settings2.iter == 0);
  
}

BOOST_AUTO_TEST_SUITE_END ()
