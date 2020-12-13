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

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_models)
{
  using namespace pinocchio;
  
  // Check default constructor
  RigidContactModel cmodel1;
  BOOST_CHECK(cmodel1.type == CONTACT_UNDEFINED);
  BOOST_CHECK(cmodel1.size() == 0);
  
  // Check complete constructor
  const SE3 M(SE3::Random());
  RigidContactModel cmodel2(CONTACT_3D,0,M);
  BOOST_CHECK(cmodel2.type == CONTACT_3D);
  BOOST_CHECK(cmodel2.joint1_id == 0);
  BOOST_CHECK(cmodel2.joint1_placement.isApprox(M));
  BOOST_CHECK(cmodel2.size() == 3);
  
  // Check contructor with two arguments
  RigidContactModel cmodel2prime(CONTACT_3D,0);
  BOOST_CHECK(cmodel2prime.type == CONTACT_3D);
  BOOST_CHECK(cmodel2prime.joint1_id == 0);
  BOOST_CHECK(cmodel2prime.joint1_placement.isIdentity());
  BOOST_CHECK(cmodel2prime.size() == 3);
  
  // Check default copy constructor
  RigidContactModel cmodel3(cmodel2);
  BOOST_CHECK(cmodel3 == cmodel2);
  
  // Check complete constructor 6D
  RigidContactModel cmodel4(CONTACT_6D,0);
  BOOST_CHECK(cmodel4.type == CONTACT_6D);
  BOOST_CHECK(cmodel4.joint1_id == 0);
  BOOST_CHECK(cmodel4.joint1_placement.isIdentity());
  BOOST_CHECK(cmodel4.size() == 6);
}

/// \brief Computes motions in the world frame
pinocchio::Motion computeAcceleration(const pinocchio::Model & model,
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
      if(type == CONTACT_3D)
        classicAcceleration(ov,oa,res.linear());
      else
        res.linear() = oa.linear();
      res.angular() = oa.angular();
      break;
    case LOCAL_WORLD_ALIGNED:
      if(type == CONTACT_3D)
        res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id],data.a[joint_id],iMc);
      else
        res.linear() = oMc.rotation() * (iMc.actInv(data.a[joint_id])).linear();
      res.angular() = oMi.rotation() * data.a[joint_id].angular();
      break;
    case LOCAL:
      if(type == CONTACT_3D)
        classicAcceleration(data.v[joint_id],data.a[joint_id],iMc,res.linear());
      else
        res.linear() = (iMc.actInv(data.a[joint_id])).linear();
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) empty_contact_datas;
  
  const double mu0 = 0.;

  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv,model.nv);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  
  initContactDynamics(model,data,empty_contact_models);
  contactDynamics(model,data,q,v,tau,empty_contact_models,empty_contact_datas,mu0);
  
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
  
  RigidContactModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL);
  contact_models_6D.push_back(ci_RF);
  contact_datas_6D.push_back(RigidContactData(ci_RF));
  contact_models_6D6D.push_back(ci_RF);
  contact_datas_6D6D.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
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
  RigidContactModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  getJointJacobian(model,data_ref,
                   ci_RF.joint1_id,ci_RF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  Jtmp.setZero();
  getJointJacobian(model,data_ref,
                   ci_LF.joint1_id,ci_LF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(6) = ci_LF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type,ci_RF.joint1_placement).toVector();
  rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type,ci_LF.joint1_placement).toVector();
  
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
  const std::string RA = "rarm6_joint";
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_3D,model.getJointId(LF),LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  RigidContactModel ci_RA(CONTACT_3D,model.getJointId(RA),LOCAL);
  contact_models.push_back(ci_RA);
  contact_datas.push_back(RigidContactData(ci_RA));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  getJointJacobian(model,data_ref,model.getJointId(RF),LOCAL,J_ref.middleRows<6>(0));
  Data::Matrix6x J_LF(6,model.nv); J_LF.setZero();
  getJointJacobian(model,data_ref,model.getJointId(LF),LOCAL_WORLD_ALIGNED,J_LF);
  J_ref.middleRows<3>(6) = J_LF.middleRows<3>(Motion::LINEAR);
  Data::Matrix6x J_RA(6,model.nv); J_RA.setZero();
  getJointJacobian(model,data_ref,model.getJointId(RA),LOCAL,J_RA);
  J_ref.middleRows<3>(9) = J_RA.middleRows<3>(Motion::LINEAR);
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,model.getJointId(RF),ci_RF.reference_frame,ci_RF.type).toVector();
  rhs_ref.segment<3>(6) = computeAcceleration(model,data_ref,model.getJointId(LF),ci_LF.reference_frame,ci_LF.type).linear();
  rhs_ref.segment<3>(9) = computeAcceleration(model,data_ref,model.getJointId(RA),ci_RA.reference_frame,ci_RA.type).linear();
  
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
  RigidContactModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
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
  getJointJacobian(model,data_ref,
                   ci_RF.joint1_id,ci_RF.reference_frame,
                   J_ref.middleRows<6>(0));
  getJointJacobian(model,data_ref,
                   ci_LF.joint1_id,ci_LF.reference_frame,
                   J_ref.middleRows<6>(6));
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type).toVector();
  rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type).toVector();
  
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

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_specifying_joint2id)
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
  const std::string RA = "rarm6_joint";

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  
  RigidContactModel ci_RF(CONTACT_6D,0,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
  RigidContactModel ci_RF_bis(CONTACT_6D,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
  ci_RF.joint1_placement.setRandom();
  ci_RF.joint2_placement.setRandom();
  ci_RF_bis.joint1_placement = ci_RF.joint2_placement;
  ci_RF_bis.joint2_placement = ci_RF.joint1_placement;
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  
  RigidContactModel ci_LF(CONTACT_6D,0,model.getJointId(LF),LOCAL);
  RigidContactModel ci_LF_bis(CONTACT_6D,model.getJointId(LF),LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.joint2_placement.setRandom();
  ci_LF_bis.joint1_placement = ci_LF.joint2_placement;
  ci_LF_bis.joint2_placement = ci_LF.joint1_placement;
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  RigidContactModel ci_RA(CONTACT_6D,0,model.getJointId(RA),LOCAL);
  RigidContactModel ci_RA_bis(CONTACT_6D,model.getJointId(RA),LOCAL);
  ci_RA.joint1_placement.setRandom();
  ci_RA.joint2_placement.setRandom();
  ci_RA_bis.joint1_placement = ci_RA.joint2_placement;
  ci_RA_bis.joint2_placement = ci_RA.joint1_placement;
  contact_models.push_back(ci_RA);
  contact_datas.push_back(RigidContactData(ci_RA));
  
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
  Data::Matrix6x J_RF(6,model.nv), J_LF(6,model.nv), J_RA(6,model.nv);
  J_RF.setZero(); J_LF.setZero(); J_RA.setZero();
  Data::Matrix6x J_RF_local(6,model.nv), J_LF_local(6,model.nv), J_RA_local(6,model.nv);
  J_RF_local.setZero(); J_LF_local.setZero(); J_RA_local.setZero();
  getJointJacobian(model,data_ref,
                   ci_RF.joint2_id,WORLD,
                   J_RF);
  getJointJacobian(model,data_ref,
                   ci_RF.joint2_id,LOCAL,
                   J_RF_local);
  getJointJacobian(model,data_ref,
                   ci_LF.joint2_id,WORLD,
                   J_LF);
  getJointJacobian(model,data_ref,
                   ci_LF.joint2_id,LOCAL,
                   J_LF_local);
  getJointJacobian(model,data_ref,
                   ci_RA.joint2_id,WORLD,
                   J_RA);
  getJointJacobian(model,data_ref,
                   ci_RA.joint2_id,LOCAL,
                   J_RA_local);
  
  {
    const SE3 oMc(SE3::Matrix3::Identity(),(data_ref.oMi[ci_RF.joint1_id]*ci_RF.joint1_placement).translation());
    J_ref.middleRows<6>(0) = -oMc.toActionMatrixInverse() * J_RF;
  }
  
  {
    J_ref.middleRows<6>(6) = -(data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement).toActionMatrixInverse() * J_LF;
  }
  
  {
    J_ref.middleRows<6>(12) = -(data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).toActionMatrixInverse() * J_RA;
  }
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  forwardKinematics(model, data_ref, q, v, 0*v);
  const SE3 c1Mc2_1 = (data.oMi[ci_RF.joint1_id]*ci_RF.joint1_placement).actInv(data_ref.oMi[ci_RF.joint2_id]*ci_RF.joint2_placement);
  SE3 c1Mc2_1_W((data_ref.oMi[ci_RF.joint2_id]).rotation(),
                -(data_ref.oMi[ci_RF.joint1_id] * ci_RF.joint1_placement).translation() + data_ref.oMi[ci_RF.joint2_id].translation());
  Motion acc_1 = c1Mc2_1_W.act(data_ref.a[ci_RF.joint2_id]);
  
  const SE3 c1Mc2_2 = (data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement).actInv(data_ref.oMi[ci_LF.joint2_id]*ci_LF.joint2_placement);
  Motion acc_2 = c1Mc2_2.act(ci_LF.joint2_placement.actInv(data_ref.a[ci_LF.joint2_id]));
  
  const SE3 c1Mc2_3 = (data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).actInv(data_ref.oMi[ci_RA.joint2_id]*ci_RA.joint2_placement);
  Motion acc_3 = c1Mc2_3.act(ci_RA.joint2_placement.actInv(data_ref.a[ci_RA.joint2_id]));
  
  rhs_ref.segment<6>(0) = -acc_1.toVector();
  rhs_ref.segment<6>(6) = -acc_2.toVector();
  rhs_ref.segment<6>(12) = -acc_3.toVector();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.topLeftCorner(constraint_dim,constraint_dim).diagonal().fill(-mu0);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);
  
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);
  
  std::cout << "acc_1 ref:\n" << acc_1 << std::endl;
  std::cout << "acc_1:\n" << contact_datas[0].contact2_acceleration_drift << std::endl;
  BOOST_CHECK(acc_1.isApprox(contact_datas[0].contact2_acceleration_drift));

  std::cout << "acc_2 ref:\n" << acc_2 << std::endl;
  std::cout << "acc_2:\n" << contact_datas[1].contact2_acceleration_drift << std::endl;
  BOOST_CHECK(acc_2.isApprox(contact_datas[1].contact2_acceleration_drift));

  std::cout << "acc_3 ref:\n" << acc_3 << std::endl;
  std::cout << "acc_3:\n" << contact_datas[2].contact2_acceleration_drift << std::endl;
  BOOST_CHECK(acc_3.isApprox(contact_datas[2].contact2_acceleration_drift));

  BOOST_CHECK(contact_datas[0].c1Mc2.isApprox(c1Mc2_1));
 
  const SE3 c1Mc2_1_LWA(contact_datas[0].oMc2.rotation(),
                        contact_datas[0].oMc1.rotation()*c1Mc2_1.translation());
  BOOST_CHECK((c1Mc2_1_LWA.toActionMatrix()*(ci_RF.joint2_placement.toActionMatrixInverse()*J_RF_local)).isApprox(-J_ref.middleRows<6>(0)));
  BOOST_CHECK(contact_datas[0].oMc1.isApprox(ci_RF.joint1_placement));
  
  BOOST_CHECK(contact_datas[1].c1Mc2.isApprox(c1Mc2_2));
  BOOST_CHECK((contact_datas[1].oMc1.toActionMatrixInverse()*J_LF).isApprox(-J_ref.middleRows<6>(6)));
  BOOST_CHECK((data_ref.oMi[ci_LF.joint2_id].toActionMatrix()*J_LF_local).isApprox(J_LF));
  BOOST_CHECK(contact_datas[1].oMc1.isApprox(ci_LF.joint1_placement));
  BOOST_CHECK(data.oa[ci_LF.joint2_id].isApprox(data_ref.oMi[ci_LF.joint2_id].act(data_ref.a[ci_LF.joint2_id])));
  
  BOOST_CHECK(contact_datas[2].c1Mc2.isApprox(c1Mc2_3));
  BOOST_CHECK((c1Mc2_3.toActionMatrix()*(ci_RA.joint2_placement.toActionMatrixInverse()*J_RA_local)).isApprox(-J_ref.middleRows<6>(12)));
  BOOST_CHECK(contact_datas[2].oMc1.isApprox(ci_RA.joint1_placement));
  BOOST_CHECK(data.oa[ci_RA.joint2_id].isApprox(data_ref.oMi[ci_RA.joint2_id].act(data_ref.a[ci_RA.joint2_id])));
  
  // Check that the decomposition is correct
  
  forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  forwardKinematics(model,data,q,v,data.ddq);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  Motion acc_1_final = c1Mc2_1_W.act(data.a[ci_RF.joint2_id]);
  BOOST_CHECK(acc_1_final.isZero());
                      
  std::cout << "acc_1_final:\n" << acc_1_final << std::endl;
  
  Motion acc_2_final = c1Mc2_2.act(data.a[ci_LF.joint2_id]);
  BOOST_CHECK(acc_2_final.isZero());
  
  std::cout << "acc_2_final:\n" << acc_2_final << std::endl;
  
  Motion acc_3_final = c1Mc2_3.act(data.a[ci_RA.joint2_id]);
  BOOST_CHECK(acc_3_final.isZero());
  
  std::cout << "acc_3_final:\n" << acc_3_final << std::endl;
  
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
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_bis;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_bis;
  
  contact_models_bis.push_back(ci_RF_bis);
  contact_models_bis.push_back(ci_LF_bis);
  contact_models_bis.push_back(ci_RA_bis);
  
  for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)::const_iterator it = contact_models_bis.begin();
      it != contact_models_bis.end(); ++it)
    contact_datas_bis.push_back(RigidContactData(*it));
  
  Data data_bis(model);
  initContactDynamics(model,data_bis,contact_models_bis);
  contactDynamics(model,data_bis,q,v,tau,contact_models_bis,contact_datas_bis,mu0);
  
  BOOST_CHECK(data_bis.ddq.isApprox(data.ddq));
  std::cout << "ddq: " << data_bis.ddq.transpose() << std::endl;
  std::cout << "ddq: " << data.ddq.transpose() << std::endl;
  
//  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_datas[k];
    const RigidContactModel & cmodel_bis = contact_models_bis[k];
    const RigidContactData & cdata_bis = contact_datas_bis[k];
    
    BOOST_CHECK(cmodel_bis.reference_frame == cmodel.reference_frame);
    BOOST_CHECK(cmodel_bis.joint1_id == cmodel.joint2_id);
    BOOST_CHECK(cmodel_bis.joint2_id == cmodel.joint1_id);
    BOOST_CHECK(cdata.oMc1.isApprox(cdata_bis.oMc2));
    BOOST_CHECK(cdata.oMc2.isApprox(cdata_bis.oMc1));
    BOOST_CHECK(cdata.c1Mc2.isApprox(cdata_bis.c1Mc2.inverse()));
    
    std::cout << "cdata.c1Mc2:\n" << cdata.c1Mc2 << std::endl;
    Force contact_force, contact_force_bis;
    switch(cmodel.reference_frame)
    {
      case LOCAL_WORLD_ALIGNED:
      {
        SE3 c1Mc2_LWA(SE3::Matrix3::Identity(),
                      cdata.oMc1.rotation()*cdata.c1Mc2.translation());
        contact_force_bis = cdata_bis.contact_force;
        BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(c1Mc2_LWA.actInv(cdata.contact2_acceleration_drift)));

        contact_force = c1Mc2_LWA.actInv(cdata.contact_force);
        BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
        break;
      }
      case LOCAL:
      {
        contact_force_bis = cdata_bis.contact_force;
        BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(cdata.c1Mc2.actInv(cdata.contact2_acceleration_drift)));

        contact_force = cdata.c1Mc2.actInv(cdata.contact_force);
        BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
        break;
      }
      case WORLD:
        BOOST_CHECK(false);
        break;
    }
    
    std::cout << "contact_force: " << contact_force.toVector().transpose() << std::endl;
    std::cout << "contact_force_bis: " << contact_force_bis.toVector().transpose() << std::endl;
  }
}

PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactData)
createData(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactModel) & contact_models)
{
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactData) contact_datas;
  for(size_t k = 0; k < contact_models.size(); ++k)
    contact_datas.push_back(pinocchio::RigidContactData(contact_models[k]));
  
  return contact_datas;
}

BOOST_AUTO_TEST_CASE(test_correction)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
//  pinocchio::buildModels::humanoidRandom(model,true);
  const JointIndex joint_id = model.addJoint(0,JointModelFreeFlyer(),SE3::Identity(),"root");
  const Inertia box_inertia = Inertia::FromBox(100.,1.,1.,1.);
  model.appendBodyToJoint(joint_id,box_inertia);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "root";
  const JointIndex RF_id = model.getJointId(RF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  
  RigidContactModel ci_RF(CONTACT_6D,RF_id,LOCAL);
  ci_RF.joint1_placement.setIdentity();
  ci_RF.joint2_placement.setIdentity();
  ci_RF.corrector.Kp = 10.;
  ci_RF.corrector.Kd = 2. * sqrt(ci_RF.corrector.Kp);
  contact_models.push_back(ci_RF);
  
//  RigidContactModel ci_LF(CONTACT_6D,LF_id,LOCAL);
//  ci_LF.joint1_placement.setRandom();
//  ci_LF.joint2_placement.setRandom();
//  ci_LF.corrector.Kp = 50.;
//  ci_LF.corrector.Kd = 2. * sqrt(ci_LF.corrector.Kp);
////  contact_models.push_back(ci_LF);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas = createData(contact_models);
  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas);
  
  BOOST_CHECK(contact_datas[0].oMc1.isApprox(data.oMi[ci_RF.joint1_id] * ci_RF.joint1_placement));
  BOOST_CHECK(contact_datas[0].oMc2.isApprox(data.oMi[ci_RF.joint2_id] * ci_RF.joint2_placement));
  BOOST_CHECK(contact_datas[0].contact1_velocity.isApprox(contact_datas[0].oMc1.actInv(data.ov[ci_RF.joint1_id])));
  BOOST_CHECK(contact_datas[0].contact2_velocity.isZero());
  
//  BOOST_CHECK(contact_datas[1].oMc1.isApprox(data.oMi[ci_LF.joint1_id] * ci_LF.joint1_placement));
//  BOOST_CHECK(contact_datas[1].oMc2.isApprox(data.oMi[ci_LF.joint2_id] * ci_LF.joint2_placement));
  
  const double dt = 1e-8;
  const VectorXd q_plus = integrate(model,q,v*dt);

  Data data_plus(model);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_plus = createData(contact_models);
  initContactDynamics(model,data_plus,contact_models);
  contactDynamics(model,data_plus,q_plus,v,tau,contact_models,contact_datas_plus);
  
  const Motion contact_RF_velocity_error_fd = log6(contact_datas[0].c1Mc2.act(contact_datas_plus[0].c1Mc2.inverse()))/dt;
  BOOST_CHECK(contact_RF_velocity_error_fd.isApprox(contact_datas[0].contact_velocity_error,sqrt(dt)));
  std::cout << "contact_RF_velocity_error_fd:\n" << contact_RF_velocity_error_fd << std::endl;
  std::cout << "contact_velocity_error:\n" << contact_datas[0].contact_velocity_error << std::endl;
  
  // Simulation loop
  {
    const int N = 200;
    const double dt = 1e-3;
    const double mu = 1e-12;
    
    model.gravity.setZero();
    Data data_sim(model);
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data_sim = createData(contact_models);
    initContactDynamics(model,data_sim,contact_models);
    
    Eigen::VectorXd q0 (model.nq);
    const SE3 M0 = SE3::Random();
    q0 << M0.translation(), SE3::Quaternion(M0.rotation()).coeffs();
    const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    
    Eigen::VectorXd q(q0), v(v0);
    
    tau = rnea(model,data_sim,q,v,0*a);
    contactDynamics(model,data_sim,q0,v0,tau,contact_models,contact_data_sim,mu);
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data_sim_prev(contact_data_sim);

    for(int it = 0; it <= N; it++)
    {
      a = contactDynamics(model,data_sim,q,v,tau,contact_models,contact_data_sim,mu);
      v += a*dt;
      std::cout << "a: " << a.transpose() << std::endl;
      std::cout << "v: " << v.transpose() << std::endl;
      q = integrate(model,q,v*dt);
      
      if(it > 1)
      {
        for(size_t k = 0; k < contact_models.size(); ++k)
        {
          const RigidContactData & cdata = contact_data_sim[k];
          const RigidContactData & cdata_prev = contact_data_sim_prev[k];
          
          std::cout << "cdata error: " << cdata.contact_placement_error.toVector().norm() << std::endl;
          std::cout << "cdata error prev: " << cdata_prev.contact_placement_error.toVector().norm() << std::endl;
          BOOST_CHECK(cdata.contact_placement_error.toVector().norm() <= cdata_prev.contact_placement_error.toVector().norm());
        }
      }
      
      contact_data_sim_prev = contact_data_sim;
    }
  }
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_specifying_joint2id_case3D)
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
  const std::string RA = "rarm6_joint";

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  
  RigidContactModel ci_RF(CONTACT_3D,0,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
  RigidContactModel ci_RF_bis(CONTACT_3D,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
  ci_RF.joint1_placement.setRandom();
  ci_RF.joint2_placement.setRandom();
  ci_RF_bis.joint1_placement = ci_RF.joint2_placement;
  ci_RF_bis.joint2_placement = ci_RF.joint1_placement;
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  
  RigidContactModel ci_LF(CONTACT_3D,0,model.getJointId(LF),LOCAL);
  RigidContactModel ci_LF_bis(CONTACT_3D,model.getJointId(LF),LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.joint2_placement.setRandom();
  ci_LF_bis.joint1_placement = ci_LF.joint2_placement;
  ci_LF_bis.joint2_placement = ci_LF.joint1_placement;
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidContactData(ci_LF));
  
  RigidContactModel ci_RA(CONTACT_6D,0,model.getJointId(RA),LOCAL);
  RigidContactModel ci_RA_bis(CONTACT_6D,model.getJointId(RA),LOCAL);
  ci_RA.joint1_placement.setRandom();
  ci_RA.joint2_placement.setRandom();
  ci_RA_bis.joint1_placement = ci_RA.joint2_placement;
  ci_RA_bis.joint2_placement = ci_RA.joint1_placement;
  contact_models.push_back(ci_RA);
  contact_datas.push_back(RigidContactData(ci_RA));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Data::Matrix6x J_RF(6,model.nv), J_LF(6,model.nv), J_RA(6,model.nv);
  J_RF.setZero(); J_LF.setZero(); J_RA.setZero();
  Data::Matrix6x J_RF_local(6,model.nv), J_LF_local(6,model.nv), J_RA_local(6,model.nv);
  J_RF_local.setZero(); J_LF_local.setZero(); J_RA_local.setZero();
  getJointJacobian(model,data_ref,
                   ci_RF.joint2_id,WORLD,
                   J_RF);
  getJointJacobian(model,data_ref,
                   ci_RF.joint2_id,LOCAL,
                   J_RF_local);
  J_RF_local = ci_RF.joint2_placement.toActionMatrixInverse() * J_RF_local;
  getJointJacobian(model,data_ref,
                   ci_LF.joint2_id,WORLD,
                   J_LF);
  getJointJacobian(model,data_ref,
                   ci_LF.joint2_id,LOCAL,
                   J_LF_local);
  J_LF_local = ci_LF.joint2_placement.toActionMatrixInverse() * J_LF_local;
  getJointJacobian(model,data_ref,
                   ci_RA.joint2_id,WORLD,
                   J_RA);
  getJointJacobian(model,data_ref,
                   ci_RA.joint2_id,LOCAL,
                   J_RA_local);
  
  {
    const SE3 oMc2 = data_ref.oMi[ci_RF.joint2_id]*ci_RF.joint2_placement;
    J_ref.middleRows<3>(0) = -oMc2.rotation() * J_RF_local.middleRows<3>(Motion::LINEAR);
  }

  {
    const SE3 oMc1 = data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement;
    const SE3 oMc2 = data_ref.oMi[ci_LF.joint2_id]*ci_LF.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    J_ref.middleRows<3>(3) = - c1Mc2.rotation() * J_LF_local.middleRows<3>(SE3::LINEAR);
  }
  
  {
    J_ref.middleRows<6>(6) = -(data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).toActionMatrixInverse() * J_RA;
  }
  
  Eigen::VectorXd rhs_ref(constraint_dim);

  forwardKinematics(model, data_ref, q, v, 0*v);
  Motion::Vector3 acc_1;
  {
    const SE3 oMc2 = data_ref.oMi[ci_RF.joint2_id]*ci_RF.joint2_placement;
    const Motion v2 = ci_RF.joint2_placement.actInv(data_ref.v[ci_RF.joint2_id]);
    const Motion a2 = ci_RF.joint2_placement.actInv(data_ref.a[ci_RF.joint2_id]);
    acc_1 = oMc2.rotation()*(a2.linear() + v2.angular().cross(v2.linear()));
  }
  
  Motion::Vector3 acc_2;
  {
    const SE3 oMc1 = data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement;
    const SE3 oMc2 = data_ref.oMi[ci_LF.joint2_id]*ci_LF.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const Motion v2 = ci_LF.joint2_placement.actInv(data_ref.v[ci_LF.joint2_id]);
    const Motion a2 = ci_LF.joint2_placement.actInv(data_ref.a[ci_LF.joint2_id]);
    acc_2 = c1Mc2.rotation()*(a2.linear() + v2.angular().cross(v2.linear()));
  }
  
  const SE3 c1Mc2_3 = (data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).actInv(data_ref.oMi[ci_RA.joint2_id]*ci_RA.joint2_placement);
  Motion acc_3 = c1Mc2_3.act(ci_RA.joint2_placement.actInv(data_ref.a[ci_RA.joint2_id]));

  rhs_ref.segment<3>(0) = -acc_1;
  rhs_ref.segment<3>(3) = -acc_2;
  rhs_ref.segment<6>(6) = -acc_3.toVector();

  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.topLeftCorner(constraint_dim,constraint_dim).diagonal().fill(-mu0);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();

  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,contact_datas,mu0);

  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();

  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.topRightCorner(constraint_dim,model.nv).isApprox(J_ref));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  std::cout << "KKT_matrix.topRightCorner(constraint_dim,model.nv):\n" << KKT_matrix.topRightCorner(constraint_dim,model.nv) << std::endl;
  std::cout << "KKT_matrix_ref.topRightCorner(constraint_dim,model.nv):\n" << KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) << std::endl;

  // Check solutions
  forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);
  forwardKinematics(model,data,q,v,data.ddq);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  std::cout << "data_ref.ddq: " << data_ref.ddq.transpose() << std::endl;
  std::cout << "data.ddq: " << data.ddq.transpose() << std::endl;
  std::cout << "res: " << (J_ref*data.ddq+rhs_ref).transpose() << std::endl;
  std::cout << "res_ref: " << (J_ref*data_ref.ddq+rhs_ref).transpose() << std::endl;
  
  const Motion vel_1_final = ci_RF.joint2_placement.actInv(data.v[ci_RF.joint2_id]);
  const Motion::Vector3 acc_1_final = ci_RF.joint2_placement.actInv(data.a[ci_RF.joint2_id]).linear() + vel_1_final.angular().cross(vel_1_final.linear());
  BOOST_CHECK(acc_1_final.isZero());

  std::cout << "acc_1_final:" << acc_1_final.transpose() << std::endl;
  
  const Motion vel_2_final = ci_LF.joint2_placement.actInv(data.v[ci_LF.joint2_id]);
  const Motion::Vector3 acc_2_final = ci_LF.joint2_placement.actInv(data.a[ci_LF.joint2_id]).linear() + vel_2_final.angular().cross(vel_2_final.linear());
  BOOST_CHECK(acc_2_final.isZero());
  
  std::cout << "acc_2_final:" << acc_2_final.transpose() << std::endl;
  
  Motion acc_3_final = c1Mc2_3.act(data.a[ci_RA.joint2_id]);
  BOOST_CHECK(acc_3_final.isZero());

  std::cout << "acc_3_final:\n" << acc_3_final << std::endl;

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

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_bis;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_bis;

  contact_models_bis.push_back(ci_RF_bis);
  contact_models_bis.push_back(ci_LF_bis);
  contact_models_bis.push_back(ci_RA_bis);

  for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)::const_iterator it = contact_models_bis.begin();
      it != contact_models_bis.end(); ++it)
    contact_datas_bis.push_back(RigidContactData(*it));

  Data data_bis(model);
  initContactDynamics(model,data_bis,contact_models_bis);
  contactDynamics(model,data_bis,q,v,tau,contact_models_bis,contact_datas_bis,mu0);

  BOOST_CHECK(data_bis.ddq.isApprox(data.ddq));
  std::cout << "ddq: " << data_bis.ddq.transpose() << std::endl;
  std::cout << "ddq: " << data.ddq.transpose() << std::endl;

  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidContactModel & cmodel = contact_models[k];
    const RigidContactData & cdata = contact_datas[k];
    const RigidContactModel & cmodel_bis = contact_models_bis[k];
    const RigidContactData & cdata_bis = contact_datas_bis[k];

    BOOST_CHECK(cmodel_bis.reference_frame == cmodel.reference_frame);
    BOOST_CHECK(cmodel_bis.joint1_id == cmodel.joint2_id);
    BOOST_CHECK(cmodel_bis.joint2_id == cmodel.joint1_id);
    BOOST_CHECK(cdata.oMc1.isApprox(cdata_bis.oMc2));
    BOOST_CHECK(cdata.oMc2.isApprox(cdata_bis.oMc1));
    BOOST_CHECK(cdata.c1Mc2.isApprox(cdata_bis.c1Mc2.inverse()));

    std::cout << "cdata.c1Mc2:\n" << cdata.c1Mc2 << std::endl;
    Force contact_force, contact_force_bis;
    switch(cmodel.reference_frame)
    {
      case LOCAL_WORLD_ALIGNED:
      {
        SE3 c1Mc2_LWA(SE3::Matrix3::Identity(),
                      cdata.oMc1.rotation()*cdata.c1Mc2.translation());
        contact_force_bis = cdata_bis.contact_force;

        if(cmodel.type == CONTACT_3D)
          contact_force = cdata.contact_force;
        else
        {
          contact_force = c1Mc2_LWA.actInv(cdata.contact_force);
          BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(c1Mc2_LWA.actInv(cdata.contact2_acceleration_drift)));
        }
        BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
        break;
      }
      case LOCAL:
      {
        contact_force_bis = cdata_bis.contact_force;

        if(cmodel.type == CONTACT_3D)
          contact_force.linear() = cdata.c1Mc2.actInv(cdata.contact_force).linear();
        else
        {
          contact_force = cdata.c1Mc2.actInv(cdata.contact_force);
          BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(cdata.c1Mc2.actInv(cdata.contact2_acceleration_drift)));
        }
        BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
        break;
      }
      case WORLD:
        BOOST_CHECK(false);
        break;
    }

    std::cout << "contact_force: " << contact_force.toVector().transpose() << std::endl;
    std::cout << "contact_force_bis: " << contact_force_bis.toVector().transpose() << std::endl;
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
//  const Frame & RF_frame = model.frames[model.getJointId(RF)];
//  Frame RF_contact_frame("RF_contact_frame",
//                         RF_frame.parent,model.getJointId(RF),
//                         SE3::Random(),OP_FRAME);
//  model.addFrame(RF_contact_frame);
  
  const std::string LF = "lleg6_joint";
//  const Frame & LF_frame = model.frames[model.getJointId(LF)];
//  Frame LF_contact_frame("LF_contact_frame",
//                         LF_frame.parent,model.getJointId(RF),
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
  RigidContactModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
  ci_LF.joint1_placement.setRandom();
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
  getJointJacobian(model,data_ref,
                   ci_RF.joint1_id,ci_RF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix()*Jtmp;
  
  Jtmp.setZero();
  getJointJacobian(model,data_ref,
                   ci_LF.joint1_id,ci_LF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(6) = ci_LF.joint1_placement.inverse().toActionMatrix()*Jtmp;
  
  Eigen::VectorXd gamma(constraint_dim);
  
  gamma.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type,ci_RF.joint1_placement).toVector();
  gamma.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type,ci_LF.joint1_placement).toVector();
  
  BOOST_CHECK((J_ref*data_ref.ddq + gamma).isZero());
  
  Data data_constrained_dyn(model);
  forwardDynamics(model,data_constrained_dyn,q,v,tau,J_ref,gamma,mu0);
  BOOST_CHECK((J_ref*data_constrained_dyn.ddq + gamma).isZero());
  
  ProximalSettings prox_settings;
  prox_settings.max_iter = 10;
  prox_settings.mu = 1e8;
  const double mu = prox_settings.mu;
  contactABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  
  std::cout << "data.ddq: " << data.ddq.transpose() << std::endl;
  std::cout << "data_ref.ddq: " << data_ref.ddq.transpose() << std::endl;
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

    const RigidContactModel::JointIndex & joint1_id = cmodel.joint1_id;
    
    // Check contact placement
    const SE3 & iMc = cmodel.joint1_placement;
    const SE3 oMc = data_ref.oMi[joint1_id] * iMc;
    BOOST_CHECK(cdata.oMc1.isApprox(oMc));
    
    // Check contact velocity
    const Motion contact1_velocity_ref = iMc.actInv(data_ref.v[joint1_id]);
    BOOST_CHECK(cdata.contact1_velocity.isApprox(contact1_velocity_ref));
    
    // Check contact inertia
    Symmetric3 S(Symmetric3::Zero());
    if(cmodel.type == CONTACT_6D)
      S.setDiagonal(Symmetric3::Vector3::Constant(mu));
    
    const Inertia contact_inertia(mu,oMc.translation(),S);
    
    Inertia::Matrix6 contact_inertia_ref = Inertia::Matrix6::Zero();
    
    if(cmodel.type == CONTACT_6D)
      contact_inertia_ref.diagonal().fill(mu);
    else
      contact_inertia_ref.diagonal().head<3>().fill(mu);
    contact_inertia_ref = oMc.toDualActionMatrix() * contact_inertia_ref * oMc.toActionMatrixInverse();
    BOOST_CHECK(contact_inertia_ref.isApprox(contact_inertia.matrix()));
    
    Inertia::Matrix6 Yaba_ref
    = data_ref.oMi[joint1_id].toDualActionMatrix() * model.inertias[joint1_id].matrix() * data_ref.oMi[joint1_id].toActionMatrixInverse()
    + contact_inertia_ref;
    
    const JointModel & jmodel = model.joints[joint1_id];
    const JointData & jdata = data.joints[joint1_id];
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
    
    BOOST_CHECK(data.oYaba[joint1_id].isApprox(Yaba_ref));
    
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
  RigidContactModel ci_RF(CONTACT_3D,model.getJointId(RF),LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidContactData(ci_RF));
  RigidContactModel ci_LF(CONTACT_3D,model.getJointId(LF),LOCAL);
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
  
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  getJointJacobian(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,Jtmp);
  J_ref.middleRows<3>(0) = Jtmp.middleRows<3>(Motion::LINEAR);
  Jtmp.setZero(); getJointJacobian(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,Jtmp);
  J_ref.middleRows<3>(3) = Jtmp.middleRows<3>(Motion::LINEAR);
  
  Eigen::VectorXd gamma(constraint_dim);
  
  gamma.segment<3>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type).linear();
  gamma.segment<3>(3) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type).linear();
  
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

BOOST_AUTO_TEST_SUITE_END()
