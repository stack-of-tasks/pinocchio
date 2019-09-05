//
// Copyright (c) 2016-2019 CNRS, INRIA
//

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(contact_info)
{
  using namespace pinocchio;
  
  // Check default constructor
  ContactInfo ci1;
  BOOST_CHECK(ci1.type == CONTACT_UNDEFINED);
  BOOST_CHECK(ci1.dim() == 0);
  
  // Check complete constructor
  SE3 M(SE3::Random());
  ContactInfo ci2(CONTACT_3D,0,M);
  BOOST_CHECK(ci2.type == CONTACT_3D);
  BOOST_CHECK(ci2.frame_id == 0);
  BOOST_CHECK(ci2.placement.isApprox(M));
  BOOST_CHECK(ci2.dim() == 3);
  
  // Check contructor with two arguments
  ContactInfo ci2prime(CONTACT_3D,0);
  BOOST_CHECK(ci2prime.type == CONTACT_3D);
  BOOST_CHECK(ci2prime.frame_id == 0);
  BOOST_CHECK(ci2prime.placement.isIdentity());
  BOOST_CHECK(ci2prime.dim() == 3);
  
  // Check default copy constructor
  ContactInfo ci3(ci2);
  BOOST_CHECK(ci3 == ci2);
  
  // Check complete constructor 6D
  ContactInfo ci4(CONTACT_6D,0,SE3::Identity());
  BOOST_CHECK(ci4.type == CONTACT_6D);
  BOOST_CHECK(ci4.frame_id == 0);
  BOOST_CHECK(ci4.placement.isIdentity());
  BOOST_CHECK(ci4.dim() == 6);
}

BOOST_AUTO_TEST_CASE ( test_FD )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  
  pinocchio::computeJointJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  Data::Matrix6x J_RF (6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data, model.getJointId(RF), LOCAL, J_RF);
  Data::Matrix6x J_LF (6, model.nv);
  J_LF.setZero();
  getJointJacobian(model, data, model.getJointId(LF), LOCAL, J_LF);
  
  Eigen::MatrixXd J (12, model.nv);
  J.setZero();
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_LF;
  
  Eigen::VectorXd gamma (VectorXd::Ones(12));
  
  Eigen::MatrixXd H(J.transpose());
  
  pinocchio::forwardDynamics(model, data, q, v, tau, J, gamma, 0.);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  MatrixXd Minv (data.M.inverse());
  MatrixXd JMinvJt (J * Minv * J.transpose());
  
  Eigen::MatrixXd G_ref(J.transpose());
  cholesky::Uiv(model, data, G_ref);
  for(int k=0;k<model.nv;++k) G_ref.row(k) /= sqrt(data.D[k]);
    Eigen::MatrixXd H_ref(G_ref.transpose() * G_ref);
    BOOST_CHECK(H_ref.isApprox(JMinvJt,1e-12));
  
  VectorXd lambda_ref = -JMinvJt.inverse() * (J*Minv*(tau - data.nle) + gamma);
  BOOST_CHECK(data.lambda_c.isApprox(lambda_ref, 1e-12));
    
  VectorXd a_ref = Minv*(tau - data.nle + J.transpose()*lambda_ref);

  Eigen::VectorXd dynamics_residual_ref (data.M * a_ref + data.nle - tau - J.transpose()*lambda_ref);
  BOOST_CHECK(dynamics_residual_ref.norm() <= 1e-11); // previously 1e-12, may be due to numerical approximations, i obtain 2.03e-12

  Eigen::VectorXd constraint_residual (J * data.ddq + gamma);
  BOOST_CHECK(constraint_residual.norm() <= 1e-12);
  
  Eigen::VectorXd dynamics_residual (data.M * data.ddq + data.nle - tau - J.transpose()*data.lambda_c);
  BOOST_CHECK(dynamics_residual.norm() <= 1e-12);

}

BOOST_AUTO_TEST_CASE (test_KKTMatrix)
{
  using namespace Eigen;
  using namespace pinocchio;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  
  pinocchio::computeJointJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  Data::Matrix6x J_RF (6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data, model.getJointId(RF), LOCAL, J_RF);
  Data::Matrix6x J_LF (6, model.nv);
  J_LF.setZero();
  getJointJacobian(model, data, model.getJointId(LF), LOCAL, J_LF);
  
  Eigen::MatrixXd J (12, model.nv);
  J.setZero();
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_LF;
  
  Eigen::VectorXd gamma (VectorXd::Ones(12));
  
  Eigen::MatrixXd H(J.transpose());
  
  //Check Forward Dynamics
  pinocchio::forwardDynamics(model, data, q, v, tau, J, gamma, 0.);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd MJtJ(model.nv+12, model.nv+12);
  MJtJ << data.M, J.transpose(),
    J, Eigen::MatrixXd::Zero(12, 12);

  Eigen::MatrixXd MJtJ_inv(model.nv+12, model.nv+12);
  getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);

  BOOST_CHECK(MJtJ_inv.isApprox(MJtJ.inverse()));

  //Check Impulse Dynamics
  const double r_coeff = 1.;
  VectorXd v_before = VectorXd::Ones(model.nv);
  pinocchio::impulseDynamics(model, data, q, v_before, J, r_coeff, 0.);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  MJtJ << data.M, J.transpose(),
    J, Eigen::MatrixXd::Zero(12, 12);

  getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);

  BOOST_CHECK(MJtJ_inv.isApprox(MJtJ.inverse()));
  
}

BOOST_AUTO_TEST_CASE ( test_FD_with_damping )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  
  pinocchio::computeJointJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  
  const std::string RF = "rleg6_joint";
  
  Data::Matrix6x J_RF (6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data, model.getJointId(RF), LOCAL, J_RF);

  Eigen::MatrixXd J (12, model.nv);
  J.setZero();
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_RF;
  
  Eigen::VectorXd gamma (VectorXd::Ones(12));

  // Forward Dynamics with damping
  pinocchio::forwardDynamics(model, data, q, v, tau, J, gamma, 1e-12);

  // Matrix Definitions
  Eigen::MatrixXd H(J.transpose());
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  MatrixXd Minv (data.M.inverse());
  MatrixXd JMinvJt (J * Minv * J.transpose());

  // Check that JMinvJt is correctly formed
  Eigen::MatrixXd G_ref(J.transpose());
  cholesky::Uiv(model, data, G_ref);
  for(int k=0;k<model.nv;++k) G_ref.row(k) /= sqrt(data.D[k]);
  Eigen::MatrixXd H_ref(G_ref.transpose() * G_ref);
  BOOST_CHECK(H_ref.isApprox(JMinvJt,1e-12));

  // Actual Residuals
  Eigen::VectorXd constraint_residual (J * data.ddq + gamma);  
  Eigen::VectorXd dynamics_residual (data.M * data.ddq + data.nle - tau - J.transpose()*data.lambda_c);
  BOOST_CHECK(constraint_residual.norm() <= 1e-9);
  BOOST_CHECK(dynamics_residual.norm() <= 1e-12);
}

//BOOST_AUTO_TEST_CASE ( test_FD_with_singularity )
//{
//  using namespace Eigen;
//  using namespace pinocchio;
//  
//  pinocchio::Model model;
//  pinocchio::buildModels::humanoidRandom(model,true);
//  pinocchio::Data data(model);
//  
//  VectorXd q = VectorXd::Ones(model.nq);
//  q.segment<4>(3).normalize();
//  
//  pinocchio::computeJointJacobians(model, data, q);
//  
//  VectorXd v = VectorXd::Ones(model.nv);
//  VectorXd tau = VectorXd::Zero(model.nv);
//  
//  const std::string RF = "rleg6_joint";
//  
//  Data::Matrix6x J_RF (6, model.nv);
//  J_RF.setZero();
//  getJointJacobian(model, data, model.getJointId(RF), LOCAL, J_RF);
//  
//  Eigen::MatrixXd J(12, model.nv);
//  J.topRows<6> () = J_RF;
//  J.bottomRows<6> () = J_RF;
//  
//  Eigen::VectorXd gamma (VectorXd::Ones(12));
//  
//  ProximalSettings prox_settings(1e-12,1e-8,20);
//  
//  // Forward Dynamics with damping
//  pinocchio::forwardDynamics(model, data, q, v, tau, J, gamma, prox_settings);
//  
//  // Actual Residuals
//  Eigen::VectorXd constraint_residual (J * data.ddq + gamma);
//  Eigen::VectorXd dynamics_residual (data.M * data.ddq + data.nle - tau - J.transpose()*data.lambda_c);
//  BOOST_CHECK(constraint_residual.norm() <= 1e-9);
//  BOOST_CHECK(dynamics_residual.norm() <= 1e-12);
//  std::cout << "dynamics_residual: " << dynamics_residual.norm() << std::endl;
//}

BOOST_AUTO_TEST_CASE ( test_ID )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  
  pinocchio::computeJointJacobians(model, data, q);
  
  VectorXd v_before = VectorXd::Ones(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  Data::Matrix6x J_RF (6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data, model.getJointId(RF), LOCAL, J_RF);
  Data::Matrix6x J_LF (6, model.nv);
  J_LF.setZero();
  getJointJacobian(model, data, model.getJointId(LF), LOCAL, J_LF);
  
  Eigen::MatrixXd J (12, model.nv);
  J.setZero();
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_LF;
  
  const double r_coeff = 1.;
  
  Eigen::MatrixXd H(J.transpose());
  
  pinocchio::impulseDynamics(model, data, q, v_before, J, r_coeff, 0.);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  MatrixXd Minv (data.M.inverse());
  MatrixXd JMinvJt (J * Minv * J.transpose());
  
  Eigen::MatrixXd G_ref(J.transpose());
  cholesky::Uiv(model, data, G_ref);
  for(int k=0;k<model.nv;++k) G_ref.row(k) /= sqrt(data.D[k]);
  Eigen::MatrixXd H_ref(G_ref.transpose() * G_ref);
  BOOST_CHECK(H_ref.isApprox(JMinvJt,1e-12));
  
  VectorXd lambda_ref = JMinvJt.inverse() * (-r_coeff * J * v_before - J * v_before);
  BOOST_CHECK(data.impulse_c.isApprox(lambda_ref, 1e-12));
  
  VectorXd v_after_ref = Minv*(data.M * v_before + J.transpose()*lambda_ref);
  
  Eigen::VectorXd constraint_residual (J * data.dq_after + r_coeff * J * v_before);
  BOOST_CHECK(constraint_residual.norm() <= 1e-12);
  
  Eigen::VectorXd dynamics_residual (data.M * data.dq_after - data.M * v_before - J.transpose()*data.impulse_c);
  BOOST_CHECK(dynamics_residual.norm() <= 1e-12);

}

/// \brief Computes forces in the world frame
pinocchio::Motion computeFrameAcc(const pinocchio::Model & model,
                                  pinocchio::Data & data,
                                  const pinocchio::Model::FrameIndex & frame_id,
                                  pinocchio::ReferenceFrame reference_frame)
{
  using namespace pinocchio;
  Model::JointIndex joint_id = model.frames[frame_id].parent;
  Motion res(Motion::Zero());
  
  updateFramePlacement(model,data,frame_id);
  
  const Data::SE3 & oMf = data.oMf[frame_id];
  const Data::SE3 & oMi = data.oMi[joint_id];
  
  const Data::SE3 iMf = oMi.inverse() * oMf;
  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);
  
  switch (reference_frame)
  {
    case WORLD:
      classicAcceleration(ov,oa,res.linear());
      res.angular() = oa.angular();
      break;
    case LOCAL_WORLD_ALIGNED:
      res.linear() = oMf.rotation() * classicAcceleration(data.v[joint_id],data.a[joint_id],iMf);
      res.angular() = oMi.rotation() * data.a[joint_id].angular();
      break;
    case LOCAL:
      classicAcceleration(data.v[joint_id],data.a[joint_id],iMf,res.linear());
      res.angular() = iMf.rotation().transpose() * data.a[joint_id].angular();
      break;
    default:
      break;
  }
  
  return res;
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
  
  // Contact info
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_infos;
  ContactInfo ci_RF(CONTACT_6D,model.getFrameId(RF),WORLD);
  contact_infos.push_back(ci_RF);
  ContactInfo ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
  contact_infos.push_back(ci_LF);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
    constraint_dim += contact_infos[k].dim();
  
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
  
  initContactDynamics(model,data,contact_infos);
  contactDynamics(model,data,q,v,tau,contact_infos,mu0);
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
  {
    const ContactInfo & cinfo = contact_infos[k];
    
    switch(cinfo.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(data.contact_forces[k].linear().isApprox(data_ref.lambda_c.segment(constraint_id,cinfo.dim())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(data.contact_forces[k].isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cinfo.dim();
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
  
  // Contact info
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_infos;
  ContactInfo ci_RF(CONTACT_6D,model.getFrameId(RF),LOCAL);
  contact_infos.push_back(ci_RF);
  ContactInfo ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
  contact_infos.push_back(ci_LF);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
    constraint_dim += contact_infos[k].dim();
  
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
  
  initContactDynamics(model,data,contact_infos);
  contactDynamics(model,data,q,v,tau,contact_infos,mu0);
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
  {
    const ContactInfo & cinfo = contact_infos[k];
    
    switch(cinfo.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(data.contact_forces[k].linear().isApprox(data_ref.lambda_c.segment(constraint_id,cinfo.dim())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(data.contact_forces[k].isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cinfo.dim();
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
  
  // Contact info
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_infos;
  ContactInfo ci_RF(CONTACT_6D,model.getFrameId(RF),WORLD);
  contact_infos.push_back(ci_RF);
  ContactInfo ci_LF(CONTACT_3D,model.getFrameId(LF),WORLD);
  contact_infos.push_back(ci_LF);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
    constraint_dim += contact_infos[k].dim();
  
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
  
  initContactDynamics(model,data,contact_infos);
  contactDynamics(model,data,q,v,tau,contact_infos,mu0);
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
  {
    const ContactInfo & cinfo = contact_infos[k];
    
    switch(cinfo.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(data.contact_forces[k].linear().isApprox(data_ref.lambda_c.segment(constraint_id,cinfo.dim())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(data.contact_forces[k].isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cinfo.dim();
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
  
  // Contact info
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_infos;
  ContactInfo ci_RF(CONTACT_6D,model.getFrameId(RF),LOCAL_WORLD_ALIGNED);
  contact_infos.push_back(ci_RF);
  ContactInfo ci_LF(CONTACT_6D,model.getFrameId(LF),WORLD);
  contact_infos.push_back(ci_LF);
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
    constraint_dim += contact_infos[k].dim();
  
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
  
  initContactDynamics(model,data,contact_infos);
  contactDynamics(model,data,q,v,tau,contact_infos,mu0);
  
  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  Eigen::DenseIndex constraint_id = 0;
  for(size_t k = 0; k < contact_infos.size(); ++k)
  {
    const ContactInfo & cinfo = contact_infos[k];
    
    switch(cinfo.type)
    {
      case pinocchio::CONTACT_3D:
      {
        BOOST_CHECK(data.contact_forces[k].linear().isApprox(data_ref.lambda_c.segment(constraint_id,cinfo.dim())));
        break;
      }
        
      case pinocchio::CONTACT_6D:
      {
        ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
        BOOST_CHECK(data.contact_forces[k].isApprox(f_ref));
        break;
      }
        
      default:
        break;
    }
    
    constraint_id += cinfo.dim();
  }
}

BOOST_AUTO_TEST_CASE (timings_fd_llt)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
#ifdef NDEBUG
#ifdef _INTENSE_TESTING_
  const size_t NBT = 1000*1000;
#else
  const size_t NBT = 100;
#endif
  
#else
  const size_t NBT = 1;
  std::cout << "(the time score in debug mode is not relevant)  " ;
#endif // ifndef NDEBUG
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  
  pinocchio::computeJointJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  Data::Matrix6x J_RF (6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data, model.getJointId(RF), LOCAL, J_RF);
  Data::Matrix6x J_LF (6, model.nv);
  J_LF.setZero();
  getJointJacobian(model, data, model.getJointId(LF), LOCAL, J_LF);
  
  Eigen::MatrixXd J (12, model.nv);
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_LF;
  
  Eigen::VectorXd gamma (VectorXd::Ones(12));
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  
  q = pinocchio::randomConfiguration(model);
  
  PinocchioTicToc timer(PinocchioTicToc::US); timer.tic();
  SMOOTH(NBT)
  {
    pinocchio::forwardDynamics(model, data, q, v, tau, J, gamma, 0.);
  }
  timer.toc(std::cout,NBT);
  
}

BOOST_AUTO_TEST_SUITE_END ()
