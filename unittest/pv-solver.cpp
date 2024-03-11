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
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"
#include "pinocchio/algorithm/pv_solver.hpp"

#include <iostream>
#include "pinocchio/parsers/urdf.hpp"
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)



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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) empty_contact_datas;
  
  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);

  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv,model.nv);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
  pv_settings.mu = 0.0;
  initPvSolver(model, data, empty_contact_models);
  pv(model, data, q, v, tau, empty_contact_models, empty_contact_datas, prox_settings, pv_settings);
  
  // initConstraintDynamics(model,data,empty_contact_models);
  // constraintDynamics(model,data,q,v,tau,empty_contact_models,empty_contact_datas,prox_settings);
  
  // data.M.triangularView<Eigen::StrictlyLower>() =
  // data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  // Data data_ag(model);
  // ccrba(model,data_ag,q,v);
  
  // BOOST_CHECK(data.J.isApprox(data_ref.J));
  // BOOST_CHECK(data.M.isApprox(data_ref.M));
  // BOOST_CHECK(data.Ag.isApprox(data_ag.Ag));
  // BOOST_CHECK(data.nle.isApprox(data_ref.nle));
  
  // for(Model::JointIndex k = 1; k < model.joints.size(); ++k)
  // {
  //   BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
  //   BOOST_CHECK(data.liMi[k].isApprox(data_ref.liMi[k]));
  //   BOOST_CHECK(data.ov[k].isApprox(data_ref.oMi[k].act(data_ref.v[k])));
  //   BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oMi[k].act(data_ref.a_gf[k])));
  // }
  
  // Check that the decomposition is correct
  // const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  // Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  // BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  // BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  aba(model,data_ref,q,v,tau);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  // Checking if solving again the same problem gives the same solution
  pv(model, data, q, v, tau, empty_contact_models, empty_contact_datas, prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  pv_settings.use_early = true;
  pv_settings.mu = 1e-5;
  // initPvSolver(model, data, empty_contact_models);
  pv(model, data, q, v, tau, empty_contact_models, empty_contact_datas, prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  std::cout << "Error for iiwa (pv) = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";

}

// BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_double_init)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
//   pinocchio::buildModels::humanoidRandom(model,true);
//   pinocchio::Data data1(model), data2(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const std::string RF = "rleg6_joint";
//   //  const Model::JointIndex RF_id = model.getJointId(RF);
//   const std::string LF = "lleg6_joint";

//   // Contact models and data
//   const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_empty;
  
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_6D;
  
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D6D;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_6D6D;
  
//   RigidConstraintModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL);
//   contact_models_6D.push_back(ci_RF);
//   contact_datas_6D.push_back(RigidConstraintData(ci_RF));
//   contact_models_6D6D.push_back(ci_RF);
//   contact_datas_6D6D.push_back(RigidConstraintData(ci_RF));
//   RigidConstraintModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
//   contact_models_6D6D.push_back(ci_LF);
//   contact_datas_6D6D.push_back(RigidConstraintData(ci_LF));
  
//   const double mu0 = 0.;
//   ProximalSettings prox_settings(1e-12,mu0,1);
  
//   initConstraintDynamics(model,data1,contact_models_empty);
//   BOOST_CHECK(data1.contact_chol.size() == (model.nv + 0));
//   constraintDynamics(model,data1,q,v,tau,contact_models_empty,contact_datas_empty,prox_settings);
//   BOOST_CHECK(!hasNaN(data1.ddq));
  
//   initConstraintDynamics(model,data1,contact_models_6D);
//   BOOST_CHECK(data1.contact_chol.size() == (model.nv + 1*6));
//   constraintDynamics(model,data1,q,v,tau,contact_models_6D,contact_datas_6D,prox_settings);
//   BOOST_CHECK(!hasNaN(data1.ddq));
  
//   initConstraintDynamics(model,data1,contact_models_6D6D);
//   BOOST_CHECK(data1.contact_chol.size() == (model.nv + 2*6));
//   constraintDynamics(model,data1,q,v,tau,contact_models_6D6D,contact_datas_6D6D,prox_settings);
//   BOOST_CHECK(!hasNaN(data1.ddq));
  
//   initConstraintDynamics(model,data2,contact_models_6D6D);
//   initConstraintDynamics(model,data2,contact_models_6D);
//   initConstraintDynamics(model,data2,contact_models_empty);
// }

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D_LOCAL)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  // pinocchio::urdf::buildModel("/home/ajay/mlibs/pinocchio/iiwa2.urdf", model);
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/iiwa2.urdf");
  pinocchio::urdf::buildModel(filename,model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  // q = randomConfiguration(model); // resampling because previous one seems to be a challenging case
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "iiwa_joint_7";
   const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D,model, RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  // RigidConstraintModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
  // ci_LF.joint1_placement.setRandom();
  // contact_models.push_back(ci_LF);
  // contact_datas.push_back(RigidConstraintData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.0; //TODO: set to 0
  
  // computeAllTerms(model,data_ref,q,v);
  // data_ref.M.triangularView<Eigen::StrictlyLower>() =
  // data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  // Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  // J_ref.setZero();
  // Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  // getJointJacobian(model,data_ref,
  //                  ci_RF.joint1_id,ci_RF.reference_frame,
  //                  Jtmp);
  // J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  // Jtmp.setZero();
  // getJointJacobian(model,data_ref,
  //                  ci_LF.joint1_id,ci_LF.reference_frame,
  //                  Jtmp);
  // J_ref.middleRows<6>(6) = ci_LF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  // Eigen::VectorXd rhs_ref(constraint_dim);
  // rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type,ci_RF.joint1_placement).toVector();
  // rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type,ci_LF.joint1_placement).toVector();
  
  // Eigen::MatrixXd KKT_matrix_ref
  // = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  // KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  // KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  // KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  // forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  // forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  // BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());

  ProximalSettings prox_settings(1e-12,mu0,1); // TODO: set max_iter to 1
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
  pv_settings.mu = 0.0;
  pv_settings.absolute_accuracy = 1e-14;
  pv_settings.max_iter = 1;
  initPvSolver(model,data,contact_models);
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  
  // Check solutions
  auto ddq_err = data_ref.ddq - data.ddq;
  std::cout << "Error for iiwa = " << std::sqrt(ddq_err.dot(ddq_err)) << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));


  pv_settings.use_early = true;
  pv_settings.mu = 1e-4;
  pv_settings.absolute_accuracy = 1e-14;
  pv_settings.max_iter = 10;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  auto ddq_err2 = data_ref.ddq - data.ddq;
  std::cout << "Error for iiwa = " << std::sqrt(ddq_err2.dot(ddq_err2)) << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  prox_settings.max_iter = 10;
  prox_settings.mu = 1e4;
  proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
    // std::cout << "M (proxLTLs) = " <<  data.M << std::endl;
    // std::cout << "qdd LTLs = " <<  data.ddq << std::endl;
    // std::cout << "qdd LTL = " <<  data_ref.ddq << std::endl;
  std::cout << "Error for iiwa (proxLTLs) = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";

  aba(model,data_ref,q,v,tau);

  prox_settings.max_iter = 1;
  contact_models.pop_back();
  contact_datas.pop_back();
  proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  std::cout << "Error for iiwa (proxLTLs) = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
}


BOOST_AUTO_TEST_CASE(test_forward_dynamics_in_contact_6D_LOCAL_humanoid)
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
   const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
   const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D,model, LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  
  // Eigen::DenseIndex constraint_dim = 0;
  // for(size_t k = 0; k < contact_models.size(); ++k)
  //   constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.0;
  
  // computeAllTerms(model,data_ref,q,v);
  // data_ref.M.triangularView<Eigen::StrictlyLower>() =
  // data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  // Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  // J_ref.setZero();
  // Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  // getJointJacobian(model,data_ref,
  //                  ci_RF.joint1_id,ci_RF.reference_frame,
  //                  Jtmp);
  // J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  // Jtmp.setZero();
  // getJointJacobian(model,data_ref,
  //                  ci_LF.joint1_id,ci_LF.reference_frame,
  //                  Jtmp);
  // J_ref.middleRows<6>(6) = ci_LF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  // Eigen::VectorXd rhs_ref(constraint_dim);
  // rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type,ci_RF.joint1_placement).toVector();
  // rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type,ci_LF.joint1_placement).toVector();
  
  // Eigen::MatrixXd KKT_matrix_ref
  // = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  // KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  // KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  // KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  // forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  // forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  // BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());

  ProximalSettings prox_settings(1e-12,mu0,1);
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  
  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
   pv_settings.max_iter = 1;
  pv_settings.mu = 0.0;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  std::cout << "Error for pv = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  // pv_settings.use_early = true;
  //  pv_settings.max_iter = 1;
  // pv_settings.mu = 1e-3;
  // pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  // std::cout << "Error for pv = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  // auto con_Residual = J_ref*data.ddq+rhs_ref;
  // BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());

  prox_settings.max_iter = 10;
  prox_settings.mu = 1e4;
  data.ddq.setZero();
  proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout << "Error for proxLTLs = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  prox_settings.mu = 0.0;
  
  // Check that the decomposition is correct
  // const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  // Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  // BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  // BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  // auto ddq_err = (data.ddq - data_ref.ddq).eval();
  

  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  pv_settings.mu = 0.0;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  pv_settings.use_early = true;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-3;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout<< "Talos cABA err = " << std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq)) << std::endl;

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout<< "Talos cABA err = " << std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq)) << std::endl;
  // Send non-zero mu and max 1 iteration and the solution should not match
  // prox_settings.mu = 1e-6;
  // pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  // BOOST_CHECK(!data.ddq.isApprox(data_ref.ddq, 1e-10));

  // Change max iter to 10 and now should work
  // prox_settings.max_iter = 20;
  

}

BOOST_AUTO_TEST_CASE(test_forward_dynamics_3D_humanoid)
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
  const Model::JointIndex RF_id = model.getJointId(RF);
    
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_3D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
    
  const double mu0 = 0.0;
  
  ProximalSettings prox_settings(1e-12,mu0,1);
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  
  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
   pv_settings.max_iter = 1;
  pv_settings.mu = 0.0;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  std::cout << "Error for pv = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  prox_settings.max_iter = 10;
  prox_settings.mu = 1e4;
  data.ddq.setZero();
  proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout << "Error for proxLTLs = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  prox_settings.mu = 0.0;
  
  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  pv_settings.mu = 0.0;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  initPvSolver(model,data,contact_models);
  pv_settings.use_early = true;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-3;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout << "data.ddq = " << data.ddq.transpose() << "\ndata_ref.ddq = " << data_ref.ddq.transpose() << "\n";
  std::cout<< "Talos cABA err = " << std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq)) << std::endl;

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
 
}

BOOST_AUTO_TEST_CASE(test_forward_dynamics_repeating_3D_humanoid)
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
  const Model::JointIndex RF_id = model.getJointId(RF);
    
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kd.setZero();
  ci_RF.corrector.Kp.setZero();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_RF2(CONTACT_6D,model,model.getJointId("rleg5_joint"),LOCAL);
  ci_RF2.joint1_placement.setRandom();
  ci_RF2.corrector.Kd.setZero();
  ci_RF2.corrector.Kp.setZero();
  contact_models.push_back(ci_RF2);
  contact_datas.push_back(RigidConstraintData(ci_RF2));
    
  const double mu0 = 1e-4;
  
  ProximalSettings prox_settings(1e-14,mu0,10);
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);

  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();   
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  getJointJacobian(model,data_ref,
                   ci_RF.joint1_id,ci_RF.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  Jtmp.setZero();
  getJointJacobian(model,data_ref,
                   ci_RF2.joint1_id,ci_RF2.reference_frame,
                   Jtmp);
  J_ref.middleRows<6>(6) = ci_RF2.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type,ci_RF.joint1_placement).toVector();
  rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_RF2.joint1_id,ci_RF2.reference_frame,ci_RF2.type,ci_RF2.joint1_placement).toVector();
  
  // Eigen::MatrixXd KKT_matrix_ref
  // = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  // KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  // KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  // KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  // forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  // forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  // std::cout << "Delassus matrix = " << data_ref.contact_chol.getInverseOperationalSpaceInertiaMatrix() << "\n";
  std::cout << "ProxLTL L2 residual in tests = " << (J_ref.transpose()*(J_ref*data_ref.ddq+rhs_ref)).template lpNorm<2>() << "\n";
  BOOST_CHECK((J_ref.transpose()*(J_ref*data_ref.ddq+rhs_ref)).isZero(1e-10));

  // contactABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  // BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  // std::cout << "data.ddq = " << data.ddq.transpose() << "\ndata_ref.ddq = " << data_ref.ddq.transpose() << "\n";
  
  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  // std::cout << "data.ddq = " << data.ddq.transpose() << "\ndata_ref.ddq = " << data_ref.ddq.transpose() << "\n";
  // std::cout << "PV Delassus matrix = " << data.LA[0] << "\n";
  // std::cout << "Error for pv now = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  BOOST_CHECK((J_ref.transpose()*(J_ref*data.ddq+rhs_ref)).isZero(1e-10));
  std::cout << "PV L2 residual in tests = " << (J_ref.transpose()*(J_ref*data.ddq+rhs_ref)).template lpNorm<2>() << "\n";

  prox_settings.max_iter = 10;
  prox_settings.mu = 1e4;
  data.ddq.setZero();
  proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  // std::cout << "Error for proxLTLs = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  
  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv)*0;
  tau = VectorXd::Random(model.nv);
  
  prox_settings.mu = 1e-4;
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  initPvSolver(model,data,contact_models);
  pv_settings.use_early = true;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout << "data.ddq = " << data.ddq.transpose() << "\ndata_ref.ddq = " << data_ref.ddq.transpose() << "\n";
  std::cout<< "Talos cABA err = " << std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq)) << std::endl;

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
 
}

BOOST_AUTO_TEST_CASE(test_FD_humanoid_redundant_baumgarte)
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
  const Model::JointIndex RF_id = model.getJointId(RF);
    
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_3D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kd.setIdentity();
  ci_RF.corrector.Kp.setIdentity();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_RF2(CONTACT_6D,model,model.getJointId("rleg5_joint"),LOCAL);
  ci_RF2.joint1_placement.setRandom();
  ci_RF2.corrector.Kd.setIdentity();
  ci_RF2.corrector.Kp.setZero();
  contact_models.push_back(ci_RF2);
  contact_datas.push_back(RigidConstraintData(ci_RF2));

  std::cout << "Contact data c1Mc2 = " << contact_datas[1].c1Mc2 << "\n";
    
  const double mu0 = 1e-4;
  
  ProximalSettings prox_settings(1e-14,mu0,10);
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);

  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();   
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  // std::cout << "data.ddq = " << data.ddq.transpose() << "\ndata_ref.ddq = " << data_ref.ddq.transpose() << "\n";
  // std::cout << "PV Delassus matrix = " << data.LA[0] << "\n";
  std::cout << "Error for pv now = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  
  // prox_settings.max_iter = 10;
  // prox_settings.mu = 1e4;
  // data.ddq.setZero();
  // proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  // BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  // std::cout << "Error for proxLTLs = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  
  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  
  prox_settings.mu = 1e-4;
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  initPvSolver(model,data,contact_models);
  pv_settings.use_early = true;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout << "data.ddq = " << data.ddq.transpose() << "\ndata_ref.ddq = " << data_ref.ddq.transpose() << "\n";
  std::cout<< "Talos cABA err = " << std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq)) << std::endl;

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
 
}

BOOST_AUTO_TEST_CASE(test_CD_3D_LOCAL_Quadruped)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  // pinocchio::urdf::buildModel("/home/ajay/mlibs/pinocchio/iiwa2.urdf", model);
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/solo_description/robots/solo12.urdf");
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(), model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  // q = randomConfiguration(model);BOOST_AUTO_TEST_CASE(test_forward_dynamics_in_contact_6D_LOCAL_humanoid)
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
   const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
   const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D,model, LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  
  // Eigen::DenseIndex constraint_dim = 0;
  // for(size_t k = 0; k < contact_models.size(); ++k)
  //   constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.0;
  
  // computeAllTerms(model,data_ref,q,v);
  // data_ref.M.triangularView<Eigen::StrictlyLower>() =
  // data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  // Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  // J_ref.setZero();
  // Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6,model.nv);
  
  // getJointJacobian(model,data_ref,
  //                  ci_RF.joint1_id,ci_RF.reference_frame,
  //                  Jtmp);
  // J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  // Jtmp.setZero();
  // getJointJacobian(model,data_ref,
  //                  ci_LF.joint1_id,ci_LF.reference_frame,
  //                  Jtmp);
  // J_ref.middleRows<6>(6) = ci_LF.joint1_placement.inverse().toActionMatrix() * Jtmp;
  
  // Eigen::VectorXd rhs_ref(constraint_dim);
  // rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type,ci_RF.joint1_placement).toVector();
  // rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type,ci_LF.joint1_placement).toVector();
  
  // Eigen::MatrixXd KKT_matrix_ref
  // = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  // KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  // KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  // KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  // forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  // forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
  // BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());

  ProximalSettings prox_settings(1e-12,mu0,1);
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  
  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
   pv_settings.max_iter = 1;
  pv_settings.mu = 0.0;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  std::cout << "Error for pv = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  // pv_settings.use_early = true;
  //  pv_settings.max_iter = 1;
  // pv_settings.mu = 1e-3;
  // pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  // std::cout << "Error for pv = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  // auto con_Residual = J_ref*data.ddq+rhs_ref;
  // BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());

  prox_settings.max_iter = 10;
  prox_settings.mu = 1e4;
  data.ddq.setZero();
  proxLTLs(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout << "Error for proxLTLs = " <<  std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq))  << "\n";
  prox_settings.mu = 0.0;
  
  // Check that the decomposition is correct
  // const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  // Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
  // BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
  // BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
  // Check solutions
  // auto ddq_err = (data.ddq - data_ref.ddq).eval();
  

  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  pv_settings.mu = 0.0;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  initPvSolver(model,data,contact_models);
  pv_settings.use_early = true;
  pv_settings.max_iter = 10;
  pv_settings.mu = 1e-4;
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  std::cout<< "Quadruped cABA err = " << std::sqrt((data.ddq - data_ref.ddq).dot(data.ddq - data_ref.ddq)) << std::endl;

  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  // Send non-zero mu and max 1 iteration and the solution should not match
  // prox_settings.mu = 1e-6;
  // pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  // BOOST_CHECK(!data.ddq.isApprox(data_ref.ddq, 1e-10));

  // Change max iter to 10 and now should work
  // prox_settings.max_iter = 20;
  

}
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "FR_KFE";
  const JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "FL_KFE";
  const JointIndex LF_id = model.getJointId(LF);
  const std::string LR = "HL_KFE";
  const JointIndex LR_id = model.getJointId(LR);
  const std::string RR = "HR_KFE";
  const JointIndex RR_id = model.getJointId(RR);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_3D,model, RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kp.setZero();
  ci_RF.corrector.Kd.setZero();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_3D,model,LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.corrector.Kp.setZero();
  ci_LF.corrector.Kd.setZero();
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  RigidConstraintModel ci_LR(CONTACT_3D,model, LR_id,LOCAL);
  ci_LR.joint1_placement.setRandom();
  ci_LR.corrector.Kp.setZero();
  ci_LR.corrector.Kd.setZero();
  contact_models.push_back(ci_LR);
  contact_datas.push_back(RigidConstraintData(ci_LR));
  RigidConstraintModel ci_RR(CONTACT_3D,model, RR_id,LOCAL);
  ci_RR.joint1_placement.setRandom();
  ci_RR.corrector.Kp.setZero();
  ci_RR.corrector.Kd.setZero();
  contact_models.push_back(ci_RR);
  contact_datas.push_back(RigidConstraintData(ci_RR));

  
  
  
  
  const double mu0 = 0.0; //TODO: set to 0


  ProximalSettings prox_settings(1e-12,mu0,1); // TODO: set max_iter to 1
  // prox_settings.accuracy = 1e-10;
  initConstraintDynamics(model,data_ref,contact_models);
  constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);
  
  Data::PvSettings pv_settings;
  pv_settings.use_early = false;
  pv_settings.mu = 0.0;
  pv_settings.absolute_accuracy = 1e-14;
  pv_settings.max_iter = 10;
  initPvSolver(model,data,contact_models);
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  
  // Check solutions
  auto ddq_err = data_ref.ddq - data.ddq;
  std::cout << "Error for solo = " << std::sqrt(ddq_err.dot(ddq_err)) << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));
  

  // initConstraintDynamics(model,data_ref,contact_models);
  // constraintDynamics(model,data_ref,q,v,tau,contact_models,contact_datas,prox_settings);

  initPvSolver(model,data,contact_models);
  pv(model,data,q,v,tau,contact_models,contact_datas,prox_settings, pv_settings);
  std::cout << "Front foot velocity = " << data.a[4] - data.a_gf[4] << "\n";
  std::cout << "Constraint velocity = " << ci_LF.joint1_placement.actInv(data.a[4] - data.a_gf[4]) << "\n";
  // Check solutions
  std::cout << "Error for solo = " << std::sqrt(ddq_err.dot(ddq_err)) << "\n";
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-10));

  // Eigen::DenseIndex constraint_id = 0;
  // for(size_t k = 0; k < contact_models.size(); ++k)
  // {
  //   const RigidConstraintModel & cmodel = contact_models[k];
  //   const RigidConstraintData & cdata = contact_datas[k];
    
  //   switch(cmodel.type)
  //   {
  //     case pinocchio::CONTACT_3D:
  //     {
  //       BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
  //       break;
  //     }
        
  //     case pinocchio::CONTACT_6D:
  //     {
  //       ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
  //       BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
  //       break;
  //     }
        
  //     default:
  //       break;
  //   }
    
  //   constraint_id += cmodel.size();
  // }
}

// BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D_3D)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
//   pinocchio::buildModels::humanoidRandom(model,true);
//   pinocchio::Data data(model), data_ref(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const std::string RF = "rleg6_joint";
//   const std::string LF = "lleg6_joint";
//   const std::string RA = "rarm6_joint";
  
//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
//   RigidConstraintModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL);
//   contact_models.push_back(ci_RF);
//   contact_datas.push_back(RigidConstraintData(ci_RF));
//   RigidConstraintModel ci_LF(CONTACT_3D,model.getJointId(LF),LOCAL_WORLD_ALIGNED);
//   contact_models.push_back(ci_LF);
//   contact_datas.push_back(RigidConstraintData(ci_LF));
//   RigidConstraintModel ci_RA(CONTACT_3D,model.getJointId(RA),LOCAL);
//   contact_models.push_back(ci_RA);
//   contact_datas.push_back(RigidConstraintData(ci_RA));
  
//   Eigen::DenseIndex constraint_dim = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//     constraint_dim += contact_models[k].size();
  
//   const double mu0 = 0.;
  
//   Eigen::MatrixXd J_ref(constraint_dim,model.nv);
//   J_ref.setZero();
  
//   computeAllTerms(model,data_ref,q,v);
//   data_ref.M.triangularView<Eigen::StrictlyLower>() =
//   data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
//   getJointJacobian(model,data_ref,model.getJointId(RF),LOCAL,J_ref.middleRows<6>(0));
//   Data::Matrix6x J_LF(6,model.nv); J_LF.setZero();
//   getJointJacobian(model,data_ref,model.getJointId(LF),LOCAL_WORLD_ALIGNED,J_LF);
//   J_ref.middleRows<3>(6) = J_LF.middleRows<3>(Motion::LINEAR);
//   Data::Matrix6x J_RA(6,model.nv); J_RA.setZero();
//   getJointJacobian(model,data_ref,model.getJointId(RA),LOCAL,J_RA);
//   J_ref.middleRows<3>(9) = J_RA.middleRows<3>(Motion::LINEAR);
  
//   Eigen::VectorXd rhs_ref(constraint_dim);
  
//   rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,model.getJointId(RF),ci_RF.reference_frame,ci_RF.type).toVector();
//   rhs_ref.segment<3>(6) = computeAcceleration(model,data_ref,model.getJointId(LF),ci_LF.reference_frame,ci_LF.type).linear();
//   rhs_ref.segment<3>(9) = computeAcceleration(model,data_ref,model.getJointId(RA),ci_RA.reference_frame,ci_RA.type).linear();
  
//   Eigen::MatrixXd KKT_matrix_ref
//   = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
//   KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
//   KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
//   KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
//   forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
//   forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
//   ProximalSettings prox_settings(1e-12,mu0,1);
//   initConstraintDynamics(model,data,contact_models);
//   constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
//   // Check that the decomposition is correct
//   const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
//   Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
//   BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
//   BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
//   // Check solutions
//   BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
//   Eigen::DenseIndex constraint_id = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//   {
//     const RigidConstraintModel & cmodel = contact_models[k];
//     const RigidConstraintData & cdata = contact_datas[k];
    
//     switch(cmodel.type)
//     {
//       case pinocchio::CONTACT_3D:
//       {
//         BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
//         break;
//       }
        
//       case pinocchio::CONTACT_6D:
//       {
//         ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
//         BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
//         break;
//       }
        
//       default:
//         break;
//     }
    
//     constraint_id += cmodel.size();
//   }
// }

// BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_6D_LOCAL_WORLD_ALIGNED)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
//   pinocchio::buildModels::humanoidRandom(model,true);
//   pinocchio::Data data(model), data_ref(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const std::string RF = "rleg6_joint";
//   //  const Model::JointIndex RF_id = model.getJointId(RF);
//   const std::string LF = "lleg6_joint";
//   //  const Model::JointIndex LF_id = model.getJointId(LF);
  
//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
//   RigidConstraintModel ci_RF(CONTACT_6D,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
//   contact_models.push_back(ci_RF);
//   contact_datas.push_back(RigidConstraintData(ci_RF));
//   RigidConstraintModel ci_LF(CONTACT_6D,model.getJointId(LF),LOCAL);
//   contact_models.push_back(ci_LF);
//   contact_datas.push_back(RigidConstraintData(ci_LF));
  
//   Eigen::DenseIndex constraint_dim = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//     constraint_dim += contact_models[k].size();
  
//   const double mu0 = 0.;
  
//   Eigen::MatrixXd J_ref(constraint_dim,model.nv);
//   J_ref.setZero();
  
//   computeAllTerms(model,data_ref,q,v);
//   data_ref.M.triangularView<Eigen::StrictlyLower>() =
//   data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
//   updateFramePlacements(model,data_ref);
//   getJointJacobian(model,data_ref,
//                    ci_RF.joint1_id,ci_RF.reference_frame,
//                    J_ref.middleRows<6>(0));
//   getJointJacobian(model,data_ref,
//                    ci_LF.joint1_id,ci_LF.reference_frame,
//                    J_ref.middleRows<6>(6));
  
//   Eigen::VectorXd rhs_ref(constraint_dim);
  
//   rhs_ref.segment<6>(0) = computeAcceleration(model,data_ref,ci_RF.joint1_id,ci_RF.reference_frame,ci_RF.type).toVector();
//   rhs_ref.segment<6>(6) = computeAcceleration(model,data_ref,ci_LF.joint1_id,ci_LF.reference_frame,ci_LF.type).toVector();
  
//   Eigen::MatrixXd KKT_matrix_ref
//   = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
//   KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
//   KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
//   KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
//   forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
//   forwardKinematics(model,data_ref,q,v,data_ref.ddq);
  
//   ProximalSettings prox_settings(1e-12,mu0,1);
//   initConstraintDynamics(model,data,contact_models);
//   constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
//   // Check that the decomposition is correct
//   const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
//   Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
//   BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
//   BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
//   // Check solutions
//   BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
//   BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
//   Eigen::DenseIndex constraint_id = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//   {
//     const RigidConstraintModel & cmodel = contact_models[k];
//     const RigidConstraintData & cdata = contact_datas[k];
    
//     switch(cmodel.type)
//     {
//       case pinocchio::CONTACT_3D:
//       {
//         BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
//         break;
//       }
        
//       case pinocchio::CONTACT_6D:
//       {
//         ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
//         BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
//         break;
//       }
        
//       default:
//         break;
//     }
    
//     constraint_id += cmodel.size();
//   }
// }

// BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_specifying_joint2id)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
//   pinocchio::buildModels::humanoidRandom(model,true);
//   pinocchio::Data data(model), data_ref(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const std::string RF = "rleg6_joint";
//   const std::string LF = "lleg6_joint";
//   const std::string RA = "rarm6_joint";

//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  
//   RigidConstraintModel ci_RF(CONTACT_6D,0,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
//   RigidConstraintModel ci_RF_bis(CONTACT_6D,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
//   ci_RF.joint1_placement.setRandom();
//   ci_RF.joint2_placement.setRandom();
//   ci_RF_bis.joint1_placement = ci_RF.joint2_placement;
//   ci_RF_bis.joint2_placement = ci_RF.joint1_placement;
//   contact_models.push_back(ci_RF);
//   contact_datas.push_back(RigidConstraintData(ci_RF));
  
//   RigidConstraintModel ci_LF(CONTACT_6D,0,model.getJointId(LF),LOCAL);
//   RigidConstraintModel ci_LF_bis(CONTACT_6D,model.getJointId(LF),LOCAL);
//   ci_LF.joint1_placement.setRandom();
//   ci_LF.joint2_placement.setRandom();
//   ci_LF_bis.joint1_placement = ci_LF.joint2_placement;
//   ci_LF_bis.joint2_placement = ci_LF.joint1_placement;
//   contact_models.push_back(ci_LF);
//   contact_datas.push_back(RigidConstraintData(ci_LF));
  
//   RigidConstraintModel ci_RA(CONTACT_6D,0,model.getJointId(RA),LOCAL);
//   RigidConstraintModel ci_RA_bis(CONTACT_6D,model.getJointId(RA),LOCAL);
//   ci_RA.joint1_placement.setRandom();
//   ci_RA.joint2_placement.setRandom();
//   ci_RA_bis.joint1_placement = ci_RA.joint2_placement;
//   ci_RA_bis.joint2_placement = ci_RA.joint1_placement;
//   contact_models.push_back(ci_RA);
//   contact_datas.push_back(RigidConstraintData(ci_RA));
  
//   Eigen::DenseIndex constraint_dim = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//     constraint_dim += contact_models[k].size();
  
//   const double mu0 = 0.;
  
//   Eigen::MatrixXd J_ref(constraint_dim,model.nv);
//   J_ref.setZero();
  
//   computeAllTerms(model,data_ref,q,v);
//   data_ref.M.triangularView<Eigen::StrictlyLower>() =
//   data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
//   updateFramePlacements(model,data_ref);
//   Data::Matrix6x J_RF(6,model.nv), J_LF(6,model.nv), J_RA(6,model.nv);
//   J_RF.setZero(); J_LF.setZero(); J_RA.setZero();
//   Data::Matrix6x J_RF_local(6,model.nv), J_LF_local(6,model.nv), J_RA_local(6,model.nv);
//   J_RF_local.setZero(); J_LF_local.setZero(); J_RA_local.setZero();
//   getJointJacobian(model,data_ref,
//                    ci_RF.joint2_id,WORLD,
//                    J_RF);
//   getJointJacobian(model,data_ref,
//                    ci_RF.joint2_id,LOCAL,
//                    J_RF_local);
//   getJointJacobian(model,data_ref,
//                    ci_LF.joint2_id,WORLD,
//                    J_LF);
//   getJointJacobian(model,data_ref,
//                    ci_LF.joint2_id,LOCAL,
//                    J_LF_local);
//   getJointJacobian(model,data_ref,
//                    ci_RA.joint2_id,WORLD,
//                    J_RA);
//   getJointJacobian(model,data_ref,
//                    ci_RA.joint2_id,LOCAL,
//                    J_RA_local);
  
//   {
//     const SE3 oMc(SE3::Matrix3::Identity(),(data_ref.oMi[ci_RF.joint1_id]*ci_RF.joint1_placement).translation());
//     J_ref.middleRows<6>(0) = -oMc.toActionMatrixInverse() * J_RF;
//   }
  
//   {
//     J_ref.middleRows<6>(6) = -(data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement).toActionMatrixInverse() * J_LF;
//   }
  
//   {
//     J_ref.middleRows<6>(12) = -(data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).toActionMatrixInverse() * J_RA;
//   }
  
//   Eigen::VectorXd rhs_ref(constraint_dim);
  
//   forwardKinematics(model, data_ref, q, v, 0*v);
//   const SE3 c1Mc2_1 = (data.oMi[ci_RF.joint1_id]*ci_RF.joint1_placement).actInv(data_ref.oMi[ci_RF.joint2_id]*ci_RF.joint2_placement);
//   SE3 c1Mc2_1_W((data_ref.oMi[ci_RF.joint2_id]).rotation(),
//                 -(data_ref.oMi[ci_RF.joint1_id] * ci_RF.joint1_placement).translation() + data_ref.oMi[ci_RF.joint2_id].translation());
//   Motion acc_1 = c1Mc2_1_W.act(data_ref.a[ci_RF.joint2_id]);
  
//   const SE3 c1Mc2_2 = (data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement).actInv(data_ref.oMi[ci_LF.joint2_id]*ci_LF.joint2_placement);
//   Motion acc_2 = c1Mc2_2.act(ci_LF.joint2_placement.actInv(data_ref.a[ci_LF.joint2_id]));
  
//   const SE3 c1Mc2_3 = (data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).actInv(data_ref.oMi[ci_RA.joint2_id]*ci_RA.joint2_placement);
//   Motion acc_3 = c1Mc2_3.act(ci_RA.joint2_placement.actInv(data_ref.a[ci_RA.joint2_id]));
  
//   rhs_ref.segment<6>(0) = -acc_1.toVector();
//   rhs_ref.segment<6>(6) = -acc_2.toVector();
//   rhs_ref.segment<6>(12) = -acc_3.toVector();
  
//   Eigen::MatrixXd KKT_matrix_ref
//   = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
//   KKT_matrix_ref.topLeftCorner(constraint_dim,constraint_dim).diagonal().fill(-mu0);
//   KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
//   KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
//   KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
//   forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
//   forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);
  
//   ProximalSettings prox_settings(1e-12,mu0,1);
//   initConstraintDynamics(model,data,contact_models);
//   constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
//   std::cout << "acc_1 ref:\n" << acc_1 << std::endl;
//   std::cout << "acc_1:\n" << contact_datas[0].contact2_acceleration_drift << std::endl;
//   BOOST_CHECK(acc_1.isApprox(contact_datas[0].contact2_acceleration_drift));

//   std::cout << "acc_2 ref:\n" << acc_2 << std::endl;
//   std::cout << "acc_2:\n" << contact_datas[1].contact2_acceleration_drift << std::endl;
//   BOOST_CHECK(acc_2.isApprox(contact_datas[1].contact2_acceleration_drift));

//   std::cout << "acc_3 ref:\n" << acc_3 << std::endl;
//   std::cout << "acc_3:\n" << contact_datas[2].contact2_acceleration_drift << std::endl;
//   BOOST_CHECK(acc_3.isApprox(contact_datas[2].contact2_acceleration_drift));

//   BOOST_CHECK(contact_datas[0].c1Mc2.isApprox(c1Mc2_1));
 
//   const SE3 c1Mc2_1_LWA(contact_datas[0].oMc2.rotation(),
//                         contact_datas[0].oMc1.rotation()*c1Mc2_1.translation());
//   BOOST_CHECK((c1Mc2_1_LWA.toActionMatrix()*(ci_RF.joint2_placement.toActionMatrixInverse()*J_RF_local)).isApprox(-J_ref.middleRows<6>(0)));
//   BOOST_CHECK(contact_datas[0].oMc1.isApprox(ci_RF.joint1_placement));
  
//   BOOST_CHECK(contact_datas[1].c1Mc2.isApprox(c1Mc2_2));
//   BOOST_CHECK((contact_datas[1].oMc1.toActionMatrixInverse()*J_LF).isApprox(-J_ref.middleRows<6>(6)));
//   BOOST_CHECK((data_ref.oMi[ci_LF.joint2_id].toActionMatrix()*J_LF_local).isApprox(J_LF));
//   BOOST_CHECK(contact_datas[1].oMc1.isApprox(ci_LF.joint1_placement));
//   BOOST_CHECK(data.oa[ci_LF.joint2_id].isApprox(data_ref.oMi[ci_LF.joint2_id].act(data_ref.a[ci_LF.joint2_id])));
  
//   BOOST_CHECK(contact_datas[2].c1Mc2.isApprox(c1Mc2_3));
//   BOOST_CHECK((c1Mc2_3.toActionMatrix()*(ci_RA.joint2_placement.toActionMatrixInverse()*J_RA_local)).isApprox(-J_ref.middleRows<6>(12)));
//   BOOST_CHECK(contact_datas[2].oMc1.isApprox(ci_RA.joint1_placement));
//   BOOST_CHECK(data.oa[ci_RA.joint2_id].isApprox(data_ref.oMi[ci_RA.joint2_id].act(data_ref.a[ci_RA.joint2_id])));
  
//   // Check that the decomposition is correct
  
//   forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);
//   const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
//   Eigen::MatrixXd KKT_matrix = contact_chol.matrix();
  
//   BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
//   BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
//   // Check solutions
//   forwardKinematics(model,data,q,v,data.ddq);
//   BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
//   BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
//   Motion acc_1_final = c1Mc2_1_W.act(data.a[ci_RF.joint2_id]);
//   BOOST_CHECK(acc_1_final.isZero());
                      
//   std::cout << "acc_1_final:\n" << acc_1_final << std::endl;
  
//   Motion acc_2_final = c1Mc2_2.act(data.a[ci_LF.joint2_id]);
//   BOOST_CHECK(acc_2_final.isZero());
  
//   std::cout << "acc_2_final:\n" << acc_2_final << std::endl;
  
//   Motion acc_3_final = c1Mc2_3.act(data.a[ci_RA.joint2_id]);
//   BOOST_CHECK(acc_3_final.isZero());
  
//   std::cout << "acc_3_final:\n" << acc_3_final << std::endl;
  
//   Eigen::DenseIndex constraint_id = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//   {
//     const RigidConstraintModel & cmodel = contact_models[k];
//     const RigidConstraintData & cdata = contact_datas[k];
    
//     switch(cmodel.type)
//     {
//       case pinocchio::CONTACT_3D:
//       {
//         BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
//         break;
//       }
        
//       case pinocchio::CONTACT_6D:
//       {
//         ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
//         BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
//         break;
//       }
        
//       default:
//         break;
//     }
    
//     constraint_id += cmodel.size();
//   }
  
//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_bis;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_bis;
  
//   contact_models_bis.push_back(ci_RF_bis);
//   contact_models_bis.push_back(ci_LF_bis);
//   contact_models_bis.push_back(ci_RA_bis);
  
//   for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)::const_iterator it = contact_models_bis.begin();
//       it != contact_models_bis.end(); ++it)
//     contact_datas_bis.push_back(RigidConstraintData(*it));
  
//   Data data_bis(model);
//   initConstraintDynamics(model,data_bis,contact_models_bis);
//   constraintDynamics(model,data_bis,q,v,tau,contact_models_bis,contact_datas_bis,prox_settings);
  
//   BOOST_CHECK(data_bis.ddq.isApprox(data.ddq));
//   std::cout << "ddq: " << data_bis.ddq.transpose() << std::endl;
//   std::cout << "ddq: " << data.ddq.transpose() << std::endl;
  
// //  Eigen::DenseIndex constraint_id = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//   {
//     const RigidConstraintModel & cmodel = contact_models[k];
//     const RigidConstraintData & cdata = contact_datas[k];
//     const RigidConstraintModel & cmodel_bis = contact_models_bis[k];
//     const RigidConstraintData & cdata_bis = contact_datas_bis[k];
    
//     BOOST_CHECK(cmodel_bis.reference_frame == cmodel.reference_frame);
//     BOOST_CHECK(cmodel_bis.joint1_id == cmodel.joint2_id);
//     BOOST_CHECK(cmodel_bis.joint2_id == cmodel.joint1_id);
//     BOOST_CHECK(cdata.oMc1.isApprox(cdata_bis.oMc2));
//     BOOST_CHECK(cdata.oMc2.isApprox(cdata_bis.oMc1));
//     BOOST_CHECK(cdata.c1Mc2.isApprox(cdata_bis.c1Mc2.inverse()));
    
//     std::cout << "cdata.c1Mc2:\n" << cdata.c1Mc2 << std::endl;
//     Force contact_force, contact_force_bis;
//     switch(cmodel.reference_frame)
//     {
//       case LOCAL_WORLD_ALIGNED:
//       {
//         SE3 c1Mc2_LWA(SE3::Matrix3::Identity(),
//                       cdata.oMc1.rotation()*cdata.c1Mc2.translation());
//         contact_force_bis = cdata_bis.contact_force;
//         BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(c1Mc2_LWA.actInv(cdata.contact2_acceleration_drift)));

//         contact_force = c1Mc2_LWA.actInv(cdata.contact_force);
//         BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
//         break;
//       }
//       case LOCAL:
//       {
//         contact_force_bis = cdata_bis.contact_force;
//         BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(cdata.c1Mc2.actInv(cdata.contact2_acceleration_drift)));

//         contact_force = cdata.c1Mc2.actInv(cdata.contact_force);
//         BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
//         break;
//       }
//       case WORLD:
//         BOOST_CHECK(false);
//         break;
//     }
    
//     std::cout << "contact_force: " << contact_force.toVector().transpose() << std::endl;
//     std::cout << "contact_force_bis: " << contact_force_bis.toVector().transpose() << std::endl;
//   }
// }

// PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData)
// createData(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models)
// {
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_datas;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//     contact_datas.push_back(pinocchio::RigidConstraintData(contact_models[k]));
  
//   return contact_datas;
// }

// BOOST_AUTO_TEST_CASE(test_correction_CONTACT_6D)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
// //  pinocchio::buildModels::humanoidRandom(model,true);
//   const JointIndex joint_id = model.addJoint(0,JointModelFreeFlyer(),SE3::Identity(),"root");
//   const Inertia box_inertia = Inertia::FromBox(100.,1.,1.,1.);
//   model.appendBodyToJoint(joint_id,box_inertia);
//   pinocchio::Data data(model), data_ref(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const std::string RF = "root";
//   const JointIndex RF_id = model.getJointId(RF);

//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  
//   RigidConstraintModel ci_RF(CONTACT_6D,RF_id,LOCAL);
//   ci_RF.joint1_placement.setIdentity();
//   ci_RF.joint2_placement.setIdentity();
//   ci_RF.corrector.Kp = 10.;
//   ci_RF.corrector.Kd = 2. * sqrt(ci_RF.corrector.Kp);
//   contact_models.push_back(ci_RF);
  
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas = createData(contact_models);
//   initConstraintDynamics(model,data,contact_models);
//   constraintDynamics(model,data,q,v,tau,contact_models,contact_datas);
  
//   BOOST_CHECK(contact_datas[0].oMc1.isApprox(data.oMi[ci_RF.joint1_id] * ci_RF.joint1_placement));
//   BOOST_CHECK(contact_datas[0].oMc2.isApprox(data.oMi[ci_RF.joint2_id] * ci_RF.joint2_placement));
//   BOOST_CHECK(contact_datas[0].contact1_velocity.isApprox(contact_datas[0].oMc1.actInv(data.ov[ci_RF.joint1_id])));
//   BOOST_CHECK(contact_datas[0].contact2_velocity.isZero());
  
//   const double dt = 1e-8;
//   const VectorXd q_plus = integrate(model,q,v*dt);

//   Data data_plus(model);
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_plus = createData(contact_models);
//   initConstraintDynamics(model,data_plus,contact_models);
//   constraintDynamics(model,data_plus,q_plus,v,tau,contact_models,contact_datas_plus);
  
//   const Motion contact_RF_velocity_error_fd = log6(contact_datas[0].c1Mc2.act(contact_datas_plus[0].c1Mc2.inverse()))/dt;
//   BOOST_CHECK(contact_RF_velocity_error_fd.isApprox(contact_datas[0].contact_velocity_error,sqrt(dt)));
//   std::cout << "contact_RF_velocity_error_fd:\n" << contact_RF_velocity_error_fd << std::endl;
//   std::cout << "contact_velocity_error:\n" << contact_datas[0].contact_velocity_error << std::endl;
  
//   // Simulation loop
//   {
//     const int N = 200;
//     const double dt = 1e-3;
//     const double mu = 1e-12;
    
// //    model.gravity.setZero();
//     Data data_sim(model);
//     PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_sim = createData(contact_models);
//     initConstraintDynamics(model,data_sim,contact_models);
    
//     Eigen::VectorXd q0 (model.nq);
//     const SE3 M0 = SE3::Random();
//     q0 << M0.translation(), SE3::Quaternion(M0.rotation()).coeffs();
//     const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
//     Eigen::VectorXd a = Eigen::VectorXd(model.nv);
//     Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    
//     Eigen::VectorXd q(q0), v(v0);
    
//     tau = rnea(model,data_sim,q,v,0*a);
//     ProximalSettings prox_settings(1e-12,mu,1);
//     constraintDynamics(model,data_sim,q0,v0,tau,contact_models,contact_data_sim,prox_settings);
//     PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_sim_prev(contact_data_sim);

//     for(int it = 0; it <= N; it++)
//     {
//       a = constraintDynamics(model,data_sim,q,v,tau,contact_models,contact_data_sim,prox_settings);
//       v += a*dt;
//       q = integrate(model,q,v*dt);
      
//       if(it > 1)
//       {
//         for(size_t k = 0; k < contact_models.size(); ++k)
//         {
//           const RigidConstraintData & cdata = contact_data_sim[k];
//           const RigidConstraintData & cdata_prev = contact_data_sim_prev[k];
          
//           BOOST_CHECK(cdata.contact_placement_error.toVector().norm() <= cdata_prev.contact_placement_error.toVector().norm());
//         }
//       }
      
//       contact_data_sim_prev = contact_data_sim;
//     }
//   }
// }

// BOOST_AUTO_TEST_CASE(test_correction_CONTACT_3D)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
// //  pinocchio::buildModels::humanoidRandom(model,true);
//   const JointIndex joint_id = model.addJoint(0,JointModelFreeFlyer(),SE3::Identity(),"root");
//   const Inertia box_inertia = Inertia::FromBox(100.,1.,1.,1.);
//   model.appendBodyToJoint(joint_id,box_inertia);
//   pinocchio::Data data(model), data_ref(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const double mu = 1e-8;
//   const std::string RF = "root";
//   const JointIndex RF_id = model.getJointId(RF);

//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  
//   RigidConstraintModel ci_RF1(CONTACT_3D,RF_id,LOCAL);
//   ci_RF1.joint1_placement.translation() = SE3::Vector3(0.5,0.5,-0.5);
//   ci_RF1.joint2_placement.setRandom();
//   ci_RF1.corrector.Kp = 10.;
//   ci_RF1.corrector.Kd = 2. * sqrt(ci_RF1.corrector.Kp);
//   contact_models.push_back(ci_RF1);
  
//   RigidConstraintModel ci_RF2(CONTACT_3D,RF_id,LOCAL);
//   ci_RF2.joint1_placement.translation() = SE3::Vector3(-0.5,0.5,-0.5);
//   ci_RF2.joint2_placement.setRandom();
//   ci_RF2.corrector.Kp = 10.;
//   ci_RF2.corrector.Kd = 2. * sqrt(ci_RF2.corrector.Kp);
//   contact_models.push_back(ci_RF2);
  
//   RigidConstraintModel ci_RF3(CONTACT_3D,RF_id,LOCAL);
//   ci_RF3.joint1_placement.translation() = SE3::Vector3(-0.5,-0.5,-0.5);
//   ci_RF3.joint2_placement.setRandom();
//   ci_RF3.corrector.Kp = 10.;
//   ci_RF3.corrector.Kd = 2. * sqrt(ci_RF3.corrector.Kp);
//   contact_models.push_back(ci_RF3);
  
//   RigidConstraintModel ci_RF4(CONTACT_3D,RF_id,LOCAL);
//   ci_RF4.joint1_placement.translation() = SE3::Vector3(0.5,-0.5,-0.5);
//   ci_RF4.joint2_placement.setRandom();
//   ci_RF4.corrector.Kp = 10.;
//   ci_RF4.corrector.Kd = 2. * sqrt(ci_RF4.corrector.Kp);
//   contact_models.push_back(ci_RF4);
  
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas = createData(contact_models);
//   ProximalSettings prox_settings(1e-12,mu,1);
//   initConstraintDynamics(model,data,contact_models);
//   constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);


//   Eigen::VectorXd contact_placement_error_prev(contact_models.size() * 6);
//   Eigen::VectorXd contact_placement_error(contact_models.size() * 6);
//   contact_placement_error_prev.setZero();
//   contact_placement_error.setZero();
  
//   // Simulation loop
//   {
//     const int N = 200;
//     const double dt = 1e-3;
    
// //    model.gravity.setZero();
//     Data data_sim(model);
//     PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_sim = createData(contact_models);
//     initConstraintDynamics(model,data_sim,contact_models);
    
//     Eigen::VectorXd q0 (model.nq);
//     const SE3 M0 = SE3::Random();
//     q0 << M0.translation(), SE3::Quaternion(M0.rotation()).coeffs();
//     const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
//     Eigen::VectorXd a = Eigen::VectorXd(model.nv);
//     Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    
//     Eigen::VectorXd q(q0), v(v0);
    
//     constraintDynamics(model,data_sim,q0,v0,tau,contact_models,contact_data_sim,prox_settings);
//     PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_sim_prev(contact_data_sim);

//     for(int it = 0; it <= N; it++)
//     {
//       a = constraintDynamics(model,data_sim,q,v,tau,contact_models,contact_data_sim,prox_settings);
//       v += a*dt;
//       q = integrate(model,q,v*dt);


      
      
//       if(it > 1)
//       {
//         for(size_t k = 0; k < contact_models.size(); ++k)
//         {
// 	  const RigidConstraintData & cdata = contact_data_sim[k];
// 	  const RigidConstraintData & cdata_prev = contact_data_sim_prev[k];
// 	  contact_placement_error.segment<6>(6*k) = cdata.contact_placement_error.toVector();
// 	  contact_placement_error_prev.segment<6>(6*k) =
// 	    cdata_prev.contact_placement_error.toVector();
//         }
// 	BOOST_CHECK(contact_placement_error.norm() <= contact_placement_error_prev.norm());
//       }
//       contact_data_sim_prev = contact_data_sim;
//     }
//   }
// }

// BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_in_contact_specifying_joint2id_case3D)
// {
//   using namespace Eigen;
//   using namespace pinocchio;
  
//   pinocchio::Model model;
//   pinocchio::buildModels::humanoidRandom(model,true);
//   pinocchio::Data data(model), data_ref(model);
  
//   model.lowerPositionLimit.head<3>().fill(-1.);
//   model.upperPositionLimit.head<3>().fill( 1.);
//   VectorXd q = randomConfiguration(model);
  
//   VectorXd v = VectorXd::Random(model.nv);
//   VectorXd tau = VectorXd::Random(model.nv);
  
//   const std::string RF = "rleg6_joint";
//   const std::string LF = "lleg6_joint";
//   const std::string RA = "rarm6_joint";

//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  
//   RigidConstraintModel ci_RF(CONTACT_3D,0,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
//   RigidConstraintModel ci_RF_bis(CONTACT_3D,model.getJointId(RF),LOCAL_WORLD_ALIGNED);
//   ci_RF.joint1_placement.setRandom();
//   ci_RF.joint2_placement.setRandom();
//   ci_RF_bis.joint1_placement = ci_RF.joint2_placement;
//   ci_RF_bis.joint2_placement = ci_RF.joint1_placement;
//   contact_models.push_back(ci_RF);
//   contact_datas.push_back(RigidConstraintData(ci_RF));
  
//   RigidConstraintModel ci_LF(CONTACT_3D,0,model.getJointId(LF),LOCAL);
//   RigidConstraintModel ci_LF_bis(CONTACT_3D,model.getJointId(LF),LOCAL);
//   ci_LF.joint1_placement.setRandom();
//   ci_LF.joint2_placement.setRandom();
//   ci_LF_bis.joint1_placement = ci_LF.joint2_placement;
//   ci_LF_bis.joint2_placement = ci_LF.joint1_placement;
//   contact_models.push_back(ci_LF);
//   contact_datas.push_back(RigidConstraintData(ci_LF));
  
//   RigidConstraintModel ci_RA(CONTACT_6D,0,model.getJointId(RA),LOCAL);
//   RigidConstraintModel ci_RA_bis(CONTACT_6D,model.getJointId(RA),LOCAL);
//   ci_RA.joint1_placement.setRandom();
//   ci_RA.joint2_placement.setRandom();
//   ci_RA_bis.joint1_placement = ci_RA.joint2_placement;
//   ci_RA_bis.joint2_placement = ci_RA.joint1_placement;
//   contact_models.push_back(ci_RA);
//   contact_datas.push_back(RigidConstraintData(ci_RA));
  
//   Eigen::DenseIndex constraint_dim = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//     constraint_dim += contact_models[k].size();
  
//   const double mu0 = 0.;
  
//   Eigen::MatrixXd J_ref(constraint_dim,model.nv);
//   J_ref.setZero();
  
//   computeAllTerms(model,data_ref,q,v);
//   data_ref.M.triangularView<Eigen::StrictlyLower>() =
//   data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
//   Data::Matrix6x J_RF(6,model.nv), J_LF(6,model.nv), J_RA(6,model.nv);
//   J_RF.setZero(); J_LF.setZero(); J_RA.setZero();
//   Data::Matrix6x J_RF_local(6,model.nv), J_LF_local(6,model.nv), J_RA_local(6,model.nv);
//   J_RF_local.setZero(); J_LF_local.setZero(); J_RA_local.setZero();
//   getJointJacobian(model,data_ref,
//                    ci_RF.joint2_id,WORLD,
//                    J_RF);
//   getJointJacobian(model,data_ref,
//                    ci_RF.joint2_id,LOCAL,
//                    J_RF_local);
//   J_RF_local = ci_RF.joint2_placement.toActionMatrixInverse() * J_RF_local;
//   getJointJacobian(model,data_ref,
//                    ci_LF.joint2_id,WORLD,
//                    J_LF);
//   getJointJacobian(model,data_ref,
//                    ci_LF.joint2_id,LOCAL,
//                    J_LF_local);
//   J_LF_local = ci_LF.joint2_placement.toActionMatrixInverse() * J_LF_local;
//   getJointJacobian(model,data_ref,
//                    ci_RA.joint2_id,WORLD,
//                    J_RA);
//   getJointJacobian(model,data_ref,
//                    ci_RA.joint2_id,LOCAL,
//                    J_RA_local);
  
//   {
//     const SE3 oMc2 = data_ref.oMi[ci_RF.joint2_id]*ci_RF.joint2_placement;
//     J_ref.middleRows<3>(0) = -oMc2.rotation() * J_RF_local.middleRows<3>(Motion::LINEAR);
//   }

//   {
//     const SE3 oMc1 = data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement;
//     const SE3 oMc2 = data_ref.oMi[ci_LF.joint2_id]*ci_LF.joint2_placement;
//     const SE3 c1Mc2 = oMc1.actInv(oMc2);
//     J_ref.middleRows<3>(3) = - c1Mc2.rotation() * J_LF_local.middleRows<3>(SE3::LINEAR);
//   }
  
//   {
//     J_ref.middleRows<6>(6) = -(data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).toActionMatrixInverse() * J_RA;
//   }
  
//   Eigen::VectorXd rhs_ref(constraint_dim);

//   forwardKinematics(model, data_ref, q, v, 0*v);
//   Motion::Vector3 acc_1;
//   {
//     const SE3 oMc2 = data_ref.oMi[ci_RF.joint2_id]*ci_RF.joint2_placement;
//     const Motion v2 = ci_RF.joint2_placement.actInv(data_ref.v[ci_RF.joint2_id]);
//     const Motion a2 = ci_RF.joint2_placement.actInv(data_ref.a[ci_RF.joint2_id]);
//     acc_1 = oMc2.rotation()*(a2.linear() + v2.angular().cross(v2.linear()));
//   }
  
//   Motion::Vector3 acc_2;
//   {
//     const SE3 oMc1 = data_ref.oMi[ci_LF.joint1_id]*ci_LF.joint1_placement;
//     const SE3 oMc2 = data_ref.oMi[ci_LF.joint2_id]*ci_LF.joint2_placement;
//     const SE3 c1Mc2 = oMc1.actInv(oMc2);
//     const Motion v2 = ci_LF.joint2_placement.actInv(data_ref.v[ci_LF.joint2_id]);
//     const Motion a2 = ci_LF.joint2_placement.actInv(data_ref.a[ci_LF.joint2_id]);
//     acc_2 = c1Mc2.rotation()*(a2.linear() + v2.angular().cross(v2.linear()));
//   }
  
//   const SE3 c1Mc2_3 = (data_ref.oMi[ci_RA.joint1_id]*ci_RA.joint1_placement).actInv(data_ref.oMi[ci_RA.joint2_id]*ci_RA.joint2_placement);
//   Motion acc_3 = c1Mc2_3.act(ci_RA.joint2_placement.actInv(data_ref.a[ci_RA.joint2_id]));

//   rhs_ref.segment<3>(0) = -acc_1;
//   rhs_ref.segment<3>(3) = -acc_2;
//   rhs_ref.segment<6>(6) = -acc_3.toVector();

//   Eigen::MatrixXd KKT_matrix_ref
//   = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
//   KKT_matrix_ref.topLeftCorner(constraint_dim,constraint_dim).diagonal().fill(-mu0);
//   KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
//   KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
//   KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();

//   forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
//   forwardKinematics(model,data_ref,q,v,0*data_ref.ddq);

//   ProximalSettings prox_settings(1e-12,0,1);
//   initConstraintDynamics(model,data,contact_models);
//   constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);

//   const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
//   Eigen::MatrixXd KKT_matrix = contact_chol.matrix();

//   BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv,model.nv).isApprox(KKT_matrix_ref.bottomRightCorner(model.nv,model.nv)));
//   BOOST_CHECK(KKT_matrix.topRightCorner(constraint_dim,model.nv).isApprox(J_ref));
//   BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));
  
//   std::cout << "KKT_matrix.topRightCorner(constraint_dim,model.nv):\n" << KKT_matrix.topRightCorner(constraint_dim,model.nv) << std::endl;
//   std::cout << "KKT_matrix_ref.topRightCorner(constraint_dim,model.nv):\n" << KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) << std::endl;

//   // Check solutions
//   forwardKinematics(model,data,q,v,data.ddq);
//   BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
//   BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
//   std::cout << "data_ref.ddq: " << data_ref.ddq.transpose() << std::endl;
//   std::cout << "data.ddq: " << data.ddq.transpose() << std::endl;
//   std::cout << "res: " << (J_ref*data.ddq+rhs_ref).transpose() << std::endl;
//   std::cout << "res_ref: " << (J_ref*data_ref.ddq+rhs_ref).transpose() << std::endl;
  
//   const Motion vel_1_final = ci_RF.joint2_placement.actInv(data.v[ci_RF.joint2_id]);
//   const Motion::Vector3 acc_1_final = ci_RF.joint2_placement.actInv(data.a[ci_RF.joint2_id]).linear() + vel_1_final.angular().cross(vel_1_final.linear());
//   BOOST_CHECK(acc_1_final.isZero());

//   std::cout << "acc_1_final:" << acc_1_final.transpose() << std::endl;
  
//   const Motion vel_2_final = ci_LF.joint2_placement.actInv(data.v[ci_LF.joint2_id]);
//   const Motion::Vector3 acc_2_final = ci_LF.joint2_placement.actInv(data.a[ci_LF.joint2_id]).linear() + vel_2_final.angular().cross(vel_2_final.linear());
//   BOOST_CHECK(acc_2_final.isZero());
  
//   std::cout << "acc_2_final:" << acc_2_final.transpose() << std::endl;
  
//   Motion acc_3_final = c1Mc2_3.act(data.a[ci_RA.joint2_id]);
//   BOOST_CHECK(acc_3_final.isZero());

//   std::cout << "acc_3_final:\n" << acc_3_final << std::endl;

//   Eigen::DenseIndex constraint_id = 0;
//   for(size_t k = 0; k < contact_models.size(); ++k)
//   {
//     const RigidConstraintModel & cmodel = contact_models[k];
//     const RigidConstraintData & cdata = contact_datas[k];

//     switch(cmodel.type)
//     {
//       case pinocchio::CONTACT_3D:
//       {
//         BOOST_CHECK(cdata.contact_force.linear().isApprox(data_ref.lambda_c.segment(constraint_id,cmodel.size())));
//         break;
//       }

//       case pinocchio::CONTACT_6D:
//       {
//         ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(data_ref.lambda_c.segment<6>(constraint_id));
//         BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
//         break;
//       }

//       default:
//         break;
//     }

//     constraint_id += cmodel.size();
//   }

//   // Contact models and data
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_bis;
//   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_bis;

//   contact_models_bis.push_back(ci_RF_bis);
//   contact_models_bis.push_back(ci_LF_bis);
//   contact_models_bis.push_back(ci_RA_bis);

//   for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)::const_iterator it = contact_models_bis.begin();
//       it != contact_models_bis.end(); ++it)
//     contact_datas_bis.push_back(RigidConstraintData(*it));

//   Data data_bis(model);
//   initConstraintDynamics(model,data_bis,contact_models_bis);
//   constraintDynamics(model,data_bis,q,v,tau,contact_models_bis,contact_datas_bis,prox_settings);

//   BOOST_CHECK(data_bis.ddq.isApprox(data.ddq));
//   std::cout << "ddq: " << data_bis.ddq.transpose() << std::endl;
//   std::cout << "ddq: " << data.ddq.transpose() << std::endl;

//   for(size_t k = 0; k < contact_models.size(); ++k)
//   {
//     const RigidConstraintModel & cmodel = contact_models[k];
//     const RigidConstraintData & cdata = contact_datas[k];
//     const RigidConstraintModel & cmodel_bis = contact_models_bis[k];
//     const RigidConstraintData & cdata_bis = contact_datas_bis[k];

//     BOOST_CHECK(cmodel_bis.reference_frame == cmodel.reference_frame);
//     BOOST_CHECK(cmodel_bis.joint1_id == cmodel.joint2_id);
//     BOOST_CHECK(cmodel_bis.joint2_id == cmodel.joint1_id);
//     BOOST_CHECK(cdata.oMc1.isApprox(cdata_bis.oMc2));
//     BOOST_CHECK(cdata.oMc2.isApprox(cdata_bis.oMc1));
//     BOOST_CHECK(cdata.c1Mc2.isApprox(cdata_bis.c1Mc2.inverse()));

//     std::cout << "cdata.c1Mc2:\n" << cdata.c1Mc2 << std::endl;
//     Force contact_force, contact_force_bis;
//     switch(cmodel.reference_frame)
//     {
//       case LOCAL_WORLD_ALIGNED:
//       {
//         SE3 c1Mc2_LWA(SE3::Matrix3::Identity(),
//                       cdata.oMc1.rotation()*cdata.c1Mc2.translation());
//         contact_force_bis = cdata_bis.contact_force;

//         if(cmodel.type == CONTACT_3D)
//           contact_force = cdata.contact_force;
//         else
//         {
//           contact_force = c1Mc2_LWA.actInv(cdata.contact_force);
//           BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(c1Mc2_LWA.actInv(cdata.contact2_acceleration_drift)));
//         }
//         BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
//         break;
//       }
//       case LOCAL:
//       {
//         contact_force_bis = cdata_bis.contact_force;

//         if(cmodel.type == CONTACT_3D)
//           contact_force.linear() = cdata.c1Mc2.actInv(cdata.contact_force).linear();
//         else
//         {
//           contact_force = cdata.c1Mc2.actInv(cdata.contact_force);
//           BOOST_CHECK(cdata_bis.contact1_acceleration_drift.isApprox(cdata.c1Mc2.actInv(cdata.contact2_acceleration_drift)));
//         }
//         BOOST_CHECK(contact_force.isApprox(-contact_force_bis));
//         break;
//       }
//       case WORLD:
//         BOOST_CHECK(false);
//         break;
//     }

//     std::cout << "contact_force: " << contact_force.toVector().transpose() << std::endl;
//     std::cout << "contact_force_bis: " << contact_force_bis.toVector().transpose() << std::endl;
//   }
// }

// BOOST_AUTO_TEST_CASE(test_diagonal_inertia)
// {
//   using namespace pinocchio;

//   const double mu = 1e2;
//   const Inertia diagonal6_inertia(mu,Inertia::Vector3::Zero(),
//                                   Symmetric3(mu,0,mu,0,0,mu));
//   const Inertia::Matrix6 diagonal6_inertia_mat = diagonal6_inertia.matrix();
//   BOOST_CHECK(diagonal6_inertia_mat.block(Inertia::LINEAR,Inertia::ANGULAR,3,3).isZero());
//   BOOST_CHECK(diagonal6_inertia_mat.block(Inertia::ANGULAR,Inertia::LINEAR,3,3).isZero());
  
//   const SE3 M = SE3::Random();
// //  const Inertia::Matrix3 RtRmu = mu * M.rotation().transpose()*M.rotation();
//   const Inertia::Matrix3 RtRmu = mu * Inertia::Matrix3::Identity();
//   Inertia I6_translate(mu,M.translation(),Symmetric3(RtRmu));
  
//   const Inertia I6_ref = M.act(diagonal6_inertia);
//   BOOST_CHECK(I6_translate.isApprox(I6_ref));
  
//   const Inertia diagonal3_inertia(mu,Inertia::Vector3::Zero(),
//                                   Symmetric3(0,0,0,0,0,0));
//   const Inertia::Matrix6 diagonal3_inertia_mat = diagonal3_inertia.matrix();
//   BOOST_CHECK(diagonal3_inertia_mat.block(Inertia::LINEAR,Inertia::ANGULAR,3,3).isZero());
//   BOOST_CHECK(diagonal3_inertia_mat.block(Inertia::ANGULAR,Inertia::LINEAR,3,3).isZero());
//   BOOST_CHECK(diagonal3_inertia_mat.block(Inertia::ANGULAR,Inertia::ANGULAR,3,3).isZero());
  
//   Inertia I3_translate(mu,
//                        M.translation(),
//                        Symmetric3(0,0,0,0,0,0));
  
//   const Inertia I3_ref = M.act(diagonal3_inertia);
//   BOOST_CHECK(I3_translate.isApprox(I3_ref));
// }


BOOST_AUTO_TEST_SUITE_END()
