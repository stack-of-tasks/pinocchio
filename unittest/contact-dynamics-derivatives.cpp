//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE ( test_sparse_contact_dynamics_derivatives )
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
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),LOCAL);
  RigidContactModel ci_RF(CONTACT_3D,model.getFrameId(RF),LOCAL);

  contact_models.push_back(ci_LF);
  contact_models.push_back(ci_RF);
  RigidContactData cd_LF(model.nv);

  RigidContactData cd_RF(model.nv);

  contact_datas.push_back(cd_LF);
  contact_datas.push_back(cd_RF);
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,mu0);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, q, v, tau, contact_models, contact_datas, mu0);  
  
  //Data_ref
  crba(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeABADerivatives(model, data_ref, q, v, tau, data.contact_forces);
  
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
  
  getJointAccelerationDerivatives(model, data_ref,
                                  model.getJointId(LF),
                                  LOCAL,
                                  vLF_partial_dq,
                                  aLF_partial_dq,
                                  aLF_partial_dv,
                                  aLF_partial_da);
  getJointAccelerationDerivatives(model, data_ref,
                                  model.getJointId(RF),
                                  LOCAL,
                                  vRF_partial_dq,
                                  aRF_partial_dq,
                                  aRF_partial_dv,
                                  aRF_partial_da);

  MatrixXd Jc(6+3, model.nv);
  Jc << aLF_partial_da,
    aRF_partial_da.topRows<3>();
  
  MatrixXd K(model.nv+6+3, model.nv+6+3);
  K << data_ref.M, Jc.transpose(),
    Jc, MatrixXd::Zero(6+3,6+3);
  MatrixXd Kinv = K.inverse();

  MatrixXd J_LF(6, model.nv), J_RF(6, model.nv);
  J_LF.setZero(); J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_RF);
  MatrixXd osim((Jc * data_ref.M.inverse() * Jc.transpose()).inverse());
  BOOST_CHECK(data.osim.isApprox(osim,1e-8));

  MatrixXd ac_partial_dq(6+3, model.nv);

  aRF_partial_dq.topRows<3>() +=
    cross(data_ref.v[model.getJointId(RF)].angular(),
          vRF_partial_dq.topRows<3>())
    - cross(data_ref.v[model.getJointId(RF)].linear(),
            vRF_partial_dq.bottomRows<3>());

  aLF_partial_dq.topRows<3>() +=
    cross(data_ref.v[model.getJointId(LF)].angular(),
          vLF_partial_dq.topRows<3>())
    - cross(data_ref.v[model.getJointId(LF)].linear(),
            vLF_partial_dq.bottomRows<3>());    
  
  ac_partial_dq << aLF_partial_dq,
    aRF_partial_dq.topRows<3>();

  MatrixXd dac_dq = ac_partial_dq - Jc * data_ref.Minv*data_ref.dtau_dq;

  //std::cerr<<"data.dac_dq"<<std::endl<<data.dac_dq<<std::endl;
  BOOST_CHECK(data.dac_dq.isApprox(dac_dq,1e-8));
  BOOST_CHECK(Kinv.bottomLeftCorner(6+3, model.nv).isApprox(osim*Jc*data_ref.Minv,1e-8));

  MatrixXd df_dq = Kinv.bottomLeftCorner(6+3, model.nv)* data_ref.dtau_dq +
    Kinv.bottomRightCorner(6+3,6+3)*ac_partial_dq;
  
  MatrixXd ddq_dq =  data_ref.M.inverse() * (-data_ref.dtau_dq +  Jc.transpose() * df_dq);
  
  //std::cerr<<"df_dq"<<std::endl<<df_dq<<std::endl;
  //std::cerr<<"dlambda_dq"<<std::endl<<data.dlambda_dq<<std::endl;
  
  BOOST_CHECK(df_dq.isApprox(data.dlambda_dq, 1e-8));
  BOOST_CHECK(ddq_dq.isApprox(data.ddq_dq, 1e-8));
}

BOOST_AUTO_TEST_CASE ( test_contact_dynamics_derivatives_fd )
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
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas;
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),LOCAL);
  RigidContactModel ci_RF(CONTACT_3D,model.getFrameId(RF),LOCAL);

  contact_models.push_back(ci_LF);
  contact_models.push_back(ci_RF);
  RigidContactData cd_LF(model.nv);

  RigidContactData cd_RF(model.nv);

  contact_datas.push_back(cd_LF);
  contact_datas.push_back(cd_RF);
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;


  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,mu0);

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, q, v, tau, contact_models, contact_datas, mu0);  


  //Data_fd
  initContactDynamics(model,data_fd,contact_models);
  
  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  const VectorXd ddq0 = contactDynamics(model,data_fd,q,v,tau,contact_models,mu0);
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);
  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = contactDynamics(model,data_fd,q_plus,v,tau,contact_models,mu0);

    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    v_eps[k] -= alpha;
  }

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = contactDynamics(model,data_fd,q,v_plus,tau,contact_models,mu0);

    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
}


BOOST_AUTO_TEST_SUITE_END ()
