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
  RigidContactModel ci_RF(CONTACT_6D,model.getFrameId(RF),LOCAL);
  //contact_models.push_back(ci_RF);
  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),LOCAL);
  contact_models.push_back(ci_LF);

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;

  initContactDynamics(model,data,contact_models);
  contactDynamics(model,data,q,v,tau,contact_models,mu0);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeContactDynamicsDerivatives(model, data, q, v, tau, contact_models, mu0);  

  //Data_ref
  crba(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeABADerivatives(model, data_ref, q, v, tau, data.contact_forces);
  
  MatrixXd
    v_partial_dq(MatrixXd::Zero(6, model.nv)),
                 a_partial_dq(MatrixXd::Zero(6, model.nv)),
                 a_partial_dv(MatrixXd::Zero(6, model.nv)),
                 a_partial_da(MatrixXd::Zero(6, model.nv));
  
  getJointAccelerationDerivatives(model, data_ref,
                                  model.getJointId(LF),
                                  LOCAL,
                                  v_partial_dq,
                                  a_partial_dq,
                                  a_partial_dv,
                                  a_partial_da);
  MatrixXd K(model.nv+6, model.nv+6);
  K << data_ref.M, a_partial_da.transpose(),
    a_partial_da, MatrixXd::Zero(6,6);
  MatrixXd Kinv = K.inverse();

  MatrixXd J_LF(6, model.nv), J_LF2(6, model.nv);
  J_LF.setZero(); J_LF2.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);
  MatrixXd osim((a_partial_da * data_ref.M.inverse() * a_partial_da.transpose()).inverse());
  BOOST_CHECK(data.osim.isApprox(osim,1e-8));

  MatrixXd df_dq = Kinv.bottomLeftCorner(6, model.nv)* data_ref.dtau_dq + Kinv.bottomRightCorner(6,6)*a_partial_dq;
  
  MatrixXd ddq_dq =  data_ref.M.inverse() * (-data_ref.dtau_dq +  a_partial_da.transpose() * df_dq);
  
  MatrixXd ddq_dq1 = -Kinv.topLeftCorner(model.nv, model.nv) * data_ref.dtau_dq - Kinv.topRightCorner(model.nv, 6) * a_partial_dq;
  
  BOOST_CHECK(df_dq.isApprox(data.dlambda_dq, 1e-8));
  BOOST_CHECK(ddq_dq.isApprox(data.ddq_dq, 1e-8));
  BOOST_CHECK(ddq_dq.isApprox(ddq_dq1, 1e-8));
}

BOOST_AUTO_TEST_SUITE_END ()
