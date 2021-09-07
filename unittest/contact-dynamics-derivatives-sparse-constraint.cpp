//
// Copyright (c) 2020 CNRS INRIA
//

#include <iostream>

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;


BOOST_AUTO_TEST_CASE(test_sparse_constraint_dynamics_derivatives)
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_LF(CONTACT_6D,LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  RigidConstraintModel ci_RF(CONTACT_3D,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();

  contact_models.push_back(ci_LF); contact_data.push_back(RigidConstraintData(ci_LF));
  contact_models.push_back(ci_RF); contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(model,data,contact_models,contact_data);

  // Reference values
  crba(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  container::aligned_vector<Force> fext((size_t)model.njoints,Force::Zero());
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = contact_models[k];
    const RigidConstraintData & cdata = contact_data[k];
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

BOOST_AUTO_TEST_SUITE_END ()
