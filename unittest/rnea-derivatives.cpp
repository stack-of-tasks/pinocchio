//
// Copyright (c) 2017-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_generalized_gravity_derivatives)
{
  using namespace Eigen;
  using namespace se3;

  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_fd(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Zero(model.nv));
  VectorXd a(VectorXd::Zero(model.nv));
  
  /// Check againt non-derivative algo
  MatrixXd g_partial_dq(model.nv,model.nv); g_partial_dq.setZero();
  computeGeneralizedGravityDerivatives(model,data,q,g_partial_dq);
  
  VectorXd g0 = computeGeneralizedGravity(model,data_fd,q);
  BOOST_CHECK(data.g.isApprox(g0));

  MatrixXd g_partial_dq_fd(model.nv,model.nv); g_partial_dq_fd.setZero();

  VectorXd v_eps(Eigen::VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd g_plus(model.nv);
  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    g_plus = computeGeneralizedGravity(model,data_fd,q_plus);
    
    g_partial_dq_fd.col(k) = (g_plus - g0)/alpha;
    v_eps[k] -= alpha;
  }
  
  BOOST_CHECK(g_partial_dq.isApprox(g_partial_dq_fd,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_rnea_derivatives)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_fd(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  /// Check againt computeGeneralizedGravityDerivatives
  MatrixXd rnea_partial_dq(model.nv,model.nv); rnea_partial_dq.setZero();
  MatrixXd rnea_partial_dv(model.nv,model.nv); rnea_partial_dv.setZero();
  MatrixXd rnea_partial_da(model.nv,model.nv); rnea_partial_da.setZero();
  computeRNEADerivatives(model,data,q,0*v,0*a,rnea_partial_dq,rnea_partial_dv,rnea_partial_da);
  
  MatrixXd g_partial_dq(model.nv,model.nv); g_partial_dq.setZero();
  computeGeneralizedGravityDerivatives(model,data_ref,q,g_partial_dq);
  
  BOOST_CHECK(rnea_partial_dq.isApprox(g_partial_dq));
  BOOST_CHECK(data.tau.isApprox(data_ref.g));
  
  VectorXd tau0 = rnea(model,data_fd,q,0*v,0*a);
  MatrixXd rnea_partial_dq_fd(model.nv,model.nv); rnea_partial_dq_fd.setZero();
  
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    tau_plus = rnea(model,data_fd,q_plus,0*v,0*a);
    
    rnea_partial_dq_fd.col(k) = (tau_plus - tau0)/alpha;
    v_eps[k] -= alpha;
  }
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd,sqrt(alpha)));

  // Check with q and a non zero
  tau0 = rnea(model,data_fd,q,0*v,a);
  rnea_partial_dq_fd.setZero();

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    tau_plus = rnea(model,data_fd,q_plus,0*v,a);

    rnea_partial_dq_fd.col(k) = (tau_plus - tau0)/alpha;
    v_eps[k] -= alpha;
  }
  
  rnea_partial_dq.setZero();
  computeRNEADerivatives(model,data,q,0*v,a,rnea_partial_dq,rnea_partial_dv,rnea_partial_da);
  forwardKinematics(model,data_ref,q,0*v,a);
  
  for(Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.a[k].isApprox(data_ref.a[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.oh[k].isApprox(Force::Zero()));
  }
  
  BOOST_CHECK(data.tau.isApprox(tau0));
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd,sqrt(alpha)));
  
  // Check with q and v non zero
  const Motion gravity(model.gravity);
  model.gravity.setZero();
  tau0 = rnea(model,data_fd,q,v,0*a);
  rnea_partial_dq_fd.setZero();
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    tau_plus = rnea(model,data_fd,q_plus,v,0*a);
    
    rnea_partial_dq_fd.col(k) = (tau_plus - tau0)/alpha;
    v_eps[k] -= alpha;
  }
  
  VectorXd v_plus(v);
  MatrixXd rnea_partial_dv_fd(model.nv,model.nv); rnea_partial_dv_fd.setZero();
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    tau_plus = rnea(model,data_fd,q,v_plus,0*a);
    
    rnea_partial_dv_fd.col(k) = (tau_plus - tau0)/alpha;
    v_plus[k] -= alpha;
  }
  
  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  computeRNEADerivatives(model,data,q,v,0*a,rnea_partial_dq,rnea_partial_dv,rnea_partial_da);
  forwardKinematics(model,data_ref,q,v,0*a);
  
  for(Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.a[k].isApprox(data_ref.a[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
  }
  
  BOOST_CHECK(data.tau.isApprox(tau0));
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd,sqrt(alpha)));
  BOOST_CHECK(rnea_partial_dv.isApprox(rnea_partial_dv_fd,sqrt(alpha)));
  
//    std::cout << "rnea_partial_dv:\n" << rnea_partial_dv.block<10,10>(0,0) << std::endl;
//    std::cout << "rnea_partial_dv ref:\n" << rnea_partial_dv_fd.block<10,10>(0,0) << std::endl;
//    std::cout << "rnea_partial_dv:\n" << rnea_partial_dv.topRows<10>() << std::endl;
//    std::cout << "rnea_partial_dv ref:\n" << rnea_partial_dv_fd.topRows<10>() << std::endl;
  // Check with q, v and a non zero
  model.gravity = gravity;
  v_plus = v;
  tau0 = rnea(model,data_fd,q,v,a);
  rnea_partial_dq_fd.setZero();
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    tau_plus = rnea(model,data_fd,q_plus,v,a);
    
    rnea_partial_dq_fd.col(k) = (tau_plus - tau0)/alpha;
    v_eps[k] -= alpha;
  }
  
  rnea_partial_dv_fd.setZero();
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    tau_plus = rnea(model,data_fd,q,v_plus,a);
    
    rnea_partial_dv_fd.col(k) = (tau_plus - tau0)/alpha;
    v_plus[k] -= alpha;
  }
  
  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  computeRNEADerivatives(model,data,q,v,a,rnea_partial_dq,rnea_partial_dv,rnea_partial_da);
  forwardKinematics(model,data_ref,q,v,a);
  
  for(Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.a[k].isApprox(data_ref.a[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
  }
  
  computeJointJacobiansTimeVariation(model,data_ref,q,v);
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  crba(model,data_ref,q);
  
  rnea_partial_da.triangularView<Eigen::StrictlyLower>()
  = rnea_partial_da.transpose().triangularView<Eigen::StrictlyLower>();
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK(rnea_partial_da.isApprox(data_ref.M));

  BOOST_CHECK(data.tau.isApprox(tau0));
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd,sqrt(alpha)));
  BOOST_CHECK(rnea_partial_dv.isApprox(rnea_partial_dv_fd,sqrt(alpha)));
  
  Data data2(model);
  computeRNEADerivatives(model,data2,q,v,a);
  data2.M.triangularView<Eigen::StrictlyLower>()
  = data2.M.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK(rnea_partial_dq.isApprox(data2.dtau_dq));
  BOOST_CHECK(rnea_partial_dv.isApprox(data2.dtau_dv));
  BOOST_CHECK(rnea_partial_da.isApprox(data2.M));
  
}

BOOST_AUTO_TEST_SUITE_END()
