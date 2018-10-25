//
// Copyright (c) 2018 CNRS
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
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_aba)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));
  VectorXd a(aba(model,data_ref,q,v,tau));
  
  MatrixXd aba_partial_dq(model.nv,model.nv); aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv,model.nv); aba_partial_dv.setZero();
  Data::RowMatrixXd aba_partial_dtau(model.nv,model.nv); aba_partial_dtau.setZero();
  
  computeABADerivatives(model, data, q, v, tau, aba_partial_dq, aba_partial_dv, aba_partial_dtau);
  computeRNEADerivatives(model,data_ref,q,v,a);
  for(Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.ov[k]));
    BOOST_CHECK(data.oa[k].isApprox(data_ref.oa[k]));
    BOOST_CHECK(data.of[k].isApprox(data_ref.of[k]));
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oYcrb[k]));
    BOOST_CHECK(data.doYcrb[k].isApprox(data_ref.doYcrb[k]));
  }
  
  computeJointJacobians(model,data_ref,q);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  
  aba(model,data_ref,q,v,tau);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  
  computeMinverse(model,data_ref,q);
  data_ref.Minv.triangularView<Eigen::StrictlyLower>()
  = data_ref.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  
  BOOST_CHECK(aba_partial_dtau.isApprox(data_ref.Minv));

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dtau_dq.isApprox(data_ref.dtau_dq));
  BOOST_CHECK(data.dtau_dv.isApprox(data_ref.dtau_dv));
  
  MatrixXd aba_partial_dq_fd(model.nv,model.nv); aba_partial_dq_fd.setZero();
  MatrixXd aba_partial_dv_fd(model.nv,model.nv); aba_partial_dv_fd.setZero();
  MatrixXd aba_partial_dtau_fd(model.nv,model.nv); aba_partial_dtau_fd.setZero();
  
  Data data_fd(model);
  VectorXd a0 = aba(model,data_fd,q,v,tau);
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd a_plus(model.nv);
  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    a_plus = aba(model,data_fd,q_plus,v,tau);

    aba_partial_dq_fd.col(k) = (a_plus - a0)/alpha;
    v_eps[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_fd,sqrt(alpha)));
  
  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    a_plus = aba(model,data_fd,q,v_plus,tau);
    
    aba_partial_dv_fd.col(k) = (a_plus - a0)/alpha;
    v_plus[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_fd,sqrt(alpha)));
  
  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    a_plus = aba(model,data_fd,q,v,tau_plus);
    
    aba_partial_dtau_fd.col(k) = (a_plus - a0)/alpha;
    tau_plus[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_fd,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_aba_minimal_argument)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));
  VectorXd a(aba(model,data_ref,q,v,tau));
  
  MatrixXd aba_partial_dq(model.nv,model.nv); aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv,model.nv); aba_partial_dv.setZero();
  Data::RowMatrixXd aba_partial_dtau(model.nv,model.nv); aba_partial_dtau.setZero();
  
  computeABADerivatives(model, data_ref, q, v, tau, aba_partial_dq, aba_partial_dv, aba_partial_dtau);
  
  computeABADerivatives(model, data, q, v, tau);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dtau_dq.isApprox(data_ref.dtau_dq));
  BOOST_CHECK(data.dtau_dv.isApprox(data_ref.dtau_dv));
  BOOST_CHECK(data.Minv.isApprox(aba_partial_dtau));
  BOOST_CHECK(data.ddq_dq.isApprox(aba_partial_dq));
  BOOST_CHECK(data.ddq_dv.isApprox(aba_partial_dv));
}

BOOST_AUTO_TEST_SUITE_END()
