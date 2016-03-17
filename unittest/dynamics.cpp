//
// Copyright (c) 2016 CNRS
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

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/dynamics.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE DynamicTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


BOOST_AUTO_TEST_SUITE ( DynamicsTest)

BOOST_AUTO_TEST_CASE ( test_FD )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  se3::buildModels::humanoidSimple(model,true);
  se3::Data data(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  
  se3::computeJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  
  const std::string RF = "rleg6_body";
  const std::string LF = "lleg6_body";
  
  Data::Matrix6x J_RF (6, model.nv);
  getJacobian <true> (model, data, model.getBodyId(RF), J_RF);
  Data::Matrix6x J_LF (6, model.nv);
  getJacobian <true> (model, data, model.getBodyId(LF), J_LF);
  
  Eigen::MatrixXd J (12, model.nv);
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_LF;
  
  Eigen::VectorXd gamma (VectorXd::Ones(12));
  
  Eigen::MatrixXd H(J.transpose());
  
  se3::forwardDynamics(model, data, q, v, tau, J, gamma, true);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  MatrixXd Minv (data.M.inverse());
  MatrixXd JMinvJt (J * Minv * J.transpose());
  
  Eigen::MatrixXd G_ref(J.transpose());
  cholesky::Uiv(model, data, G_ref);
  for(int k=0;k<model.nv;++k) G_ref.row(k) /= sqrt(data.D[k]);
    Eigen::MatrixXd H_ref(G_ref.transpose() * G_ref);
    BOOST_CHECK(H_ref.isApprox(JMinvJt,1-12));
  
  VectorXd lambda_ref = -JMinvJt.inverse() * (J*Minv*(tau - data.nle) + gamma);
  BOOST_CHECK(data.lambda.isApprox(lambda_ref, 1e-12));
    
  VectorXd a_ref = Minv*(tau - data.nle + J.transpose()*lambda_ref);
  
  Eigen::VectorXd dynamics_residual_ref (data.M * a_ref + data.nle - tau - J.transpose()*lambda_ref);
  BOOST_CHECK(dynamics_residual_ref.norm() <= 1e-12);
  
  Eigen::VectorXd constraint_residual (J * data.ddq + gamma);
  BOOST_CHECK(constraint_residual.norm() <= 1e-12);
  
  Eigen::VectorXd dynamics_residual (data.M * data.ddq + data.nle - tau - J.transpose()*data.lambda);
  BOOST_CHECK(dynamics_residual.norm() <= 1e-12);
}

BOOST_AUTO_TEST_CASE (timings_fd_llt)
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  se3::buildModels::humanoidSimple(model,true);
  se3::Data data(model);
  
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
  
  se3::computeJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  
  const std::string RF = "rleg6_body";
  const std::string LF = "lleg6_body";
  
  Data::Matrix6x J_RF (6, model.nv);
  getJacobian <true> (model, data, model.getBodyId(RF), J_RF);
  Data::Matrix6x J_LF (6, model.nv);
  getJacobian <true> (model, data, model.getBodyId(LF), J_LF);
  
  Eigen::MatrixXd J (12, model.nv);
  J.topRows<6> () = J_RF;
  J.bottomRows<6> () = J_LF;
  
  Eigen::VectorXd gamma (VectorXd::Ones(12));
  
  
  q = Eigen::VectorXd::Zero(model.nq);
  
  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(NBT)
  {
    se3::forwardDynamics(model, data, q, v, tau, J, gamma, true);
  }
  timer.toc(std::cout,NBT);
  
}

BOOST_AUTO_TEST_SUITE_END ()
