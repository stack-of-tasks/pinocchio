//
// Copyright (c) 2016-2018 CNRS
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

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_aba_simple )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidSimple(model);
  
  se3::Data data(model);
  se3::Data data_ref(model);

  VectorXd q = VectorXd::Ones(model.nq);
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  
  computeAllTerms(model, data_ref, q, v);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
    = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  tau = data_ref.M * a + data_ref.nle;
  aba(model, data, q, v, tau);
  
  BOOST_CHECK(data.ddq.isApprox(a, 1e-12));
  
}

BOOST_AUTO_TEST_CASE ( test_aba_with_fext )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model; buildModels::humanoidSimple(model);
  
  se3::Data data(model);
  
  VectorXd q = VectorXd::Random(model.nq);
  q.segment<4>(3).normalize();
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  container::aligned_vector<Force> fext(model.joints.size(), Force::Random());
  
  crba(model, data, q);
  computeJacobians(model, data, q);
  nonLinearEffects(model, data, q, v);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  

  VectorXd tau = data.M * a + data.nle;
  Data::Matrix6x J = Data::Matrix6x::Zero(6, model.nv);
  for(Model::Index i=1;i<(Model::Index)model.njoints;++i) {
    getJacobian<LOCAL>(model, data, i, J);
    tau -= J.transpose()*fext[i].toVector();
    J.setZero();
  }
  aba(model, data, q, v, tau, fext);
  
  BOOST_CHECK(data.ddq.isApprox(a, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_aba_vs_rnea )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model; buildModels::humanoidSimple(model);
  
  se3::Data data(model);
  se3::Data data_ref(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Zero(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  
  crba(model, data_ref, q);
  nonLinearEffects(model, data_ref, q, v);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  tau = data_ref.M * a + data_ref.nle;
  aba(model, data, q, v, tau);
  
  VectorXd tau_ref = rnea(model, data_ref, q, v, a);
  BOOST_CHECK(tau_ref.isApprox(tau, 1e-12));
  
  
  BOOST_CHECK(data.ddq.isApprox(a, 1e-12));
  
}

BOOST_AUTO_TEST_CASE ( test_computeMinverse )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  buildModels::humanoidSimple(model);
  model.gravity.setZero();
  
  se3::Data data(model);
  se3::Data data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);

  crba(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>()
  = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  MatrixXd Minv_ref(data_ref.M.inverse());

  computeMinverse(model, data, q);

  
  BOOST_CHECK(data.Minv.topRows<6>().isApprox(Minv_ref.topRows<6>()));
  
  data.Minv.triangularView<Eigen::StrictlyLower>()
  = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  
  BOOST_CHECK(data.Minv.isApprox(Minv_ref));
  
//  std::cout << "Minv:\n" << data.Minv.block<10,10>(0,0) << std::endl;
//  std::cout << "Minv_ref:\n" << Minv_ref.block<10,10>(0,0) << std::endl;
//
//  std::cout << "Minv:\n" << data.Minv.bottomRows<10>() << std::endl;
//  std::cout << "Minv_ref:\n" << Minv_ref.bottomRows<10>() << std::endl;
  
}
BOOST_AUTO_TEST_SUITE_END ()
