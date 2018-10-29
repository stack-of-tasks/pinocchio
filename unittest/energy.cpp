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

#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/crba.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_kinetic_energy)
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model);
  
  VectorXd q = VectorXd::Zero(model.nq);
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  
  data.M.fill(0);  crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  double kinetic_energy_ref = 0.5*v.transpose() * data.M * v;
  double kinetic_energy = kineticEnergy(model, data, q, v);
  
  BOOST_CHECK_SMALL(kinetic_energy_ref - kinetic_energy, 1e-12);
}

BOOST_AUTO_TEST_SUITE_END()
