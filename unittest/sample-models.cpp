//
// Copyright (c) 2015-2018 CNRS
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

#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( build_model_sample_humanoid_random )
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);

  BOOST_CHECK(model.nq == 33);
  BOOST_CHECK(model.nv == 32);

  pinocchio::Model modelff;
  pinocchio::buildModels::humanoidRandom(modelff,false);

  BOOST_CHECK(modelff.nq == 32);
  BOOST_CHECK(modelff.nv == 32);
}

BOOST_AUTO_TEST_CASE ( build_model_sample_manipulator )
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);

  BOOST_CHECK(model.nq == 6);
  BOOST_CHECK(model.nv == 6);

#ifdef PINOCCHIO_WITH_HPP_FCL
  pinocchio::Data data(model);
  pinocchio::GeometryModel geom;
  pinocchio::buildModels::manipulatorGeometries(model,geom);
#endif
}

BOOST_AUTO_TEST_CASE ( build_model_sample_humanoid )
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoid(model);
  pinocchio::Data data(model);

  BOOST_CHECK(model.nq == 35);
  BOOST_CHECK(model.nv == 34);

#ifdef PINOCCHIO_WITH_HPP_FCL
  pinocchio::GeometryModel geom;
  pinocchio::buildModels::humanoidGeometries(model,geom);
  pinocchio::GeometryData geomdata(geom);
  
  Eigen::VectorXd q = pinocchio::neutral(model);
  pinocchio::forwardKinematics(model,data,q);
  pinocchio::updateGeometryPlacements(model,data,geom,geomdata,q);
#endif

  /* We might want to check here the joint namings, and validate the 
   * direct geometry with respect to reference values. */
}

BOOST_AUTO_TEST_SUITE_END()
