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

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <boost/test/unit_test.hpp>

#include <iostream>

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_against_algo )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidRandom(model);
  se3::Data data(model); data.M.fill (0.);
  se3::Data data_other(model); data_other.M.fill (0.);

  VectorXd q (VectorXd::Random(model.nq));
  VectorXd v (VectorXd::Random(model.nv));

  // -------
  q.setZero ();
  v.setZero ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  getJacobianComFromCrba(model, data_other);
  computeJointJacobians(model,data_other,q);
  centerOfMass(model, data_other, q, v, true);
  kineticEnergy(model, data_other, q, v, true);
  potentialEnergy(model, data_other, q, true);

  BOOST_CHECK (data.nle.isApprox(data_other.nle, 1e-12));
  BOOST_CHECK (Eigen::MatrixXd(data.M.triangularView<Eigen::Upper>())
              .isApprox(Eigen::MatrixXd(data_other.M.triangularView<Eigen::Upper>()), 1e-12));
  BOOST_CHECK (data.J.isApprox(data_other.J, 1e-12));
  BOOST_CHECK (data.Jcom.isApprox(data_other.Jcom, 1e-12));
  
  for (int k=0; k<model.njoints; ++k)
  {
    BOOST_CHECK (data.com[(size_t)k].isApprox(data_other.com[(size_t)k], 1e-12));
    BOOST_CHECK (data.vcom[(size_t)k].isApprox(data_other.vcom[(size_t)k], 1e-12));
    BOOST_CHECK_CLOSE(data.mass[(size_t)k], data_other.mass[(size_t)k], 1e-12);
  }
  
  BOOST_CHECK_CLOSE(data.kinetic_energy, data_other.kinetic_energy, 1e-12);
  BOOST_CHECK_CLOSE(data.potential_energy, data_other.potential_energy, 1e-12);

  // -------
  q.setZero ();
  v.setOnes ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  getJacobianComFromCrba(model, data_other);
  computeJointJacobians(model,data_other,q);
  centerOfMass(model, data_other, q, v, true);
  kineticEnergy(model, data_other, q, v, true);
  potentialEnergy(model, data_other, q, true);

  BOOST_CHECK (data.nle.isApprox(data_other.nle, 1e-12));
  BOOST_CHECK (Eigen::MatrixXd(data.M.triangularView<Eigen::Upper>())
              .isApprox(Eigen::MatrixXd(data_other.M.triangularView<Eigen::Upper>()), 1e-12));
  BOOST_CHECK (data.J.isApprox(data_other.J, 1e-12));
  BOOST_CHECK (data.Jcom.isApprox(data_other.Jcom, 1e-12));
  
  for (int k=0; k<model.njoints; ++k)
  {
    BOOST_CHECK (data.com[(size_t)k].isApprox(data_other.com[(size_t)k], 1e-12));
    BOOST_CHECK (data.vcom[(size_t)k].isApprox(data_other.vcom[(size_t)k], 1e-12));
    BOOST_CHECK_CLOSE(data.mass[(size_t)k], data_other.mass[(size_t)k], 1e-12);
  }
  
  BOOST_CHECK_CLOSE(data.kinetic_energy, data_other.kinetic_energy, 1e-12);
  BOOST_CHECK_CLOSE(data.potential_energy, data_other.potential_energy, 1e-12);

//   -------
  q.setOnes ();
  q.segment<4> (3).normalize();
  v.setOnes ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  getJacobianComFromCrba(model, data_other);
  computeJointJacobians(model,data_other,q);
  centerOfMass(model, data_other, q, v, true);
  kineticEnergy(model, data_other, q, v, true);
  potentialEnergy(model, data_other, q, true);

  BOOST_CHECK (data.nle.isApprox(data_other.nle, 1e-12));
  BOOST_CHECK (Eigen::MatrixXd(data.M.triangularView<Eigen::Upper>())
              .isApprox(Eigen::MatrixXd(data_other.M.triangularView<Eigen::Upper>()), 1e-12));
  BOOST_CHECK (data.J.isApprox(data_other.J, 1e-12));
  BOOST_CHECK (data.Jcom.isApprox(data_other.Jcom, 1e-12));
  
  for (int k=0; k<model.njoints; ++k)
  {
    BOOST_CHECK (data.com[(size_t)k].isApprox(data_other.com[(size_t)k], 1e-12));
    BOOST_CHECK (data.vcom[(size_t)k].isApprox(data_other.vcom[(size_t)k], 1e-12));
    BOOST_CHECK_CLOSE(data.mass[(size_t)k], data_other.mass[(size_t)k], 1e-12);
  }
  
  BOOST_CHECK_CLOSE(data.kinetic_energy, data_other.kinetic_energy, 1e-12);
  BOOST_CHECK_CLOSE(data.potential_energy, data_other.potential_energy, 1e-12);

  // -------
  q.setRandom ();
  q.segment<4> (3).normalize();
  v.setRandom ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  getJacobianComFromCrba(model, data_other);
  computeJointJacobians(model,data_other,q);
  centerOfMass(model, data_other, q, v, true);
  kineticEnergy(model, data_other, q, v, true);
  potentialEnergy(model, data_other, q, true);

  BOOST_CHECK (data.nle.isApprox(data_other.nle, 1e-12));
  BOOST_CHECK (Eigen::MatrixXd(data.M.triangularView<Eigen::Upper>())
              .isApprox(Eigen::MatrixXd(data_other.M.triangularView<Eigen::Upper>()), 1e-12));
  BOOST_CHECK (data.J.isApprox(data_other.J, 1e-12));
  BOOST_CHECK (data.Jcom.isApprox(data_other.Jcom, 1e-12));
  
  for (int k=0; k<model.njoints; ++k)
  {
    BOOST_CHECK (data.com[(size_t)k].isApprox(data_other.com[(size_t)k], 1e-12));
    BOOST_CHECK (data.vcom[(size_t)k].isApprox(data_other.vcom[(size_t)k], 1e-12));
    BOOST_CHECK_CLOSE(data.mass[(size_t)k], data_other.mass[(size_t)k], 1e-12);
  }
  
  BOOST_CHECK_CLOSE(data.kinetic_energy, data_other.kinetic_energy, 1e-12);
  BOOST_CHECK_CLOSE(data.potential_energy, data_other.potential_energy, 1e-12);
}

BOOST_AUTO_TEST_SUITE_END ()
