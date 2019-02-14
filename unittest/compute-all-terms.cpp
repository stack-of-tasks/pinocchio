//
// Copyright (c) 2015-2018 CNRS
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
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
  using namespace pinocchio;

  pinocchio::Model model; buildModels::humanoidRandom(model);
  pinocchio::Data data(model); data.M.fill (0.);
  pinocchio::Data data_other(model); data_other.M.fill (0.);

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

BOOST_AUTO_TEST_CASE ( test_with_armature_algo )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model; buildModels::humanoidRandom(model);
  pinocchio::Data data(model); data.M.fill (0.);
  pinocchio::Data data_other(model); data_other.M.fill (0.);

  VectorXd q (VectorXd::Random(model.nq));
  pinocchio::normalize(model, q);
  VectorXd v (VectorXd::Random(model.nv));

  // ----Check that Original computeAllTerms still works-

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
  // ----Check that computeAllTerms with armature works---
  VectorXd armature(VectorXd::Random(model.nv));  //Random between -1,1
  armature.array() += 1.0;  armature /=2.0;   //Move armature between 0, 1.
  armature.head<6>().setZero();              // Set ff to zero
  model.armature = armature;
  computeAllTerms(model, data, q, v);
  
  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  getJacobianComFromCrba(model, data_other);
  computeJointJacobians(model,data_other,q);
  centerOfMass(model, data_other, q, v, true);
  kineticEnergy(model, data_other, q, v, true);
  potentialEnergy(model, data_other, q, true);

  data_other.M.diagonal() += armature;

  BOOST_CHECK (data.nle.isApprox(data_other.nle, 1e-12));
  BOOST_CHECK (Eigen::MatrixXd(data.M.triangularView<Eigen::Upper>())
              .isApprox(Eigen::MatrixXd(data_other.M.triangularView<Eigen::Upper>()), 1e-12));
  BOOST_CHECK (data.J.isApprox(data_other.J, 1e-12));
  BOOST_CHECK (data.Jcom.isApprox(data_other.Jcom, 1e-12));
  BOOST_CHECK_CLOSE(data.kinetic_energy, data_other.kinetic_energy, 1e-12);
  BOOST_CHECK_CLOSE(data.potential_energy, data_other.potential_energy, 1e-12);
}

BOOST_AUTO_TEST_SUITE_END ()
