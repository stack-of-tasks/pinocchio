//
// Copyright (c) 2016-2020 CNRS INRIA
//

#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinetic_energy)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Data data(model);

  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model, -qmax, qmax);
  VectorXd v = VectorXd::Random(model.nv);

  crba(model, data, q, Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  double kinetic_energy_ref = 0.5 * v.transpose() * data.M * v;
  double kinetic_energy = computeKineticEnergy(model, data, q, v);

  BOOST_CHECK_SMALL(kinetic_energy_ref - kinetic_energy, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_kinetic_energy_with_armature)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Data data(model), data_ref(model);

  model.armature = VectorXd::Random(model.nv) + VectorXd::Ones(model.nv);

  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model, -qmax, qmax);
  VectorXd v = VectorXd::Random(model.nv);

  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  double kinetic_energy_ref = 0.5 * v.transpose() * data_ref.M * v;
  double kinetic_energy = computeKineticEnergy(model, data, q, v);

  BOOST_CHECK_SMALL(
    kinetic_energy_ref - kinetic_energy, Eigen::NumTraits<double>::dummy_precision());
}

BOOST_AUTO_TEST_CASE(test_potential_energy)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Data data(model), data_ref(model);

  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model, -qmax, qmax);

  double potential_energy = computePotentialEnergy(model, data, q);
  centerOfMass(model, data_ref, q);

  double potential_energy_ref = -data_ref.mass[0] * (data_ref.com[0].dot(model.gravity.linear()));

  BOOST_CHECK_SMALL(
    potential_energy_ref - potential_energy, Eigen::NumTraits<double>::dummy_precision());
}

BOOST_AUTO_TEST_CASE(test_mechanical_energy)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Data data(model), data_ref(model);

  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model, -qmax, qmax);
  VectorXd v = VectorXd::Random(model.nv);

  computeKineticEnergy(model, data_ref, q, v);
  computePotentialEnergy(model, data_ref, q);

  const double mechanical_energy_ref = data_ref.kinetic_energy + data_ref.potential_energy;
  double mechanical_energy = computeMechanicalEnergy(model, data, q, v);

  BOOST_CHECK_SMALL(
    mechanical_energy_ref - mechanical_energy, Eigen::NumTraits<double>::dummy_precision());
}

template<typename ConfigVectorType, typename TangentVectorType>
Eigen::VectorXd evalMv(
  const pinocchio::Model & model,
  const Eigen::MatrixBase<ConfigVectorType> & q,
  const Eigen::MatrixBase<TangentVectorType> & v)
{
  pinocchio::Data data(model);
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data.M * v;
}

BOOST_AUTO_TEST_CASE(test_against_rnea)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  model.armature = VectorXd::Random(model.nv) + VectorXd::Ones(model.nv);
  model.armature.head<6>().setZero(); // Because we do not take into account the influcent of the
                                      // armature on the Coriolis terms
                                      //  model.gravity.setZero();
  Data data(model), data_ref(model);

  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model, -qmax, qmax);
  VectorXd v = VectorXd::Random(model.nv); // v.setZero();
  VectorXd a = VectorXd::Random(model.nv); // a.setZero();

  const VectorXd tau_ref = rnea(model, data_ref, q, v, a);
  VectorXd tau_fd = VectorXd::Zero(model.nv);

  const double eps = 1e-8;
  computeMechanicalEnergy(model, data, q, v);
  const double mechanical_energy = data.kinetic_energy - data.potential_energy;
  forwardKinematics(model, data, q);
  const SE3 oM1 = data.oMi[1];
  Data data_plus(model);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    VectorXd q_plus = integrate(model, q, VectorXd::Unit(model.nv, k) * eps);
    computeMechanicalEnergy(model, data_plus, q_plus, v);
    const double mechanical_energy_plus = data_plus.kinetic_energy - data_plus.potential_energy;
    const SE3 oM1_plus = data_plus.oMi[1];
    const SE3 Mdiff = oM1.actInv(oM1_plus);
    if (k < 6)
    {
      Force f = Force::Zero();
      f.toVector()[k] = mechanical_energy;
      Force f_plus = Force::Zero();
      f_plus.toVector()[k] = mechanical_energy_plus;
      tau_fd.head<6>() -= (Mdiff.act(f_plus) - f).toVector() / eps;
    }
    else
    {
      tau_fd[k] = -(mechanical_energy_plus - mechanical_energy) / eps;
    }
  }

  const VectorXd Mv = evalMv(model, q, v);
  const VectorXd q_plus = integrate(model, q, v * eps);
  const VectorXd v_plus = v + a * eps;
  VectorXd Mv_plus = evalMv(model, q_plus, v_plus);

  forwardKinematics(model, data, q_plus);
  const SE3 oM1_plus = data.oMi[1];
  const SE3 Mdiff = oM1.actInv(oM1_plus);
  Mv_plus.head<6>() =
    Mdiff.act(Force(Mv_plus.head<6>()))
      .toVector(); // The torque at the free flyer level is expressed in a translated frame.

  tau_fd += (Mv_plus - Mv) / eps;
  std::cout << "tau_fd: " << tau_fd.transpose() << std::endl;
  std::cout << "tau_ref: " << tau_ref.transpose() << std::endl;

  BOOST_CHECK(tau_fd.isApprox(tau_ref, sqrt(eps)));
}

BOOST_AUTO_TEST_SUITE_END()
