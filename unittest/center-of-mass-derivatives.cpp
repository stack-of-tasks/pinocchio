//
// Copyright (c) 2019-2021 CNRS INRIA
//

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_vcom)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoid(model);

  Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd vq(VectorXd::Random(model.nv));
  VectorXd aq(VectorXd::Random(model.nv));

  // Approximate dvcom_dq by finite diff.
  centerOfMass(model, data_ref, q, vq);
  const Eigen::Vector3d vcom0 = data_ref.vcom[0];
  const double alpha = 1e-8;
  Eigen::VectorXd dq = VectorXd::Zero(model.nv);
  Data::Matrix3x dvcom_dqn(3, model.nv);

  for (int k = 0; k < model.nv; ++k)
  {
    dq[k] = alpha;
    centerOfMass(model, data_ref, integrate(model, q, dq), vq);
    dvcom_dqn.col(k) = (data_ref.vcom[0] - vcom0) / alpha;
    dq[k] = 0;
  }

  {
    // Compute dvcom_dq using the algorithm
    Data data(model);
    Data::Matrix3x dvcom_dq = Data::Matrix3x::Zero(3, model.nv);
    centerOfMass(model, data, q, vq);
    getCenterOfMassVelocityDerivatives(model, data, dvcom_dq);

    // Check that algo result and finite-diff approx are similar.
    BOOST_CHECK(dvcom_dq.isApprox(dvcom_dqn, sqrt(alpha)));
  }

  {
    // Compute dvcom_dq using the algorithm
    Data data(model);
    Data::Matrix3x dvcom_dq = Data::Matrix3x::Zero(3, model.nv);
    computeAllTerms(model, data, q, vq);
    getCenterOfMassVelocityDerivatives(model, data, dvcom_dq);

    // Check that algo result and finite-diff approx are similar.
    BOOST_CHECK(dvcom_dq.isApprox(dvcom_dqn, sqrt(alpha)));
  }
}

BOOST_AUTO_TEST_SUITE_END()
