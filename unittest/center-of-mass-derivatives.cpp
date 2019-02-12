//
// Copyright (c) 2019 CNRS
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_vcom)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoid(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd vq(VectorXd::Random(model.nv));
  VectorXd aq(VectorXd::Random(model.nv));

  // Compute dvcom_dq using the algorithm
  Data::Matrix3x dvcom_dq = Data::Matrix3x::Zero(3,model.nv);
  computeForwardKinematicsDerivatives(model,data,q,vq,aq);
  centerOfMass(model,data,q,vq);
  getCenterOfMassVelocityDerivatives(model,data,dvcom_dq);

  // Approximate dvcom_dq by finite diff.
  Eigen::Vector3d vcom0 = data.vcom[0];
  const double alpha = 1e-8;
  Eigen::VectorXd dq = VectorXd::Zero(model.nv);
  Data::Matrix3x dvcom_dqn(3,model.nv);

  for(int k = 0; k < model.nv; ++k)
  {
    dq[k] = alpha;
    centerOfMass(model,data,integrate(model,q,dq),vq);
    dvcom_dqn.col(k) = (data.vcom[0]-vcom0)/alpha;
    dq[k] = 0;
  }

  // Check that algo result and finite-diff approx are similar.
  BOOST_CHECK(dvcom_dq.isApprox(dvcom_dqn,sqrt(alpha)));
}



BOOST_AUTO_TEST_SUITE_END()
