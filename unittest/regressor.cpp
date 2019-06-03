//
// Copyright (c) 2018 CNRS
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_static_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model; buildModels::humanoidRandom(model);
  
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);
  
  VectorXd q = randomConfiguration(model);
  regressor::computeStaticRegressor(model,data,q);
  
  VectorXd phi(4*(model.njoints-1));
  for(int k = 1; k < model.njoints; ++k)
  {
    const Inertia & Y = model.inertias[(size_t)k];
    phi.segment<4>(4*(k-1)) << Y.mass(), Y.mass() * Y.lever();
  }
  
  Vector3d com = centerOfMass(model,data_ref,q);
  Vector3d static_com_ref;
  static_com_ref <<  com;
  
  Vector3d static_com = data.staticRegressor * phi;
  
  BOOST_CHECK(static_com.isApprox(static_com_ref)); 
}

BOOST_AUTO_TEST_CASE(test_body_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  Inertia I(Inertia::Random());
  Motion v(Motion::Random());
  Motion a(Motion::Random());

  Force f = I*a + I.vxiv(v);

  Inertia::Vector6 f_regressor = bodyRegressor(v,a) * I.toDynamicParameters();

  BOOST_CHECK(f_regressor.isApprox(f.toVector()));
}

BOOST_AUTO_TEST_CASE(test_joint_body_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::manipulator(model);
  pinocchio::Data data(model);

  JointIndex JOINT_ID = JointIndex(model.njoints) - 1;

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd a = Eigen::VectorXd::Random(model.nv);

  rnea(model,data,q,v,a);

  Force f = data.f[JOINT_ID];

  Inertia::Vector6 f_regressor = jointBodyRegressor(model,data,JOINT_ID) * model.inertias[JOINT_ID].toDynamicParameters();

  BOOST_CHECK(f_regressor.isApprox(f.toVector()));
}

BOOST_AUTO_TEST_SUITE_END()
