//
// Copyright(c) 2018-2020 CNRS INRIA
//

#include "pinocchio/algorithm/copy.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_data_copy)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  model.upperPositionLimit.head<3>().fill(100);
  model.upperPositionLimit.segment<4>(3).setOnes();
  model.lowerPositionLimit.head<7>() = - model.upperPositionLimit.head<7>();
  
  VectorXd q(model.nq);
  q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  Data data_ref(model), data(model);
  forwardKinematics(model,data_ref,q,v,a);
  rnea(model,data_ref,q,v,a); // for a_gf to be initialized
  
  // Check zero order kinematic quantities
  copy(model,data_ref,data,POSITION);
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
  }
  
  // Check first order kinematic quantities
  copy(model,data_ref,data,VELOCITY);
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
    BOOST_CHECK(data.v[i] == data_ref.v[i]);
  }
  
  // Check second order kinematic quantities
  copy(model,data_ref,data,ACCELERATION);
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
    BOOST_CHECK(data.v[i] == data_ref.v[i]);
    BOOST_CHECK(data.a[i] == data_ref.a[i]);
    BOOST_CHECK(data.a_gf[i] == data_ref.a_gf[i]);
  }
  
}

BOOST_AUTO_TEST_SUITE_END()
