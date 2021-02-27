//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "utils/model-generator.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_contact_dynamics_code_generation)
{
  typedef double Scalar;

  typedef pinocchio::ModelTpl<Scalar> Model;   

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  typedef Model::ConfigVectorType ConfigVectorType;
  typedef Model::TangentVectorType TangentVectorType;
  ConfigVectorType q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVectorType v(TangentVectorType::Random(model.nv));
  TangentVectorType tau(TangentVectorType::Random(model.nv));

  
}

BOOST_AUTO_TEST_SUITE_END()
