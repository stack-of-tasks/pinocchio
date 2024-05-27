//
// Copyright (c) 2023 INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/constraints/constraints.hpp"
#include "pinocchio/algorithm/constraints/constraint-model-generic.hpp"
#include "pinocchio/algorithm/constraints/constraint-data-generic.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;
using namespace Eigen;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_variants)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  Data data(model);

  const SE3 M(SE3::Random());
  RigidConstraintModel rcm(CONTACT_3D, model, 0, M);
  RigidConstraintData rcd(rcm);

  ConstraintModel::ConstraintModelVariant constraint_model_variant = rcm;
  ConstraintModel constraint_model(rcm);
  ConstraintModel constraint_model_equal = rcm;

  ConstraintData constraint_data = rcm.createData();
}

BOOST_AUTO_TEST_CASE(contact_visitors)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  Data data(model);

  const SE3 M(SE3::Random());
  RigidConstraintModel rcm(CONTACT_3D, model, 0, M);
  RigidConstraintData rcd(rcm);
  BOOST_CHECK(ConstraintData(rcd) == ConstraintData(rcd));
  BOOST_CHECK(ConstraintData(rcd) == rcd);

  ConstraintModel constraint_model(rcm);

  // Test create data visitor
  ConstraintData constraint_data = createData(constraint_model);
  constraint_data = rcd;

  // Test calc visitor
  calc(constraint_model, constraint_data, model, data);
  rcm.calc(model, data, rcd);
  BOOST_CHECK(rcd == constraint_data);

  // Test jacobian visitor
  Data::MatrixXs jacobian_matrix = Data::Matrix6x::Zero(6, model.nv),
                 jacobian_matrix_ref = Data::Matrix6x::Zero(6, model.nv);
  jacobian(constraint_model, constraint_data, model, data, jacobian_matrix);
  rcm.jacobian(model, data, rcd, jacobian_matrix_ref);
  BOOST_CHECK(jacobian_matrix == jacobian_matrix_ref);
}

BOOST_AUTO_TEST_SUITE_END()
