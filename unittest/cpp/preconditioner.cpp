//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE preconditioner

#include "pinocchio/algorithm/diagonal-preconditioner.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(diagonal_preconditioner)
{
  Eigen::Index n = 10;
  Eigen::VectorXd precond_vec = Eigen::VectorXd::Random(n);
  precond_vec = precond_vec.array().abs() + 1e-6;
  DiagonalPreconditionerTpl<Eigen::VectorXd> precond(precond_vec);

  Eigen::VectorXd x = Eigen::VectorXd::Random(n);
  Eigen::VectorXd x_scaled;

  precond.scale(x, x_scaled);
  Eigen::VectorXd x_scaled_true = x.array() / precond_vec.array();
  BOOST_CHECK(x_scaled.isApprox(x_scaled_true));

  precond.scaleSquare(x, x_scaled);
  x_scaled_true.array() = x.array() / (precond_vec.array() * precond_vec.array());
  BOOST_CHECK(x_scaled.isApprox(x_scaled_true));

  Eigen::VectorXd x_unscaled;
  precond.unscale(x, x_unscaled);
  Eigen::VectorXd x_unscaled_true = x.array() * precond_vec.array();
  BOOST_CHECK(x_unscaled.isApprox(x_unscaled_true));

  precond.unscaleSquare(x, x_unscaled);
  x_unscaled_true = x.array() * (precond_vec.array() * precond_vec.array());
  BOOST_CHECK(x_unscaled.isApprox(x_unscaled_true));
}
