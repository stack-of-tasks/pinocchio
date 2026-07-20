//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE csv

#include <Eigen/Core>

#include <pinocchio/serialization.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_random_matrix)
{
  const Eigen::Index mat_size = 20;
  const Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(mat_size, mat_size);
  toCSVfile(TEST_SERIALIZATION_FOLDER "/test.csv", matrix);
}
