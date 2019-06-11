//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/multibody/model.hpp"

#include <Eigen/Core>
#include "pinocchio/math/matrix.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_matrix_matrix_product)
{
  using namespace pinocchio;
  using namespace Eigen;
  const Eigen::DenseIndex m = 20, n = 100;
  MatrixXd M1(MatrixXd::Ones(m,n)), M2(MatrixXd::Ones(n,m));
  MatrixMatrixProduct<MatrixXd,MatrixXd>::type res = M1 * M2;
  BOOST_CHECK(not res.eval().isZero());
}

BOOST_AUTO_TEST_SUITE_END()

