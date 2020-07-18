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
  BOOST_CHECK(!res.eval().isZero());
}

BOOST_AUTO_TEST_CASE(test_scalar_matrix_product)
{
  using namespace pinocchio;
  using namespace Eigen;
  const Eigen::DenseIndex m = 20, n = 100;
  MatrixXd M(MatrixXd::Ones(m,n));
  const double alpha = 0.;
  ScalarMatrixProduct<double,MatrixXd>::type res = alpha * M;
  BOOST_CHECK(res.eval().isZero());
}

BOOST_AUTO_TEST_CASE(test_matrix_scalar_product)
{
  using namespace pinocchio;
  using namespace Eigen;
  const Eigen::DenseIndex m = 20, n = 100;
  MatrixXd M(MatrixXd::Ones(m,n));
  const double alpha = 1.;
  MatrixScalarProduct<MatrixXd,double>::type res = M * alpha;
  BOOST_CHECK(res.eval() == M);
}

BOOST_AUTO_TEST_SUITE_END()

