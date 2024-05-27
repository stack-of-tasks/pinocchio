//
// Copyright (c) 2024 INRIA
//

#include <iostream>

#include <pinocchio/math/gram-schmidt-orthonormalisation.hpp>
#include <Eigen/QR>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_random_matrix)
{
  for (size_t i = 0; i < 100; ++i)
  {
    const Eigen::DenseIndex size = 20;
    const Eigen::MatrixXd random_mat = Eigen::MatrixXd::Random(size, size);
    const auto qr = random_mat.householderQr();
    const Eigen::MatrixXd basis = qr.householderQ();

    for (size_t k = 0; k < 1000; ++k)
    {
      const Eigen::VectorXd random_vec = Eigen::VectorXd::Random(size);
      orthonormalisation(basis.leftCols(10), random_vec);
      BOOST_CHECK((basis.leftCols(10).transpose() * random_vec).isZero());
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
