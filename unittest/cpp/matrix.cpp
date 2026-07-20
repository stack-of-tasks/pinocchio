//
// Copyright (c) 2024-2026 INRIA
//

#define BOOST_TEST_MODULE matrix

#include <pinocchio/math.hpp>

#include <Eigen/Core>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_isSymmetric)
{
  srand(0);

  typedef Eigen::MatrixXd Matrix;

#ifdef NDEBUG
  const int max_test = 1e3;
  const int max_size = 200;
#else
  const int max_test = 1e2;
  const int max_size = 100;
#endif
  for (int i = 0; i < max_test; ++i)
  {
    const Eigen::Index rows = rand() % max_size + 3; // random row number
    const Eigen::Index cols = rand() % max_size + 3; // random col number

    const Matrix random_matrix = Matrix::Random(rows, cols);
    Matrix symmetric_matrix = random_matrix.transpose() * random_matrix;
    BOOST_CHECK(isSymmetric(symmetric_matrix));

    symmetric_matrix.coeffRef(1, 0) += 2.;
    BOOST_CHECK(!isSymmetric(symmetric_matrix));

    // Specific check for the Zero matrix
    BOOST_CHECK(isSymmetric(Matrix::Zero(rows, rows)));
    // Specific check for the Identity matrix
    BOOST_CHECK(isSymmetric(Matrix::Identity(rows, rows)));
  }
}

BOOST_AUTO_TEST_CASE(test_enforceSymmetry)
{
  srand(0);

  typedef Eigen::MatrixXd Matrix;

#ifdef NDEBUG
  const int max_test = 1e3;
  const int max_size = 200;
#else
  const int max_test = 1e2;
  const int max_size = 100;
#endif
  for (int i = 0; i < max_test; ++i)
  {
    const Eigen::Index rows = rand() % max_size + 3; // random row number

    Matrix random_matrix = Matrix::Random(rows, rows);
    BOOST_CHECK(!isSymmetric(random_matrix));
    enforceSymmetry(random_matrix);
    BOOST_CHECK(isSymmetric(random_matrix, 0));
  }
}

BOOST_AUTO_TEST_CASE(test_nullptr_in_map)
{
  typedef Eigen::MatrixXd Matrix;
  Eigen::Index rows = 10, cols = 20;

  Eigen::Map<Matrix> map = {nullptr, rows, cols};
  BOOST_CHECK(map.data() == nullptr);

  // will crash with error "unknown location:0: fatal error: in "Test/test_nullptr_in_map": memory
  // access violation at address: 0x0: invalid permissions" Matrix copy = map;
}

BOOST_AUTO_TEST_CASE(test_remap)
{
  srand(0);

  typedef Eigen::MatrixXd Matrix;
  typedef Eigen::Matrix3d Matrix3;

#ifdef NDEBUG
  const int max_test = 1e3;
#else
  const int max_test = 1e2;
#endif
  for (int i = 0; i < max_test; ++i)
  {
    Matrix random_matrix = Matrix3::Random();
    auto input_map = make_default_map<Matrix>(random_matrix);
    BOOST_CHECK(input_map == random_matrix);
    auto re_mapped = remap<Eigen::Map<Matrix3>>(input_map);
    BOOST_CHECK(re_mapped == random_matrix);
    BOOST_CHECK(re_mapped == input_map);
  }
}
