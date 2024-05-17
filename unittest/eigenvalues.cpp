//
// Copyright (c) 2022 INRIA
//

#include <iostream>

#include <pinocchio/math/eigenvalues.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_identity)
{
  const Eigen::DenseIndex mat_size = 20;
  const Eigen::MatrixXd identity_mat = Eigen::MatrixXd::Identity(mat_size, mat_size);

  auto eigen_vec = computeLargestEigenvector(identity_mat);
  auto eigen_val = retrieveLargestEigenvalue(eigen_vec);
  BOOST_CHECK(std::fabs(eigen_val - 1.) <= 1e-4);
}

BOOST_AUTO_TEST_CASE(test_random_matrix)
{
  const Eigen::DenseIndex mat_size = 10;
  const int num_tests = 1000;
  const int num_it_max = 10;
  const int num_it_max_finer = 100 * num_it_max;

  for (int k = 0; k < num_tests; ++k)
  {
    const Eigen::MatrixXd A = Eigen::MatrixXd::Random(mat_size, mat_size);
    const Eigen::MatrixXd sym_mat = (A * A.transpose());

    const Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(sym_mat, true);
    BOOST_CHECK(eigen_solver.info() == Eigen::Success);

    // Sort eigenvalues
    Eigen::VectorXd sorted_eigen_values = eigen_solver.eigenvalues().real();
    std::sort(sorted_eigen_values.data(), sorted_eigen_values.data() + sorted_eigen_values.size());

    const double eigen_val_ref = sorted_eigen_values[sorted_eigen_values.size() - 1];

    auto eigen_vec = computeLargestEigenvector(sym_mat, num_it_max);
    auto eigen_val = retrieveLargestEigenvalue(eigen_vec);

    bool test_relative_eigen_val =
      std::fabs(eigen_val - eigen_val_ref) / (std::max)(eigen_val, eigen_val_ref) <= 1e-2;
    bool test_eigen_vec = (sym_mat * eigen_vec).isApprox(eigen_val * eigen_vec, 1e-2);

    if (!test_relative_eigen_val || !test_eigen_vec)
    {
      auto eigen_vec_finer = computeLargestEigenvector(sym_mat, num_it_max_finer);
      auto eigen_val_finer = retrieveLargestEigenvalue(eigen_vec_finer);

      test_relative_eigen_val =
        std::fabs(eigen_val_finer - eigen_val_ref) / (std::max)(eigen_val_finer, eigen_val_ref)
        <= 1e-2;
      test_eigen_vec =
        (sym_mat * eigen_vec_finer).isApprox(eigen_val_finer * eigen_vec_finer, 1e-2);
      std::cout << "res: " << (sym_mat * eigen_vec - eigen_val * eigen_vec).norm() << std::endl;
      std::cout << "eigen_val: " << eigen_val << std::endl;
      std::cout << "eigen_val_finer: " << eigen_val_finer << std::endl;
      std::cout << "eigen_val_ref: " << eigen_val_ref << std::endl;
      std::cout << "res finer: "
                << (sym_mat * eigen_vec_finer - eigen_val_finer * eigen_vec_finer).norm()
                << std::endl;
    }
    BOOST_CHECK(test_relative_eigen_val);
    BOOST_CHECK(test_eigen_vec);
  }
}

BOOST_AUTO_TEST_SUITE_END()
