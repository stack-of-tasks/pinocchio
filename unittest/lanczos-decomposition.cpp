//
// Copyright (c) 2024 INRIA
//

#include <iostream>

#include <pinocchio/math/lanczos-decomposition.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_identity)
{
  const Eigen::DenseIndex mat_size = 20;
  const Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(mat_size,mat_size);
  
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  LanczosDecomposition lanczos_decomposition(identity_matrix,10);
  
  const auto residual = lanczos_decomposition.computeDecompositionResidual(identity_matrix);
  std::cout << "alphas: " << lanczos_decomposition.Ts().diagonal() << std::endl;
  std::cout << "betas: " << lanczos_decomposition.Ts().subDiagonal() << std::endl;
  BOOST_CHECK(residual.isZero());
}

BOOST_AUTO_TEST_CASE(test_random)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  const Eigen::DenseIndex mat_size = 20;
  
  for(int it = 0; it < 1000; ++it)
  {
    const Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(mat_size,mat_size);
    LanczosDecomposition lanczos_decomposition(matrix,10);
    
    const auto residual = lanczos_decomposition.computeDecompositionResidual(matrix);
    BOOST_CHECK(residual.isZero());
  }
}

BOOST_AUTO_TEST_SUITE_END()


