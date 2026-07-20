//
// Copyright (c) 2019-2025 INRIA
//

#define BOOST_TEST_MODULE serialization

#include <iostream>

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/delassus-operator.hpp"

#include "pinocchio/serialization.hpp"
#include "serialization.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

// template<>
// struct empty_contructor_algo<pinocchio::DelassusOperatorDense>
// {
//   static pinocchio::DelassusOperatorDense * run()
//   {
//     return new pinocchio::DelassusOperatorDense(Eigen::MatrixXd(2, 2));
//   }
// };

template<typename MatrixLike, std::size_t Alignment>
struct empty_contructor_algo<pinocchio::internal::MatrixStackTpl<MatrixLike, Alignment>>
{
  typedef pinocchio::internal::MatrixStackTpl<MatrixLike, Alignment> Self;
  static Self * run()
  {
    return new Self(0);
  }
};

BOOST_AUTO_TEST_CASE(test_static_buffer)
{
  using namespace pinocchio::serialization;
  const size_t size = 10000000;
  StaticBuffer static_buffer(size);
  BOOST_CHECK(size == static_buffer.size());

  const size_t new_size = 2 * size;
  static_buffer.resize(new_size);
  BOOST_CHECK(new_size == static_buffer.size());

  BOOST_CHECK(static_buffer.data() != NULL);
  BOOST_CHECK(reinterpret_cast<const StaticBuffer &>(static_buffer).data() != NULL);
  BOOST_CHECK(reinterpret_cast<const StaticBuffer &>(static_buffer).data() == static_buffer.data());
}

BOOST_AUTO_TEST_CASE(test_eigen_serialization)
{
  using namespace pinocchio;

  const Eigen::Index num_cols = 10;
  const Eigen::Index num_rows = 20;

  const Eigen::Index array_size = 3;

  Eigen::MatrixXd Mat = Eigen::MatrixXd::Random(num_rows, num_cols);
  generic_test(Mat, TEST_SERIALIZATION_FOLDER "/eigen_matrix", "matrix");

  Eigen::VectorXd Vec = Eigen::VectorXd::Random(num_rows * num_cols);
  generic_test(Vec, TEST_SERIALIZATION_FOLDER "/eigen_vector", "vector");

  Eigen::array<Eigen::Index, array_size> array = {1, 2, 3};
  generic_test(array, TEST_SERIALIZATION_FOLDER "/eigen_array", "array");

  Eigen::ArrayXXd Array = Eigen::ArrayXXd::Random(num_rows, num_cols);
  generic_test(Array, TEST_SERIALIZATION_FOLDER "/eigen_array", "array");

  const Eigen::Index tensor_size = 3;
  const Eigen::Index x_dim = 10, y_dim = 20, z_dim = 30;

  typedef pinocchio::Tensor<double, tensor_size> Tensor3x;
  Tensor3x tensor(x_dim, y_dim, z_dim);

  Eigen::Map<Eigen::VectorXd>(tensor.data(), tensor.size(), 1).setRandom();

  generic_test(tensor, TEST_SERIALIZATION_FOLDER "/eigen_tensor", "tensor");
}

BOOST_AUTO_TEST_CASE(test_spatial_serialization)
{
  using namespace pinocchio;

  SE3 M(SE3::Random());
  generic_test(M, TEST_SERIALIZATION_FOLDER "/SE3", "SE3");

  Motion m(Motion::Random());
  generic_test(m, TEST_SERIALIZATION_FOLDER "/Motion", "Motion");

  Force f(Force::Random());
  generic_test(f, TEST_SERIALIZATION_FOLDER "/Force", "Force");

  Symmetric3 S(Symmetric3::Random());
  generic_test(S, TEST_SERIALIZATION_FOLDER "/Symmetric3", "Symmetric3");

  Inertia I(Inertia::Random());
  generic_test(I, TEST_SERIALIZATION_FOLDER "/Inertia", "Inertia");
}
