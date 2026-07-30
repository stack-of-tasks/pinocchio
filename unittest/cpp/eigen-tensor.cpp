//
// Copyright (c) 2019 INRIA
//

#define BOOST_TEST_MODULE eigen_tensor

#include "pinocchio/math.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(test_emulate_tensors)
{
  typedef double Scalar;
  const int rank = 3;
  typedef pinocchio::Tensor<Scalar, rank> Tensor;

  const Eigen::Index x_dim = 6, y_dim = 20, z_dim = 20;
  Tensor tensor1(x_dim, y_dim, z_dim), tensor1_bis(x_dim, y_dim, z_dim);

  BOOST_CHECK(tensor1.size() == x_dim * y_dim * z_dim);
  BOOST_CHECK(tensor1.dimension(0) == x_dim);
  BOOST_CHECK(tensor1.dimension(1) == y_dim);
  BOOST_CHECK(tensor1.dimension(2) == z_dim);

  Scalar * data = tensor1.data();
  for (Eigen::Index k = 0; k < tensor1.size(); ++k)
    data[k] = (Scalar)k;

  for (Eigen::Index k = 0; k < z_dim; ++k)
  {
    for (Eigen::Index j = 0; j < y_dim; ++j)
    {
      for (Eigen::Index i = 0; i < x_dim; ++i)
      {
        BOOST_CHECK(tensor1(i, j, k) == (Scalar)(i + j * x_dim + k * (x_dim * y_dim)));
      }
    }
  }

  const Eigen::Index new_x_dim = 2 * x_dim, new_y_dim = 2 * y_dim, new_z_dim = 2 * z_dim;
  const Eigen::array<Tensor::Index, rank> dims = {x_dim, y_dim, z_dim};
  tensor1.resize(dims);

  BOOST_CHECK(tensor1.size() == tensor1_bis.size());
  for (std::size_t i = 0; i < rank; ++i)
    BOOST_CHECK(tensor1.dimension(i) == dims[i]);

  const Eigen::array<Tensor::Index, rank> new_dims = {new_x_dim, new_y_dim, new_z_dim};
  tensor1.resize(new_dims);

  BOOST_CHECK(tensor1.size() == 8 * tensor1_bis.size());
  for (std::size_t i = 0; i < rank; ++i)
    BOOST_CHECK(tensor1.dimension(i) == new_dims[i]);
}
