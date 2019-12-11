//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/math/tensor.hpp"
#include "pinocchio/multibody/model.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <iostream>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_emulate_tensors)
{
  typedef double Scalar;
  const int rank = 3;
  typedef pinocchio::Tensor<Scalar,rank> Tensor;

  const Eigen::DenseIndex x_dim = 6, y_dim = 20, z_dim = 20;
  Tensor tensor1(x_dim,y_dim,z_dim), tensor1_bis(x_dim,y_dim,z_dim);
  
  BOOST_CHECK(tensor1.size() == x_dim * y_dim * z_dim);
  BOOST_CHECK(tensor1.dimension(0) == x_dim);
  BOOST_CHECK(tensor1.dimension(1) == y_dim);
  BOOST_CHECK(tensor1.dimension(2) == z_dim);
  
  Scalar * data = tensor1.data();
  for(Eigen::DenseIndex k = 0; k < tensor1.size(); ++k)
    data[k] = (Scalar)k;
  
  for(Eigen::DenseIndex k = 0; k < z_dim; ++k)
  {
    for(Eigen::DenseIndex j = 0; j < y_dim; ++j)
    {
      for(Eigen::DenseIndex i = 0; i < x_dim; ++i)
      {
        BOOST_CHECK(tensor1(i,j,k) == (Scalar)(i + j*x_dim + k*(x_dim*y_dim)));
      }
    }
  }
  
  const Eigen::DenseIndex new_x_dim = 2*x_dim, new_y_dim = 2*y_dim, new_z_dim = 2*z_dim;
  const Eigen::array<Tensor::Index,rank> dims = { x_dim,y_dim,z_dim };
  tensor1.resize(dims);
  
  BOOST_CHECK(tensor1.size() == tensor1_bis.size());
  for(std::size_t i = 0; i < rank; ++i)
    BOOST_CHECK(tensor1.dimension(i) == dims[i]);
  
  const Eigen::array<Tensor::Index,rank> new_dims = { new_x_dim,new_y_dim,new_z_dim };
  tensor1.resize(new_dims);
  
  BOOST_CHECK(tensor1.size() == 8*tensor1_bis.size());
  for(std::size_t i = 0; i < rank; ++i)
    BOOST_CHECK(tensor1.dimension(i) == new_dims[i]);
}

BOOST_AUTO_TEST_SUITE_END()
