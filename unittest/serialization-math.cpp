//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE serialization_math

#include "pinocchio/serialization.hpp"

#include "pinocchio/spatial.hpp"

#include <boost/utility/binary.hpp>

#include "serialization.hpp"

#include <boost/test/unit_test.hpp>

template<typename MatrixLike, std::size_t Alignment>
struct empty_contructor_algo<pinocchio::internal::MatrixStackTpl<MatrixLike, Alignment>>
{
  typedef pinocchio::internal::MatrixStackTpl<MatrixLike, Alignment> Self;
  static Self * run()
  {
    return new Self(0);
  }
};

BOOST_AUTO_TEST_CASE(matrix_stack)
{
  typedef pinocchio::internal::MatrixStackTpl<Eigen::MatrixXd> MatrixStack;

  {
    MatrixStack matrix_stack(20);
    matrix_stack.push_back(1, 1);
    matrix_stack.back().fill(2.2);
    matrix_stack.push_back(20, 20);
    matrix_stack.back().setOnes();

    generic_test(matrix_stack, TEST_SERIALIZATION_FOLDER "/Container", "matrix_stack");
  }

  {
    MatrixStack matrix_stack(2, 4); // 2 elements of size 4 each
    matrix_stack.push_back(2, 2);
    matrix_stack.back().setRandom();
    BOOST_CHECK(matrix_stack.sizeInBytes() < matrix_stack.memoryCapacityInBytes());
    // matrix stack contains more allocated data than it represents but this should be fine
    // for serialization

    generic_test(matrix_stack, TEST_SERIALIZATION_FOLDER "/Container", "matrix_stack_full");
  }
}

BOOST_AUTO_TEST_CASE(eigen_storage)
{
  typedef pinocchio::internal::EigenStorageTpl<Eigen::MatrixXd> EigenStorage;

  {
    EigenStorage storage(15, 8, 15, 8);
    storage.map().setRandom();
    storage.resize(10, 5);

    generic_test(storage, TEST_SERIALIZATION_FOLDER "/Container", "eigen_storage");
  }

  {
    EigenStorage storage(10, 10, 10, 10);
    storage.resize(11, 11);
    storage.map().setRandom();

    generic_test(storage, TEST_SERIALIZATION_FOLDER "/Container", "eigen_storage_full");
  }
}

BOOST_AUTO_TEST_CASE(double_entry_container)
{
  typedef pinocchio::Inertia::Matrix6 Matrix6;
  typedef pinocchio::internal::DoubleEntryContainer<std::vector<Matrix6>> DoubleEntryContainer;

  DoubleEntryContainer container(10, 20);
  for (Eigen::Index k = 0; k < 10; ++k)
  {
    container.insert({k, k}, Matrix6::Random());
  }

  generic_test(container, TEST_SERIALIZATION_FOLDER "/Container", "double_entry_container");
}

BOOST_AUTO_TEST_CASE(matrix_block_element)
{
  typedef Eigen::MatrixXd Matrix;
  typedef pinocchio::internal::MatrixBlockElementTpl<Matrix> MatrixBlockElement;

  const Eigen::Index size = 10;

  Matrix diagonal_vector = Matrix::Ones(size, 1);

  MatrixBlockElement matrix_block_elt = {
    pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector};

  BOOST_CHECK(matrix_block_elt.container() == diagonal_vector);

  generic_test(matrix_block_elt, TEST_SERIALIZATION_FOLDER "/Container", "matrix_block_elt");
}

BOOST_AUTO_TEST_CASE(block_diagonal_matrix)
{
  typedef Eigen::MatrixXd Matrix;

  typedef pinocchio::internal::BlockDiagonalMatrix::MatrixBlockElement MatrixBlockElement;
  typedef MatrixBlockElement::MatrixMap MatrixMap;

  const Eigen::Index size = 10;

  Matrix diagonal_vector = Matrix::Ones(size, 1);
  auto diagonal_vector_map = pinocchio::make_map<MatrixMap>(diagonal_vector);

  MatrixBlockElement matrix_block_elt1 = {
    pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector_map};

  MatrixBlockElement matrix_block_elt2 = {pinocchio::internal::MatrixBlockType::Zero, size};

  MatrixBlockElement matrix_block_elt3 = {pinocchio::internal::MatrixBlockType::Identity, size};

  pinocchio::internal::BlockDiagonalMatrix block_diagonal_matrix(
    {matrix_block_elt1, matrix_block_elt2, matrix_block_elt3, matrix_block_elt1});

  generic_test(
    block_diagonal_matrix, TEST_SERIALIZATION_FOLDER "/Container", "block_diagonal_matrix");
}

BOOST_AUTO_TEST_CASE(block_diagonal_matrix_nested)
{
  typedef pinocchio::internal::BlockDiagonalMatrix::MatrixBlockElement MatrixBlockElement;

  const Eigen::Index flat_diag_size = 3;
  const Eigen::Index sub_diag_size = 2;
  const Eigen::Index sub_plain_size = 4;

  // Build a BDM with one flat Diagonal block and one NestedBlockDiagonal block
  // (containing a Diagonal sub-block and a Plain sub-block).
  MatrixBlockElement flat_block(pinocchio::internal::MatrixBlockType::Diagonal, flat_diag_size);

  std::vector<MatrixBlockElement> subs;
  subs.emplace_back(pinocchio::internal::MatrixBlockType::Diagonal, sub_diag_size);
  subs.emplace_back(pinocchio::internal::MatrixBlockType::Plain, sub_plain_size);
  MatrixBlockElement nested_block(
    pinocchio::internal::MatrixBlockType::NestedBlockDiagonal, std::move(subs));

  pinocchio::internal::BlockDiagonalMatrix block_diagonal_matrix({flat_block, nested_block});
  for (auto & block : block_diagonal_matrix.blocks())
    block.setRandom();

  generic_test(
    block_diagonal_matrix, TEST_SERIALIZATION_FOLDER "/Container", "block_diagonal_matrix_nested");
}
