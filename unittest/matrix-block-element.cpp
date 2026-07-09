//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE matrix_block_element

#include <limits>
#include <pinocchio/math.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
typedef Eigen::Matrix<double, 1, 1> M11;
typedef internal::BlockDiagonalMatrix::Matrix Matrix;
typedef internal::BlockDiagonalMatrix::MatrixMap MatrixMap;
typedef internal::BlockDiagonalMatrix::ConstMatrixMap ConstMatrixMap;
typedef internal::BlockDiagonalMatrix::Vector Vector;
typedef internal::MatrixBlockElementTpl<Matrix> MatrixBlockElement;
typedef internal::MatrixBlockElementTpl<MatrixMap> MatrixMapBlockElement;
typedef internal::MatrixBlockElementTpl<const Matrix> ConstMatrixBlockElement;
typedef internal::MatrixBlockElementTpl<ConstMatrixMap> ConstMatrixMapBlockElement;

template<typename _MatrixBlockElement>
void test_assignment(
  const internal::MatrixBlockElementPlain<_MatrixBlockElement> & matrix_block_element)
{
  const auto size = matrix_block_element.size();
  const Matrix square_matrix = Matrix::Random(size, size);

  const auto matrix_block_element_plain = matrix_block_element.matrix();

  const int num_tests =
#ifdef NDEBUG
    100000
#else
    1000
#endif
    ;

  // evalTo
  for (int k = 0; k < num_tests; ++k)
  {
    Matrix res = Matrix::Random(size, size);
    matrix_block_element.evalTo(res);

    BOOST_CHECK(res.isApprox(matrix_block_element_plain));
  }

  // addTo
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix res_ref = square_matrix + matrix_block_element_plain;
    Matrix res(square_matrix);
    matrix_block_element.addTo(res);

    BOOST_CHECK(res.isApprox(res_ref));
  }

  // subTo
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix res_ref = square_matrix - matrix_block_element_plain;
    Matrix res(square_matrix);
    matrix_block_element.subTo(res);

    BOOST_CHECK(res.isApprox(res_ref));
  }
}

template<typename _MatrixBlockElement>
void test_inverse(
  const internal::MatrixBlockElementPlain<_MatrixBlockElement> & matrix_block_element)
{
  const auto size = matrix_block_element.size();
  const auto matrix_block_element_plain = matrix_block_element.matrix();
  const auto matrix_block_element_inverse = matrix_block_element.inverse();
  const auto matrix_block_element_inverse_plain = matrix_block_element_inverse.matrix();

  if (matrix_block_element.type() == pinocchio::internal::MatrixBlockType::Zero)
  {
    BOOST_CHECK(
      matrix_block_element_inverse_plain
      == Eigen::MatrixXd::Constant(size, size, std::numeric_limits<double>::infinity()));
  }
  else
  {
    BOOST_CHECK(matrix_block_element_plain.inverse().isApprox(matrix_block_element_inverse_plain));
  }

  auto matrix_block_element_copy = matrix_block_element.derived();

  const int num_tests =
#ifdef NDEBUG
    100000
#else
    1000
#endif
    ;

  for (int k = 0; k < num_tests; ++k)
  {
    matrix_block_element_copy.setRandom();
    const auto matrix_block_element_copy_inverse = matrix_block_element_copy.inverse();

    if (matrix_block_element_copy.type() == pinocchio::internal::MatrixBlockType::Zero)
    {
      BOOST_CHECK(
        matrix_block_element_copy_inverse.matrix()
        == Eigen::MatrixXd::Constant(size, size, std::numeric_limits<double>::infinity()));
    }
    else
    {
      BOOST_CHECK(matrix_block_element_copy.matrix().inverse().isApprox(
        matrix_block_element_copy_inverse.matrix()));
    }
  }
}

template<typename _MatrixBlockElement>
void test_operations(
  const internal::MatrixBlockElementPlain<_MatrixBlockElement> & _matrix_block_element)
{
  const auto & matrix_block_element = _matrix_block_element.derived();

  const auto size = matrix_block_element.size();
  const Vector diagonal_vector = Vector::Random(size);

  const auto matrix_block_element_plain = matrix_block_element.matrix();

  const int num_tests =
#ifdef NDEBUG
    100000
#else
    1000
#endif
    ;

  // operator+
  for (int k = 0; k < num_tests; ++k)
  {
    auto block_sum = matrix_block_element + diagonal_vector.asDiagonal();

    MatrixBlockElement res_block = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector};
    res_block = block_sum;

    const auto res_eigen =
      (matrix_block_element.matrix() + Matrix(diagonal_vector.asDiagonal())).eval();

    BOOST_CHECK(Matrix(res_block.container().asDiagonal()).isApprox(res_eigen));
  }

  // operator-
  for (int k = 0; k < num_tests; ++k)
  {
    auto block_sum = matrix_block_element - diagonal_vector.asDiagonal();

    MatrixBlockElement res_block = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector};
    res_block = block_sum;

    const auto res_eigen =
      (matrix_block_element.matrix() - Matrix(diagonal_vector.asDiagonal())).eval();

    BOOST_CHECK(Matrix(res_block.container().asDiagonal()).isApprox(res_eigen));
  }
}

// void test_applyOnTheRight(const BlockDiagonalMatrix & block_diagonal_matrix)
// {
//   const auto rows = block_diagonal_matrix.rows();
//   const auto cols = 20;

//   const auto bdm_plain = block_diagonal_matrix.matrix();

//   const int num_tests =
// #ifdef NDEBUG
//     100000
// #else
//     1000
// #endif
//     ;

//   // assign
//   for (int k = 0; k < num_tests; ++k)
//   {
//     const Matrix rhs_matrix = Matrix::Random(rows, cols);
//     const auto res_ref = (bdm_plain * rhs_matrix).eval();
//     const auto res = block_diagonal_matrix * rhs_matrix;
//     BOOST_CHECK(res.isApprox(res_ref));
//   }

//   // add_assign
//   for (int k = 0; k < num_tests; ++k)
//   {
//     const Matrix rhs_matrix = Matrix::Random(rows, cols);
//     const Matrix res0 = Matrix::Random(rows, cols);
//     const auto res_ref = (res0 + bdm_plain * rhs_matrix).eval();
//     Matrix res = res0;
//     block_diagonal_matrix.applyOnTheRight<pinocchio::internal::add_assign_op>(rhs_matrix, res);
//     BOOST_CHECK(res.isApprox(res_ref));
//   }

//   // sub_assign
//   for (int k = 0; k < num_tests; ++k)
//   {
//     const Matrix rhs_matrix = Matrix::Random(rows, cols);
//     const Matrix res0 = Matrix::Random(rows, cols);
//     const auto res_ref = (res0 - bdm_plain * rhs_matrix).eval();
//     Matrix res = res0;
//     block_diagonal_matrix.applyOnTheRight<pinocchio::internal::sub_assign_op>(rhs_matrix, res);
//     BOOST_CHECK(res.isApprox(res_ref));
//   }
// }

BOOST_AUTO_TEST_CASE(test_matrix_block_elements_eigen_map)
{
  typedef internal::MatrixBlockElementTpl<MatrixMap> MatrixBlockElement;
  typedef internal::MatrixBlockElementTpl<ConstMatrixMap> ConstMatrixBlockElement;
  const Eigen::Index size = 10;

  // Test non const version
  {
    Matrix diagonal_vector = Matrix::Ones(size, 1);

    const auto matrix_map = make_map<MatrixMap>(diagonal_vector);
    MatrixBlockElement lhs_block = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, matrix_map};

    BOOST_CHECK(lhs_block.map == diagonal_vector);
    BOOST_CHECK(lhs_block.map.data() == diagonal_vector.data());

    test_assignment(lhs_block);
    test_operations(lhs_block);
  }

  // Test const version
  {
    Matrix diagonal_vector = Matrix::Ones(size, 1);

    const auto matrix_map = make_map<ConstMatrixMap>(diagonal_vector);
    ConstMatrixBlockElement lhs_block = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, matrix_map};

    BOOST_CHECK(lhs_block.map == diagonal_vector);
    BOOST_CHECK(lhs_block.map.data() == diagonal_vector.data());

    test_assignment(lhs_block);
  }
}

BOOST_AUTO_TEST_CASE(test_matrix_block_elements_eigen_matrix)
{
  const Eigen::Index size = 10;
  typedef internal::MatrixBlockElementTpl<Matrix> MatrixBlockElement;
  typedef internal::MatrixBlockElementTpl<const Matrix> ConstMatrixBlockElement;

  // Test non const version
  {
    Matrix diagonal_vector = Matrix::Ones(size, 1);

    MatrixBlockElement lhs_block = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector};

    BOOST_CHECK(lhs_block.container() == diagonal_vector);

    test_assignment(lhs_block);
    test_operations(lhs_block);
  }

  // Test const version
  {
    const Matrix diagonal_vector = Matrix::Ones(size, 1);

    ConstMatrixBlockElement lhs_block = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector};

    BOOST_CHECK(lhs_block.container() == diagonal_vector);

    test_assignment(lhs_block);
  }
}

BOOST_AUTO_TEST_CASE(test_inverse_method)
{
  const Eigen::Index size = 10;
  typedef internal::MatrixBlockElementTpl<Matrix> MatrixBlockElement;

  // Zero block
  {
    MatrixBlockElement matrix_block_element = {pinocchio::internal::MatrixBlockType::Zero, size};

    test_inverse(matrix_block_element);
  }

  // Identity block
  {
    MatrixBlockElement matrix_block_element = {
      pinocchio::internal::MatrixBlockType::Identity, size};

    test_inverse(matrix_block_element);
  }

  // Scalar Identity block
  {
    const double scale_value = 1.;
    M11 scale_mat = M11(scale_value);
    MatrixBlockElement matrix_block_element = {
      pinocchio::internal::MatrixBlockType::ScalarIdentity, size, scale_mat};

    test_inverse(matrix_block_element);
  }

  // Diagonal
  {
    Matrix diagonal_vector = Matrix::Ones(size, 1);
    MatrixBlockElement matrix_block_element = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, diagonal_vector};

    test_inverse(matrix_block_element);
  }

  // Plain
  {
    Matrix plain_matrix = Matrix::Identity(size, size);
    MatrixBlockElement matrix_block_element = {
      pinocchio::internal::MatrixBlockType::Plain, size, plain_matrix};

    test_inverse(matrix_block_element);
  }
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_block_element)
{
  const std::size_t num_tests =
#ifdef NDEBUG
    1000
#else
    10
#endif
    ;

  for (std::size_t i = 0; i < num_tests; ++i)
  {
    // Build a NestedBlockDiagonal block containing a 3×3 Diagonal and a 4×4 Plain sub-block.
    // The BlockDiagonalMatrix manages backing memory; we test through its block accessor.
    const Eigen::Index diag_size = 3;
    const Eigen::Index plain_size = 4;
    const Eigen::Index total_size = diag_size + plain_size;

    typedef internal::BlockDiagonalMatrix::MatrixBlockElement BDMBlockElement;
    std::vector<BDMBlockElement> nested_subs;
    nested_subs.emplace_back(pinocchio::internal::MatrixBlockType::Diagonal, diag_size);
    nested_subs.emplace_back(pinocchio::internal::MatrixBlockType::Plain, plain_size);
    BDMBlockElement nested_block(
      pinocchio::internal::MatrixBlockType::NestedBlockDiagonal, std::move(nested_subs));

    internal::BlockDiagonalMatrix bdm({nested_block});

    BOOST_CHECK(bdm.rows() == total_size);
    BOOST_CHECK(bdm.cols() == total_size);
    BOOST_CHECK(bdm.blocks().size() == 1);
    BOOST_CHECK(
      bdm.blocks()[0].type() == pinocchio::internal::MatrixBlockType::NestedBlockDiagonal);
    BOOST_CHECK(bdm.blocks()[0].nested_blocks().size() == 2);

    // Populate sub-blocks with random data (make it invertible for inverse test)
    for (auto & block : bdm.blocks())
      block.setRandomPD();

    // Retrieve the block and verify evalTo / addTo / subTo
    const auto & block = bdm.blocks()[0];
    const Matrix expected_plain = bdm.matrix(); // full BDM as plain matrix

    // evalTo
    {
      Matrix res = Matrix::Random(total_size, total_size);
      block.evalTo(res);
      BOOST_CHECK(res.isApprox(expected_plain));
    }

    // addTo
    {
      const Matrix base = Matrix::Random(total_size, total_size);
      Matrix res = base;
      block.addTo(res);
      BOOST_CHECK(res.isApprox(base + expected_plain));
    }

    // subTo
    {
      const Matrix base = Matrix::Random(total_size, total_size);
      Matrix res = base;
      block.subTo(res);
      BOOST_CHECK(res.isApprox(base - expected_plain));
    }

    // diagonal()
    {
      Vector diag_ref = expected_plain.diagonal();
      Vector diag_result = block.diagonal();
      BOOST_CHECK(diag_result.isApprox(diag_ref));
    }

    // in-place inverse: create a result BDM with same pattern and invert each sub-block
    {

      std::vector<BDMBlockElement> res_nested_subs;
      res_nested_subs.emplace_back(pinocchio::internal::MatrixBlockType::Diagonal, diag_size);
      res_nested_subs.emplace_back(pinocchio::internal::MatrixBlockType::Plain, plain_size);
      BDMBlockElement res_nested_block(
        pinocchio::internal::MatrixBlockType::NestedBlockDiagonal, std::move(res_nested_subs));
      pinocchio::internal::BlockDiagonalMatrix res_bdm({res_nested_block});

      block.inverse(res_bdm.blocks()[0]);

      const Matrix inv_result = res_bdm.matrix();
      // Verify: block * inv = I
      BOOST_CHECK((expected_plain * inv_result).isApprox(Matrix::Identity(total_size, total_size)));
    }
  }
}
