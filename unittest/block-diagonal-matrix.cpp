//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE block_diagonal_matrix

#include <pinocchio/math.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
typedef Eigen::Matrix<double, 1, 1> M11;
typedef internal::BlockDiagonalMatrix::Matrix Matrix;
typedef internal::BlockDiagonalMatrix::MatrixMap MatrixMap;
typedef internal::BlockDiagonalMatrix::ConstMatrixMap ConstMatrixMap;
typedef internal::BlockDiagonalMatrix::Vector Vector;
typedef internal::BlockDiagonalMatrix::MatrixBlockElement MatrixBlockElement;
typedef internal::BlockDiagonalMatrix::ConstMatrixBlockElement ConstMatrixBlockElement;

void test_assignment(const internal::BlockDiagonalMatrix & block_diagonal_matrix)
{
  const auto size = block_diagonal_matrix.rows();
  const Matrix square_matrix = Matrix::Random(size, size);

  const auto bdm_plain = block_diagonal_matrix.matrix();

  // evalTo
  {
    Matrix res = Matrix::Random(size, size);
    block_diagonal_matrix.evalTo(res);

    BOOST_CHECK(res.isApprox(bdm_plain));
  }

  // addTo
  {
    const Matrix res_ref = square_matrix + bdm_plain;
    Matrix res(square_matrix);
    block_diagonal_matrix.addTo(res);

    BOOST_CHECK(res.isApprox(res_ref));
  }

  // subTo
  {
    const Matrix res_ref = square_matrix - bdm_plain;
    Matrix res(square_matrix);
    block_diagonal_matrix.subTo(res);

    BOOST_CHECK(res.isApprox(res_ref));
  }
}

void test_applyOnTheRight(const internal::BlockDiagonalMatrix & block_diagonal_matrix)
{
  const auto rows = block_diagonal_matrix.rows();
  const auto cols = 20;

  const auto bdm_plain = block_diagonal_matrix.matrix();

  const int num_tests =
#ifdef NDEBUG
    100000
#else
    1000
#endif
    ;

  // assign
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix rhs_matrix = Matrix::Random(rows, cols);
    const auto res_ref = (bdm_plain * rhs_matrix).eval();
    const auto res = block_diagonal_matrix * rhs_matrix;
    BOOST_CHECK(res.isApprox(res_ref));
  }

  // add_assign
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix rhs_matrix = Matrix::Random(rows, cols);
    const Matrix res0 = Matrix::Random(rows, cols);
    const auto res_ref = (res0 + bdm_plain * rhs_matrix).eval();
    Matrix res = res0;
    block_diagonal_matrix.applyOnTheRight<pinocchio::internal::add_assign_op>(rhs_matrix, res);
    BOOST_CHECK(res.isApprox(res_ref));
  }

  // sub_assign
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix rhs_matrix = Matrix::Random(rows, cols);
    const Matrix res0 = Matrix::Random(rows, cols);
    const auto res_ref = (res0 - bdm_plain * rhs_matrix).eval();
    Matrix res = res0;
    block_diagonal_matrix.applyOnTheRight<pinocchio::internal::sub_assign_op>(rhs_matrix, res);
    BOOST_CHECK(res.isApprox(res_ref));
  }
}

void test_applyOnTheLeft(const internal::BlockDiagonalMatrix & block_diagonal_matrix)
{
  const auto cols = block_diagonal_matrix.cols();
  const auto rows = 20;

  const auto bdm_plain = block_diagonal_matrix.matrix();

  const int num_tests =
#ifdef NDEBUG
    100000
#else
    1000
#endif
    ;

  // assign
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix lhs_matrix = Matrix::Random(rows, cols);
    const auto res_ref = (lhs_matrix * bdm_plain).eval();
    // const auto res = lhs_matrix * block_diagonal_matrix;
    Matrix res = Matrix::Random(rows, cols);
    block_diagonal_matrix.applyOnTheLeft<pinocchio::internal::assign_op>(lhs_matrix, res);
    BOOST_CHECK(res.isApprox(res_ref));
  }

  // add_assign
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix lhs_matrix = Matrix::Random(rows, cols);
    const Matrix res0 = Matrix::Random(rows, cols);
    const auto res_ref = (res0 + lhs_matrix * bdm_plain).eval();
    Matrix res = res0;
    block_diagonal_matrix.applyOnTheLeft<pinocchio::internal::add_assign_op>(lhs_matrix, res);
    BOOST_CHECK(res.isApprox(res_ref));
  }

  // sub_assign
  for (int k = 0; k < num_tests; ++k)
  {
    const Matrix lhs_matrix = Matrix::Random(rows, cols);
    const Matrix res0 = Matrix::Random(rows, cols);
    const auto res_ref = (res0 - lhs_matrix * bdm_plain).eval();
    Matrix res = res0;
    block_diagonal_matrix.applyOnTheLeft<pinocchio::internal::sub_assign_op>(lhs_matrix, res);
    BOOST_CHECK(res.isApprox(res_ref));
  }
}

BOOST_AUTO_TEST_CASE(test_default_constructor)
{
  internal::BlockDiagonalMatrix matrix;
  BOOST_CHECK(matrix.data() == nullptr);
  BOOST_CHECK(matrix.rows() == -1);
  BOOST_CHECK(matrix.cols() == -1);
}

BOOST_AUTO_TEST_CASE(test_single_block)
{
  const Eigen::Index size = 10;

  // Zero block
  {
    MatrixBlockElement single_block_info = {pinocchio::internal::MatrixBlockType::Zero, size};
    BOOST_CHECK(single_block_info.isValid());

    internal::BlockDiagonalMatrix block_diagonal_matrix({single_block_info});

    BOOST_CHECK(block_diagonal_matrix.rows() == size);
    BOOST_CHECK(block_diagonal_matrix.cols() == size);

    BOOST_CHECK(block_diagonal_matrix.getMatrixStack().size() == 0);

    const auto plain_matrix = block_diagonal_matrix.matrix();
    BOOST_CHECK(plain_matrix.isZero(0));

    test_assignment(block_diagonal_matrix);
    test_applyOnTheRight(block_diagonal_matrix);
    test_applyOnTheLeft(block_diagonal_matrix);
  }

  // Identity block
  {
    MatrixBlockElement single_block_info = {pinocchio::internal::MatrixBlockType::Identity, size};
    BOOST_CHECK(single_block_info.isValid());

    internal::BlockDiagonalMatrix block_diagonal_matrix({single_block_info});

    BOOST_CHECK(block_diagonal_matrix.rows() == size);
    BOOST_CHECK(block_diagonal_matrix.cols() == size);

    BOOST_CHECK(block_diagonal_matrix.getMatrixStack().size() == 0);

    const auto plain_matrix = block_diagonal_matrix.matrix();
    BOOST_CHECK(plain_matrix.isIdentity(0));

    test_assignment(block_diagonal_matrix);
    test_applyOnTheRight(block_diagonal_matrix);
    test_applyOnTheLeft(block_diagonal_matrix);
  }

  // Scalar Identity block
  {
    const double scale = 1.;
    M11 scale_mat = M11(scale);
    const auto matrix_map = make_map<MatrixMap>(scale_mat);
    MatrixBlockElement single_block_info = {
      pinocchio::internal::MatrixBlockType::ScalarIdentity, size, matrix_map};
    BOOST_CHECK(single_block_info.isValid());

    internal::BlockDiagonalMatrix block_diagonal_matrix({single_block_info});

    BOOST_CHECK(block_diagonal_matrix.rows() == size);
    BOOST_CHECK(block_diagonal_matrix.cols() == size);

    BOOST_CHECK(block_diagonal_matrix.getMatrixStack().size() == 1);

    {
      const auto plain_matrix = block_diagonal_matrix.matrix();
      BOOST_CHECK(plain_matrix.isIdentity(0));
    }

    {
      block_diagonal_matrix.getMatrixStack().back()(0, 0) = 0.;
      const auto plain_matrix = block_diagonal_matrix.matrix();
      BOOST_CHECK(plain_matrix.isZero(0));
    }

    test_assignment(block_diagonal_matrix);
    test_applyOnTheRight(block_diagonal_matrix);
    test_applyOnTheLeft(block_diagonal_matrix);
  }

  // Diagonal block
  {
    Matrix diagonal_vector = Matrix::Ones(size, 1);

    const auto matrix_map = make_map<MatrixMap>(diagonal_vector);
    MatrixBlockElement single_block_info = {
      pinocchio::internal::MatrixBlockType::Diagonal, size, matrix_map};
    BOOST_CHECK(single_block_info.isValid());

    internal::BlockDiagonalMatrix block_diagonal_matrix({single_block_info});

    BOOST_CHECK(block_diagonal_matrix.rows() == size);
    BOOST_CHECK(block_diagonal_matrix.cols() == size);

    BOOST_CHECK(block_diagonal_matrix.getMatrixStack().size() == 1);

    const auto plain_matrix = block_diagonal_matrix.matrix();
    BOOST_CHECK(plain_matrix.isIdentity(0));

    test_assignment(block_diagonal_matrix);
    test_applyOnTheRight(block_diagonal_matrix);
    test_applyOnTheLeft(block_diagonal_matrix);
  }

  // Plain block
  {
    Matrix diagonal_plain = Matrix::Identity(size, size);

    const auto matrix_map = make_map<MatrixMap>(diagonal_plain);
    MatrixBlockElement single_block_info = {
      pinocchio::internal::MatrixBlockType::Plain, size, matrix_map};
    BOOST_CHECK(single_block_info.isValid());

    internal::BlockDiagonalMatrix block_diagonal_matrix({single_block_info});

    BOOST_CHECK(block_diagonal_matrix.rows() == size);
    BOOST_CHECK(block_diagonal_matrix.cols() == size);

    BOOST_CHECK(block_diagonal_matrix.getMatrixStack().size() == 1);

    const auto plain_matrix = block_diagonal_matrix.matrix();
    BOOST_CHECK(plain_matrix.isIdentity(0));

    test_assignment(block_diagonal_matrix);
    test_applyOnTheRight(block_diagonal_matrix);
    test_applyOnTheLeft(block_diagonal_matrix);
  }
}

BOOST_AUTO_TEST_CASE(test_Zero_constructor)
{
  const Eigen::Index size = 20;
  const auto block_diagonal_matrix = internal::BlockDiagonalMatrix::Zero(size);

  const auto bdm_plain = block_diagonal_matrix.matrix();
  BOOST_CHECK(bdm_plain == Matrix::Zero(size, size));
}

BOOST_AUTO_TEST_CASE(test_ScalarIdentity_constructor)
{
  const Eigen::Index size = 20;
  const double scale = 2;
  const auto block_diagonal_matrix = internal::BlockDiagonalMatrix::ScalarIdentity(size, scale);

  const auto bdm_plain = block_diagonal_matrix.matrix();
  BOOST_CHECK(bdm_plain == Matrix(Vector::Constant(size, scale).asDiagonal()));
}

BOOST_AUTO_TEST_CASE(test_construct_from_diagonal_matrix)
{
  const Eigen::Index size = 20;

  {
    const Vector diagonal_terms = Vector::Random(size);

    const internal::BlockDiagonalMatrix block_diagonal_matrix(diagonal_terms.asDiagonal());
    BOOST_CHECK(block_diagonal_matrix.getMatrixBlockElements().size() == 1);
    BOOST_CHECK(
      block_diagonal_matrix.getMatrixBlockElements().back().container() == diagonal_terms);

    const auto bdm_plain = block_diagonal_matrix.matrix();

    BOOST_CHECK(bdm_plain == Matrix(diagonal_terms.asDiagonal()));
  }

  {
    const auto diagonal_terms = Vector::Constant(size, 2.);

    const internal::BlockDiagonalMatrix block_diagonal_matrix(diagonal_terms.asDiagonal());
    BOOST_CHECK(block_diagonal_matrix.getMatrixBlockElements().size() == 1);
    BOOST_CHECK(
      block_diagonal_matrix.getMatrixBlockElements().back().container() == diagonal_terms);

    const auto bdm_plain = block_diagonal_matrix.matrix();

    BOOST_CHECK(bdm_plain == Matrix(diagonal_terms.asDiagonal()));
  }
}

BOOST_AUTO_TEST_CASE(test_size_in_bytes)
{
  const Eigen::Index size = 20;
  const auto block_diagonal_matrix = internal::BlockDiagonalMatrix::Zero(size);

  BOOST_CHECK(block_diagonal_matrix.sizeInBytes() >= 2 * sizeof(Eigen::Index));
}

BOOST_AUTO_TEST_CASE(test_copy_diagonal_matrix)
{
  const Eigen::Index size = 20;
  const Vector diagonal_terms = Vector::Random(size);

  internal::BlockDiagonalMatrix block_diagonal_matrix;
  block_diagonal_matrix = diagonal_terms.asDiagonal();
  const auto bdm_plain = block_diagonal_matrix.matrix();

  BOOST_CHECK(bdm_plain == Matrix(diagonal_terms.asDiagonal()));
}

internal::BlockDiagonalMatrix create_multiple_block_info(const Eigen::Index block_size)
{
  std::vector<MatrixBlockElement> matrix_block_elements_vector;

  {
    MatrixBlockElement single_block_info = {pinocchio::internal::MatrixBlockType::Zero, block_size};
    matrix_block_elements_vector.push_back(single_block_info);
    BOOST_CHECK(matrix_block_elements_vector.back().isValid());
  }

  {
    MatrixBlockElement single_block_info = {
      pinocchio::internal::MatrixBlockType::Identity, block_size};
    matrix_block_elements_vector.push_back(single_block_info);
    BOOST_CHECK(matrix_block_elements_vector.back().isValid());
  }

  const double scale = 1.;
  M11 scale_mat = M11(scale);
  {
    const auto matrix_map = make_map<MatrixMap>(scale_mat);
    matrix_block_elements_vector.push_back(
      {pinocchio::internal::MatrixBlockType::ScalarIdentity, block_size, matrix_map});
    BOOST_CHECK(matrix_block_elements_vector.back().isValid());
  }

  Matrix diagonal_vector = Matrix::Ones(block_size, 1);
  {
    const auto matrix_map = make_map<MatrixMap>(diagonal_vector);
    matrix_block_elements_vector.push_back(
      {pinocchio::internal::MatrixBlockType::Diagonal, block_size, matrix_map});
    BOOST_CHECK(matrix_block_elements_vector.back().isValid());
  }

  Matrix identity_plain = Matrix::Identity(block_size, block_size);
  {
    const auto matrix_map = make_map<MatrixMap>(identity_plain);
    matrix_block_elements_vector.push_back(
      {pinocchio::internal::MatrixBlockType::Plain, block_size, matrix_map});
    BOOST_CHECK(matrix_block_elements_vector.back().isValid());
  }

  internal::BlockDiagonalMatrix res(matrix_block_elements_vector);
  BOOST_CHECK(res.getMatrixStack()[0].data() != matrix_block_elements_vector[2].map.data());
  BOOST_CHECK(res.getMatrixStack()[1].data() != matrix_block_elements_vector[3].map.data());
  BOOST_CHECK(res.getMatrixStack()[2].data() != matrix_block_elements_vector[4].map.data());

  return res;
}

BOOST_AUTO_TEST_CASE(test_multiple_blocks)
{
  const Eigen::Index block_size = 10;
  const auto block_diagonal_matrix = create_multiple_block_info(block_size);
  const auto num_blocks = int(block_diagonal_matrix.getMatrixBlockElements().size());

  BOOST_CHECK(block_diagonal_matrix.rows() == num_blocks * block_size);

  const auto plain_matrix = block_diagonal_matrix.matrix();
  BOOST_CHECK(plain_matrix.topLeftCorner(block_size, block_size).isZero(0));
  BOOST_CHECK(plain_matrix.bottomRightCorner(num_blocks, num_blocks).isIdentity(0));

  test_assignment(block_diagonal_matrix);
  test_applyOnTheRight(block_diagonal_matrix);
  test_applyOnTheLeft(block_diagonal_matrix);

  // Test diagonal
  const auto diagonal_elements = block_diagonal_matrix.diagonal();
  BOOST_CHECK(diagonal_elements == plain_matrix.diagonal());
}

BOOST_AUTO_TEST_CASE(test_hasNaN)
{
  const Eigen::Index block_size = 10;
  const auto block_diagonal_matrix = create_multiple_block_info(block_size);

  BOOST_CHECK(!block_diagonal_matrix.hasNaN());

  auto block_diagonal_matrix_bis = block_diagonal_matrix;
  block_diagonal_matrix_bis.getMatrixBlockElements().back().container().fill(NAN);
  BOOST_CHECK(block_diagonal_matrix_bis.getMatrixBlockElements().back().hasNaN());
  BOOST_CHECK(block_diagonal_matrix_bis.hasNaN());
}

BOOST_AUTO_TEST_CASE(test_add_bdm_diag)
{
  const Eigen::Index block_size = 5;
  const auto bdm = create_multiple_block_info(block_size);
  const auto size = bdm.rows();

  const Vector diag_vec = Vector::Random(size);
  const auto diag_mat = diag_vec.asDiagonal();

  const Matrix bdm_plain = bdm.matrix();
  const Matrix res_ref = bdm_plain + Matrix(diag_mat);

  auto bdm_res = bdm;
  bdm_res += diag_mat;

  BOOST_CHECK(bdm_res.matrix().isApprox(res_ref));
}

BOOST_AUTO_TEST_CASE(test_add_bdm_diag_operator_plus)
{
  const Eigen::Index block_size = 5;
  const auto bdm = create_multiple_block_info(block_size);
  const auto size = bdm.rows();

  const Vector diag_vec = Vector::Random(size);
  const auto diag_mat = diag_vec.asDiagonal();

  const Matrix bdm_plain = bdm.matrix();
  const Matrix res_ref = bdm_plain + Matrix(diag_mat);

  const internal::BlockDiagonalMatrix bdm_res = bdm + diag_mat;

  BOOST_CHECK(bdm_res.matrix().isApprox(res_ref));
}

BOOST_AUTO_TEST_CASE(test_inverse)
{
  const Eigen::Index block_size = 10;
  const auto block_diagonal_matrix = create_multiple_block_info(block_size);

  const auto block_diagonal_matrix_inverse_expression = block_diagonal_matrix.inverse();

  pinocchio::internal::BlockDiagonalMatrix block_diagonal_matrix_inverse_value;
  {
    std::vector<internal::BlockDiagonalMatrix::MatrixBlockElement::PlainBlockElement>
      inverse_pattern;
    for (const auto & block : block_diagonal_matrix.getMatrixBlockElements())
    {
      inverse_pattern.push_back(block.inverse());
    }
    block_diagonal_matrix_inverse_value.rebuild(inverse_pattern);
  }

  block_diagonal_matrix_inverse_value = block_diagonal_matrix_inverse_expression;

  const auto bdm_plain = block_diagonal_matrix.matrix();
  const auto bdm_inv_plain = block_diagonal_matrix_inverse_value.matrix();

  // We skip the first block (Zero) which has Inf in inverse
  const auto skip_size = block_size;
  const auto check_rows = bdm_plain.rows() - skip_size;
  const auto check_cols = bdm_plain.cols() - skip_size;

  BOOST_CHECK(bdm_inv_plain.bottomRightCorner(check_rows, check_cols)
                .isApprox(bdm_plain.bottomRightCorner(check_rows, check_cols).inverse()));
}

BOOST_AUTO_TEST_CASE(test_inverse_aliasing)
{
  const Eigen::Index block_size = 10;
  auto bdm = create_multiple_block_info(block_size);

  // Remove the first block (Zero) to ensure the matrix is invertible
  {
    std::vector<MatrixBlockElement> elements = bdm.blocks();
    elements.erase(elements.begin());
    bdm.rebuild(elements);
  }

  const Matrix original_plain = bdm.matrix();
  const Matrix expected_inverse = original_plain.inverse();

  // Perform in-place inverse
  bdm = bdm.inverse();

  BOOST_CHECK(bdm.matrix().isApprox(expected_inverse));
}

BOOST_AUTO_TEST_CASE(test_inverse_rebuild)
{
  const Eigen::Index block_size = 10;
  const auto bdm = create_multiple_block_info(block_size);

  internal::BlockDiagonalMatrix res; // Empty
  res = bdm.inverse();

  // Check structure
  BOOST_CHECK(res.blocks().size() == bdm.blocks().size());
  BOOST_CHECK(
    res.blocks()[0].type() == pinocchio::internal::MatrixBlockType::Plain); // Zero -> Plain

  // Check values (excluding Zero block)
  const auto bdm_plain = bdm.matrix();
  const auto res_plain = res.matrix();

  const auto skip_size = block_size;
  const auto check_rows = bdm_plain.rows() - skip_size;
  const auto check_cols = bdm_plain.cols() - skip_size;

  BOOST_CHECK(res_plain.bottomRightCorner(check_rows, check_cols)
                .isApprox(bdm_plain.bottomRightCorner(check_rows, check_cols).inverse()));
}

BOOST_AUTO_TEST_CASE(test_rebuild)
{
  const Eigen::Index block_size1 = 20;
  auto block_diagonal_matrix1 = create_multiple_block_info(block_size1);
  const void * block_diagonal_matrix1_data_ptr = block_diagonal_matrix1.getMatrixStack().data();

  const Eigen::Index block_size2 = 10;
  auto block_diagonal_matrix2 = create_multiple_block_info(block_size2);
  const void * block_diagonal_matrix2_data_ptr = block_diagonal_matrix2.getMatrixStack().data();

  BOOST_CHECK(block_diagonal_matrix1 != block_diagonal_matrix2);

  const auto block_diagonal_matrix1_copy = block_diagonal_matrix1;
  block_diagonal_matrix1.rebuild(block_diagonal_matrix2.getMatrixBlockElements());
  // BOOST_CHECK(block_diagonal_matrix1.rows() == block_diagonal_matrix2.rows());
  // BOOST_CHECK(block_diagonal_matrix1.cols() == block_diagonal_matrix2.cols());
  // BOOST_CHECK(block_diagonal_matrix1.getMatrixBlockElements().size() ==
  // block_diagonal_matrix2.getMatrixBlockElements().size());
  BOOST_CHECK(block_diagonal_matrix1 == block_diagonal_matrix2);
  BOOST_CHECK(
    block_diagonal_matrix1.getMatrixStack().data()
    == block_diagonal_matrix1_data_ptr); // no realloc

  block_diagonal_matrix2.rebuild(block_diagonal_matrix1_copy.getMatrixBlockElements());
  BOOST_CHECK(block_diagonal_matrix2 == block_diagonal_matrix1_copy);
  BOOST_CHECK(
    block_diagonal_matrix2.getMatrixStack().data() != block_diagonal_matrix2_data_ptr); // realloc
}

BOOST_AUTO_TEST_CASE(test_operator_equal)
{
  internal::BlockDiagonalMatrix bdm;
  bdm = Eigen::VectorXd::Constant(0, 3.14).asDiagonal();

  internal::BlockDiagonalMatrix bdm2 = bdm;

  BOOST_CHECK(bdm2 == bdm);
}

/// Helper: build a 2-outer-block BDM where one block is NestedBlockDiagonal.
/// Layout: [Diagonal(3×3)] [NestedBlockDiagonal: (Diagonal(2×2), Plain(3×3))]
static internal::BlockDiagonalMatrix create_nested_block_diagonal_matrix()
{
  const Eigen::Index flat_diag_size = 3;
  const Eigen::Index sub_diag_size = 2;
  const Eigen::Index sub_plain_size = 3;

  // Outer block 0: plain Diagonal
  MatrixBlockElement flat_block(pinocchio::internal::MatrixBlockType::Diagonal, flat_diag_size);

  // Outer block 1: NestedBlockDiagonal containing Diagonal + Plain sub-blocks
  std::vector<MatrixBlockElement> subs;
  subs.emplace_back(pinocchio::internal::MatrixBlockType::Diagonal, sub_diag_size);
  subs.emplace_back(pinocchio::internal::MatrixBlockType::Plain, sub_plain_size);
  MatrixBlockElement nested_block(
    pinocchio::internal::MatrixBlockType::NestedBlockDiagonal, std::move(subs));

  internal::BlockDiagonalMatrix bdm({flat_block, nested_block});
  for (auto & block : bdm.blocks())
    block.setRandomPD(); // make it invertible for inverse tests
  return bdm;
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_construction)
{
  const auto bdm = create_nested_block_diagonal_matrix();

  const Eigen::Index expected_rows = 3 + 2 + 3; // flat_diag + sub_diag + sub_plain
  BOOST_CHECK(bdm.rows() == expected_rows);
  BOOST_CHECK(bdm.cols() == expected_rows);
  BOOST_CHECK(bdm.blocks().size() == 2); // ONE outer block per constraint
  BOOST_CHECK(bdm.blocks()[0].type() == pinocchio::internal::MatrixBlockType::Diagonal);
  BOOST_CHECK(bdm.blocks()[1].type() == pinocchio::internal::MatrixBlockType::NestedBlockDiagonal);
  BOOST_CHECK(bdm.blocks()[1].nested_blocks().size() == 2);
  BOOST_CHECK(
    bdm.blocks()[1].nested_blocks()[0].type() == pinocchio::internal::MatrixBlockType::Diagonal);
  BOOST_CHECK(
    bdm.blocks()[1].nested_blocks()[1].type() == pinocchio::internal::MatrixBlockType::Plain);

  // Plain matrix should have zeros off the diagonal sub-blocks
  const Matrix plain = bdm.matrix();
  BOOST_CHECK(plain.block(0, 3, 3, 5).isZero()); // flat block row vs nested block columns
  BOOST_CHECK(plain.block(3, 0, 5, 3).isZero()); // nested block rows vs flat block columns
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_assignment)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  test_assignment(bdm);
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_apply_on_the_right)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  test_applyOnTheRight(bdm);
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_apply_on_the_left)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  test_applyOnTheLeft(bdm);
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_diagonal)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  const Matrix plain = bdm.matrix();
  const Vector diag_ref = plain.diagonal();
  const Vector diag_result = bdm.diagonal();
  BOOST_CHECK(diag_result.isApprox(diag_ref));
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_inverse)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  const internal::BlockDiagonalMatrix inv = bdm.inverse();

  BOOST_CHECK(inv.blocks().size() == bdm.blocks().size());
  BOOST_CHECK(inv.blocks()[1].type() == pinocchio::internal::MatrixBlockType::NestedBlockDiagonal);

  const Matrix plain = bdm.matrix();
  const Matrix inv_plain = inv.matrix();
  const Eigen::Index n = bdm.rows();
  BOOST_CHECK((plain * inv_plain).isApprox(Matrix::Identity(n, n)));
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_sum_with_diagonal)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  const Eigen::Index n = bdm.rows();

  const Vector delta = Vector::Random(n).cwiseAbs(); // positive entries
  const auto diag_mat = delta.asDiagonal();

  const internal::BlockDiagonalMatrix res = bdm + diag_mat;

  const Matrix plain_bdm = bdm.matrix();
  const Matrix expected = plain_bdm + Matrix(diag_mat);
  BOOST_CHECK(res.matrix().isApprox(expected));

  // The result's blocks()[1] should still be NestedBlockDiagonal (types upgraded)
  BOOST_CHECK(res.blocks()[1].type() == pinocchio::internal::MatrixBlockType::NestedBlockDiagonal);
}

BOOST_AUTO_TEST_CASE(test_nested_block_diagonal_copy)
{
  const auto bdm = create_nested_block_diagonal_matrix();
  const internal::BlockDiagonalMatrix bdm_copy = bdm;

  BOOST_CHECK(bdm_copy == bdm);
  // Copies must be independent (no aliasing in NestedBlockDiagonal sub-blocks)
  BOOST_CHECK(
    bdm_copy.blocks()[1].nested_blocks()[0].map.data()
    != bdm.blocks()[1].nested_blocks()[0].map.data());
}
