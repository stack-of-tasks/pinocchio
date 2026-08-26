//
// Copyright (c) 202-2026 INRIA
//

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "pinocchio/container/matrix-stack.hpp"

// #define ALIGNMENT_VALUE 128
#define ALIGNMENT_VALUE 8

using namespace pinocchio;
typedef Eigen::MatrixXf MatrixXs;
typedef MatrixXs::Scalar Scalar;
typedef internal::MatrixStackTpl<MatrixXs, ALIGNMENT_VALUE> MatrixXsStack;
typedef internal::MatrixStackTpl<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXs), ALIGNMENT_VALUE>
  RowMatrixXsStack;

bool is_aligned(const void * ptr, const std::size_t alignment)
{
  assert(
    alignment >= sizeof(void *) && (alignment & (alignment - 1)) == 0
    && "Alignment must be at least sizeof(void*) and a power of 2");
  return reinterpret_cast<std::size_t>(ptr) % alignment == 0;
}

template<typename T>
const T & as_const(T & value)
{
  return value;
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(print_info)
{
  std::cout << "sizeof(Scalar): " << sizeof(Scalar) << std::endl;
  std::cout << "alignment: " << MatrixXsStack::Alignment << std::endl;
  std::cout << "sizeof(void*): " << sizeof(void *) << std::endl;
}

BOOST_AUTO_TEST_CASE(matrix_stack_empty)
{
  MatrixXsStack matrix_stack(0);
  BOOST_CHECK(matrix_stack.size() == 0);
  BOOST_CHECK(matrix_stack.capacity() == 0);
  BOOST_CHECK(matrix_stack.data() == nullptr);

  MatrixXsStack matrix_stack_copy(matrix_stack);
  BOOST_CHECK(matrix_stack_copy.size() == 0);
  BOOST_CHECK(matrix_stack_copy.capacity() == 0);
  BOOST_CHECK(matrix_stack_copy.data() == nullptr);
}

BOOST_AUTO_TEST_CASE(matrix_stack_default)
{

  BOOST_CHECK(ALIGNMENT_VALUE == MatrixXsStack::Alignment);

  { // case with no allocation
    MatrixXsStack matrix_stack(100);
    BOOST_CHECK(matrix_stack.size() == 0);
    BOOST_CHECK(matrix_stack.capacity() == 100);

    BOOST_CHECK(matrix_stack.data() == nullptr);
    BOOST_CHECK(is_aligned(matrix_stack.data(), MatrixXsStack::Alignment));

    // First push_back
    matrix_stack.push_back(2, 2);
    BOOST_CHECK(matrix_stack.data() != nullptr);
    BOOST_CHECK(matrix_stack.size() == 1);
    BOOST_CHECK(is_aligned(matrix_stack.data(), MatrixXsStack::Alignment));
    BOOST_CHECK(matrix_stack.back().rows() == 2);
    BOOST_CHECK(matrix_stack.back().cols() == 2);
    BOOST_CHECK(matrix_stack.data() == matrix_stack.back().data());

    matrix_stack.back().setOnes();
    BOOST_CHECK(matrix_stack.back().isOnes(0));

    // Second push_back
    matrix_stack.push_back(3, 3);
    BOOST_CHECK(matrix_stack.size() == 2);
    BOOST_CHECK(matrix_stack.data() != matrix_stack.back().data());
    BOOST_CHECK(is_aligned(matrix_stack.back().data(), MatrixXsStack::Alignment));
    BOOST_CHECK(matrix_stack.back().rows() == 3);
    BOOST_CHECK(matrix_stack.back().cols() == 3);
    matrix_stack.back().setConstant(2);

    BOOST_CHECK(matrix_stack[0].isOnes(0));
    BOOST_CHECK(matrix_stack[1].isConstant(2, 0));

    // operator==
    BOOST_CHECK(matrix_stack == matrix_stack);

    // Test copy constructor
    MatrixXsStack matrix_stack_copy = matrix_stack;
    BOOST_CHECK(matrix_stack_copy == matrix_stack);

    matrix_stack_copy.push_back(1, 6);
    BOOST_CHECK(matrix_stack_copy != matrix_stack);
  }

  { // case with memory allocation
    const Eigen::Index max_rows = 3;
    const Eigen::Index max_cols = 4;
    const std::size_t matrix_max_size = max_rows * max_cols;
    MatrixXsStack matrix_stack(100, matrix_max_size);
    BOOST_CHECK(matrix_stack.size() == 0);
    BOOST_CHECK(matrix_stack.capacity() == 100);

    BOOST_CHECK(matrix_stack.data() != 0);
    BOOST_CHECK(is_aligned(matrix_stack.data(), MatrixXsStack::Alignment));

    // First push_back
    matrix_stack.push_back(2, 2);
    BOOST_CHECK(matrix_stack.data() == matrix_stack.back().data());
    BOOST_CHECK(is_aligned(matrix_stack.back().data(), MatrixXsStack::Alignment));
    BOOST_CHECK(matrix_stack.back().rows() == 2);
    BOOST_CHECK(matrix_stack.back().cols() == 2);
    matrix_stack.back().setOnes();

    // Check constness
    BOOST_CHECK(as_const(matrix_stack).data() == as_const(matrix_stack).back().data());
    BOOST_CHECK(as_const(matrix_stack).back().rows() == 2);
    BOOST_CHECK(as_const(matrix_stack).back().cols() == 2);
    BOOST_CHECK(as_const(matrix_stack).back().isOnes(0.));

    // Check a second push_back
    matrix_stack.push_back(3, 3);
    BOOST_CHECK(matrix_stack.size() == 2);
    BOOST_CHECK(matrix_stack.data() != matrix_stack.back().data());
    BOOST_CHECK(is_aligned(matrix_stack.back().data(), MatrixXsStack::Alignment));
    BOOST_CHECK(matrix_stack.back().rows() == 3);
    BOOST_CHECK(matrix_stack.back().cols() == 3);
    matrix_stack.back().setConstant(2);

    // Check element values
    BOOST_CHECK(matrix_stack[0].isOnes(0.));
    BOOST_CHECK(matrix_stack[1].isConstant(2., 0.));

    // Check distance between the two pointers
    const std::size_t first_elt_raw_size =
      std::size_t(matrix_stack[0].size()) * sizeof(MatrixXsStack::Scalar);
    const std::size_t max_distance = first_elt_raw_size + MatrixXsStack::Alignment;
    const std::size_t ptr_distance = reinterpret_cast<std::size_t>(matrix_stack[1].data())
                                     - reinterpret_cast<std::size_t>(matrix_stack[0].data());

    BOOST_CHECK(ptr_distance <= max_distance);
    BOOST_CHECK(ptr_distance >= first_elt_raw_size);

    std::cout << "ptr_distance: " << ptr_distance << std::endl;
    std::cout << "first_elt_raw_size: " << first_elt_raw_size << std::endl;
    std::cout << "max_distance: " << max_distance << std::endl;
  }
}

BOOST_AUTO_TEST_CASE(matrix_stack_from_matrix_info)
{
  // Zero block allocation
  {
    const std::vector<internal::MatrixInfo> matrix_infos;
    const MatrixXsStack matrix_stack(matrix_infos);

    BOOST_CHECK(matrix_stack.empty());
  }

  // Single block allocation
  {
    const Eigen::Index rows = 10, cols = 20;
    const std::vector<internal::MatrixInfo> matrix_infos = {{rows, cols}};

    const MatrixXsStack matrix_stack(matrix_infos);

    BOOST_CHECK(!matrix_stack.empty());
    BOOST_CHECK(matrix_stack.back().rows() == rows);
    BOOST_CHECK(matrix_stack.back().cols() == cols);
    BOOST_CHECK(matrix_stack.size() == 1);
  }

  // Multiple block allocation
  {
    const size_t num_blocks = 10;
    const Eigen::Index rows = 10, cols = 20;
    std::vector<internal::MatrixInfo> matrix_infos(num_blocks);

    for (const auto & block_info : matrix_infos)
    {
      BOOST_CHECK(!block_info.isValid());
    }

    for (auto & block_info : matrix_infos)
      block_info = {rows, cols};

    const MatrixXsStack matrix_stack(matrix_infos);

    BOOST_CHECK(!matrix_stack.empty());
    BOOST_CHECK(matrix_stack.size() == num_blocks);

    for (const auto & map : matrix_stack)
    {
      BOOST_CHECK(map.rows() == rows);
      BOOST_CHECK(map.cols() == cols);
    }
  }
}

BOOST_AUTO_TEST_CASE(matrix_stack_move_constructor)
{
  const Eigen::Index rows = 10, cols = 20;
  const std::vector<internal::MatrixInfo> matrix_infos = {{rows, cols}};

  MatrixXsStack matrix_stack(matrix_infos);
  matrix_stack.back().setZero();
  const MatrixXs map_copy = matrix_stack.back();

  const MatrixXsStack matrix_stack_move(std::move(matrix_stack));
  BOOST_CHECK(matrix_stack_move.back() == map_copy);
}

BOOST_AUTO_TEST_CASE(matrix_stack_move_assignment_operator)
{
  const Eigen::Index rows = 10, cols = 20;
  const std::vector<internal::MatrixInfo> matrix_infos = {{rows, cols}};

  MatrixXsStack matrix_stack(matrix_infos);
  matrix_stack.back().setIdentity();
  const MatrixXs map_copy = matrix_stack.back();

  MatrixXsStack matrix_stack_move(matrix_infos);
  matrix_stack_move.back().setZero();

  BOOST_CHECK(matrix_stack != matrix_stack_move);

  // Call move assignment operator
  matrix_stack_move = std::move(matrix_stack);
  BOOST_CHECK(matrix_stack_move.back() == map_copy);
}

BOOST_AUTO_TEST_CASE(matrix_stack_rebuild)
{
  const Eigen::Index rows1 = 10, cols1 = 20;
  const std::vector<internal::MatrixInfo> matrix_infos1 = {{rows1, cols1}};

  MatrixXsStack matrix_stack1(matrix_infos1);
  const void * matrix_stack1_data_ptr = matrix_stack1.data();
  matrix_stack1.back().setIdentity();

  const Eigen::Index rows2 = 20, cols2 = 40;
  const std::vector<internal::MatrixInfo> matrix_infos2 = {{rows2, cols2}};

  MatrixXsStack matrix_stack2(matrix_infos2);
  const void * matrix_stack2_data_ptr = matrix_stack2.data();
  BOOST_CHECK(matrix_stack2 != matrix_stack1);
  BOOST_CHECK(matrix_stack2.back().rows() == rows2);
  BOOST_CHECK(matrix_stack2.back().cols() == cols2);

  matrix_stack2.rebuild(matrix_infos1);
  matrix_stack2.back().setIdentity();
  BOOST_CHECK(matrix_stack2.data() == matrix_stack2_data_ptr);
  BOOST_CHECK(matrix_stack2.back().rows() == rows1);
  BOOST_CHECK(matrix_stack2.back().cols() == cols1);
  BOOST_CHECK(matrix_stack2.back() == matrix_stack1.back());

  matrix_stack1.rebuild(matrix_infos2);
  BOOST_CHECK(matrix_stack1.data() != matrix_stack1_data_ptr); // new allocation
  BOOST_CHECK(matrix_stack1.back().rows() == rows2);
  BOOST_CHECK(matrix_stack1.back().cols() == cols2);
}

BOOST_AUTO_TEST_CASE(matrix_stack_clear)
{
  MatrixXsStack matrix_stack(100);
  matrix_stack.push_back(1, 1);
  matrix_stack.push_back(1, 1);
  matrix_stack.push_back(1, 1);
  BOOST_CHECK(matrix_stack.size() == 3);
  BOOST_CHECK(!matrix_stack.empty());

  matrix_stack.clear();
  BOOST_CHECK(matrix_stack.size() == 0);
  BOOST_CHECK(matrix_stack.empty());

  MatrixXsStack matrix_stack_copy = matrix_stack;
  BOOST_CHECK(matrix_stack_copy.size() == 0);
  BOOST_CHECK(matrix_stack_copy.empty());
  BOOST_CHECK(matrix_stack_copy == matrix_stack);
}

BOOST_AUTO_TEST_CASE(matrix_stack_erase)
{
  MatrixXsStack matrix_stack(100);
  matrix_stack.push_back(1, 1);
  matrix_stack.push_back(1, 1);
  matrix_stack.push_back(1, 1);
  BOOST_CHECK(matrix_stack.size() == 3);

  matrix_stack.erase(matrix_stack.begin());
  BOOST_CHECK(matrix_stack.size() == 2);
}

BOOST_AUTO_TEST_CASE(matrix_stack_apply)
{
  MatrixXsStack matrix_stack(100);
  typedef MatrixXsStack::MapType MapType;
  matrix_stack.push_back(1, 1);
  matrix_stack.push_back(1, 1);
  matrix_stack.push_back(1, 1);

  matrix_stack.apply([](MapType v) { v.setOnes(); });
  for (const auto & map : matrix_stack)
  {
    BOOST_CHECK(map.isOnes(0));
  }

  matrix_stack.apply([](MapType v) { v.fill(10.); });
  as_const(matrix_stack).apply([](const MapType v) { BOOST_CHECK(v.isConstant(10., 0)); });
}

BOOST_AUTO_TEST_CASE(matrix_stack_no_malloc)
{
  constexpr auto matrix_size = 36;
  const size_t stack_size = 100;
  MatrixXsStack matrix_stack(stack_size, matrix_size);

  const void * init_data_ptr = matrix_stack.data();
  BOOST_CHECK(init_data_ptr != nullptr);
  BOOST_CHECK(is_aligned(init_data_ptr, MatrixXsStack::Alignment));
  BOOST_CHECK(matrix_stack.size() == 0);
  BOOST_CHECK(matrix_stack.capacity() == stack_size);

  for (size_t k = 0; k < stack_size; ++k)
  {
    matrix_stack.push_back(6, 6);
    BOOST_CHECK(is_aligned(matrix_stack.data(), MatrixXsStack::Alignment));
    BOOST_CHECK(matrix_stack.data() == init_data_ptr);
    matrix_stack.back().setConstant(float(k));
    if (matrix_stack.data() != init_data_ptr)
    {
      std::cout << "matrix_stack.data(): " << matrix_stack.data() << std::endl;
      std::cout << "init_data_ptr: " << init_data_ptr << std::endl;
    }
  }
  BOOST_CHECK(matrix_stack.size() == stack_size);

  for (size_t k = 0; k < stack_size; ++k)
  {
    BOOST_CHECK(matrix_stack[k].isConstant(float(k), float(0)));
  }

  matrix_stack.push_back(6, 6);
  matrix_stack.back().setConstant(float(stack_size));

  BOOST_CHECK(matrix_stack.size() == stack_size + 1);
  BOOST_CHECK(matrix_stack.capacity() >= stack_size + 1);
  BOOST_CHECK(matrix_stack.data() == init_data_ptr);

  for (size_t k = 0; k < stack_size + 1; ++k)
  {
    BOOST_CHECK(matrix_stack[k].isConstant(float(k), float(0)));
  }

  auto current_stack_size = matrix_stack.size();
  while (true)
  {
    matrix_stack.push_back(6, 6);
    matrix_stack.back().setConstant(float(current_stack_size));
    current_stack_size += 1;

    if (matrix_stack.data() != init_data_ptr)
      break;
  }

  BOOST_CHECK(
    matrix_stack.data() != init_data_ptr); // After some push_back, we have a new allocation
  for (size_t k = 0; k < matrix_stack.size(); ++k)
  {
    const auto matrix = matrix_stack[k];
    BOOST_CHECK(matrix.isConstant(float(k), float(0)));
  }
}

BOOST_AUTO_TEST_CASE(matrix_stack_product)
{
  using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using DiagMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
  using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using MatrixStack = pinocchio::internal::MatrixStackTpl<MatrixXd>;

  const std::size_t N = static_cast<std::size_t>(std::rand() % 10);

  // random 3x3 products
  std::vector<Matrix3d> mat3_vec;
  std::vector<Vector3d> x3s;
  std::vector<Vector3d> expected_y3s; // y = mat * x
  for (std::size_t i = 0; i < N; ++i)
  {
    mat3_vec.emplace_back(Matrix3d::Random());
    x3s.emplace_back(Vector3d::Random());
    expected_y3s.emplace_back(mat3_vec.back() * x3s.back());
  }

  // random diagonal products
  std::vector<DiagMatrixXd> diagmat_vec;
  std::vector<VectorXd> xds;
  std::vector<VectorXd> expected_yds; // y = mat * x
  for (std::size_t i = 0; i < N; ++i)
  {
    const Eigen::Index size = static_cast<Eigen::Index>(std::rand() % 10);
    diagmat_vec.emplace_back(DiagMatrixXd::Random(size));
    xds.emplace_back(VectorXd::Random(size));
    expected_yds.emplace_back(diagmat_vec.back().asDiagonal() * xds.back());
  }

  // create matrix stack and alternate between 3x3 and diagonal matrices
  MatrixStack matrix_stack;
  for (std::size_t i = 0; i < N; ++i)
  {
    matrix_stack.push_back(mat3_vec[i]);
    matrix_stack.push_back(diagmat_vec[i]);
  }

  for (std::size_t i = 0; i < N; ++i)
  {
    const auto & mat3 = matrix_stack.get<Matrix3d>(2 * i);
    Vector3d res3 = mat3 * x3s[i];
    BOOST_CHECK(res3.isApprox(expected_y3s[i]));

    const auto & diag = matrix_stack[2 * i + 1];
    VectorXd resd = diag.asDiagonal() * xds[i];
    BOOST_CHECK(resd.isApprox(expected_yds[i]));
  }
}

BOOST_AUTO_TEST_CASE(matrix_stack_empty_matrix)
{
  MatrixXsStack matrix_stack;
  matrix_stack.push_back(3, 3);
  matrix_stack.push_back(0, 0); // check matrix stack can hold 0x0 matrices
  matrix_stack.push_back(4, 3);

  BOOST_CHECK(matrix_stack[0].rows() == 3);
  BOOST_CHECK(matrix_stack[0].cols() == 3);

  BOOST_CHECK(matrix_stack[1].rows() == 0);
  BOOST_CHECK(matrix_stack[1].cols() == 0);

  BOOST_CHECK(matrix_stack[2].rows() == 4);
  BOOST_CHECK(matrix_stack[2].cols() == 3);
}

BOOST_AUTO_TEST_SUITE_END()
