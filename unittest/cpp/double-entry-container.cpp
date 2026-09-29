//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE double_entry_container

#include "pinocchio/container/double-entry-container.hpp"
#include "pinocchio/container/matrix-stack.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_all_std_vector)
{
  typedef Eigen::Matrix<double, 6, 6> Matrix6;

  typedef std::vector<Matrix6> Vector;
  typedef internal::DoubleEntryContainer<Vector> Container;

  const Eigen::Index nrows = 20, ncols = 20;

  Container container(nrows, ncols);
  BOOST_CHECK(container.size() == 0);
  BOOST_CHECK(container.rows() == nrows);
  BOOST_CHECK(container.cols() == ncols);
  BOOST_CHECK(container.begin() == container.end());

  container.reserve(size_t(nrows));
  BOOST_CHECK(container.values().capacity() == size_t(nrows));

  // Fill diagonal with random values
  for (Eigen::Index id = 0; id < nrows; id++)
  {
    const bool res = container.insert(id, id, Matrix6::Constant(double(id)));
    BOOST_CHECK(res);
    BOOST_CHECK(container.size() == size_t(id + 1));
  }

  for (Eigen::Index id = 0; id < nrows; id++)
  {
    const bool res = container.insert(id, id, Matrix6::Constant(double(id)));
    BOOST_CHECK(!res);
    BOOST_CHECK(container.size() == size_t(nrows));
  }

  {
    const Eigen::VectorXi linear_range = Eigen::VectorXi::LinSpaced(nrows, 0, nrows - 1);
    BOOST_CHECK(container.keys().matrix().diagonal() == linear_range.cast<long>());
  }

  // Set diagonal and check
  container.fill(Matrix6::Identity());
  for (auto val : container)
    BOOST_CHECK(val == Matrix6::Identity());

  // Check method find
  for (Eigen::Index row_id = 0; row_id < nrows; row_id++)
  {
    for (Eigen::Index col_id = 0; col_id < ncols; col_id++)
    {
      if (row_id == col_id)
        BOOST_CHECK(*container.find(row_id, col_id) == Matrix6::Identity());
      else
        BOOST_CHECK(container.find(row_id, col_id) == container.end());
    }
  }

  const Container copy(container);
  BOOST_CHECK(container == copy);

  // No change
  for (Eigen::Index id = 0; id < nrows; id++)
  {
    container[{id, id}].setIdentity();
  }
  BOOST_CHECK(container == copy);

  // Change values
  for (Eigen::Index id = 0; id < nrows; id++)
  {
    container[{id, id}].setZero();
    BOOST_CHECK((container[{id, id}] == Matrix6::Zero()));
  }
  BOOST_CHECK(container != copy);

  // Restore values
  for (Eigen::Index id = 0; id < nrows; id++)
  {
    container[{id, id}].setIdentity();
  }
  BOOST_CHECK(container == copy);

  // Apply
  container.apply([](Matrix6 & v) { v.setZero(); });
  container.apply([](const Matrix6 & v) { BOOST_CHECK(v.isZero(0)); });
  BOOST_CHECK(container != copy);

  container.apply([](Matrix6 & v) { v.setIdentity(); });
  BOOST_CHECK(container == copy);

  // Remove elt (4,4)
  BOOST_CHECK(!container.remove(3, 4));
  BOOST_CHECK(container == copy);
  BOOST_CHECK(container.find(4, 4) != container.end());
  BOOST_CHECK(container.remove(4, 4));
  BOOST_CHECK(container != copy);
  BOOST_CHECK(container.size() == size_t(nrows - 1));
  BOOST_CHECK(container.find(4, 4) == container.end());

  // Check operator[] insertion
  container.insert({4, 4}, Matrix6::Identity());
  BOOST_CHECK(container.keys()(4, 4) == long(nrows - 1));
  BOOST_CHECK(container != copy);
}

BOOST_AUTO_TEST_CASE(test_all_matrix_stack)
{
  typedef Eigen::Matrix<double, 6, 6> Matrix6;

  typedef internal::MatrixStackTpl<Matrix6> Vector;
  typedef internal::DoubleEntryContainer<Vector> Container;

  const Eigen::Index nrows = 20, ncols = 20;

  Container container(nrows, ncols);

  BOOST_CHECK(container.size() == 0);
  BOOST_CHECK(container.rows() == nrows);
  BOOST_CHECK(container.cols() == ncols);
  BOOST_CHECK(container.begin() == container.end());

  container.reserve(size_t(nrows));
  BOOST_CHECK(container.values().capacity() == size_t(nrows));

  // Fill diagonal with random values
  for (Eigen::Index id = 0; id < nrows; id++)
  {
    const bool res = container.insert(id, id, Matrix6::Constant(double(id)));
    BOOST_CHECK(res);
    BOOST_CHECK(container.size() == size_t(id + 1));
  }

  for (Eigen::Index id = 0; id < nrows; id++)
  {
    BOOST_CHECK(container.values()[size_t(id)].isConstant(double(id), double(0)));
  }
}
