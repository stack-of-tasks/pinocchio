//
// Copyright (c) 2025-2026 INRIA
//

#include "pinocchio/utils/size-in-bytes.hpp"
#include "pinocchio/math.hpp"

#include <cstddef>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

template<int size>
struct SimpleStruct
{
  SimpleStruct() = default;

  std::size_t sizeInBytes() const
  {
    return sizeof(char) * size;
  }

  char data[size];
};

typedef SimpleStruct<1> SimpleStruct1;
typedef SimpleStruct<10> SimpleStruct10;
typedef SimpleStruct<100> SimpleStruct100;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_simple_struct)
{
  BOOST_CHECK(1 == SimpleStruct1().sizeInBytes());
  BOOST_CHECK(internal::sizeInBytes(SimpleStruct1()) == SimpleStruct1().sizeInBytes());
  //
  BOOST_CHECK(10 == SimpleStruct10().sizeInBytes());
  BOOST_CHECK(internal::sizeInBytes(SimpleStruct10()) == SimpleStruct10().sizeInBytes());
  //
  BOOST_CHECK(100 == SimpleStruct100().sizeInBytes());
  BOOST_CHECK(internal::sizeInBytes(SimpleStruct100()) == SimpleStruct100().sizeInBytes());
}

BOOST_AUTO_TEST_CASE(test_std_vector)
{
  std::vector<SimpleStruct1> vector(100);
  BOOST_CHECK(internal::sizeInBytes(vector) == vector.size() * vector[0].sizeInBytes());
}

BOOST_AUTO_TEST_CASE(test_std_array)
{
  std::array<SimpleStruct1, 100> array;
  BOOST_CHECK(internal::sizeInBytes(array) == array.size() * array[0].sizeInBytes());
}

BOOST_AUTO_TEST_CASE(test_eigen_matrix)
{
  const Eigen::Matrix3d mat33{};
  BOOST_CHECK(internal::sizeInBytes(mat33) == sizeof(mat33));

  const Eigen::MatrixXd mat(mat33);
  BOOST_CHECK(internal::sizeInBytes(mat) - internal::sizeInBytes(mat33) == 1);
}

BOOST_AUTO_TEST_CASE(test_eigen_map)
{
  const Eigen::Matrix3d mat{};
  const auto mat_map = make_default_map(mat);
  BOOST_CHECK(internal::sizeInBytes(mat_map) == internal::sizeInBytes(mat));
}

BOOST_AUTO_TEST_CASE(test_fundamental_types)
{
  BOOST_CHECK(internal::sizeInBytes(bool(1)) == sizeof(bool));
  BOOST_CHECK(internal::sizeInBytes(char(1)) == sizeof(char));
  BOOST_CHECK(internal::sizeInBytes(int(1)) == sizeof(int));
  BOOST_CHECK(internal::sizeInBytes(float(1)) == sizeof(float));
  BOOST_CHECK(internal::sizeInBytes(double(1)) == sizeof(double));
  BOOST_CHECK(internal::sizeInBytes((long double)(1)) == sizeof(long double));
}

BOOST_AUTO_TEST_SUITE_END()
