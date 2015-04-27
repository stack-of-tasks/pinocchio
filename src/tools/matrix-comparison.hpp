//
// Copyright (c) 2015 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include <Eigen/Core>
#include <limits>

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

inline void is_matrix_absolutely_closed (const Eigen::MatrixXd & M1,
                              const Eigen::MatrixXd & M2,
                              double tolerance = std::numeric_limits <Eigen::MatrixXd::Scalar>::epsilon ()
                              )
{
  BOOST_REQUIRE_EQUAL (M1.rows (), M2.rows ());
  BOOST_REQUIRE_EQUAL (M1.cols (), M2.cols ());

  for (Eigen::MatrixXd::Index i = 0; i < M1.rows (); i++)
  {
    for (Eigen::MatrixXd::Index j = 0; j < M1.cols (); j++)
    {
      BOOST_CHECK_SMALL (M1 (i,j) - M2 (i,j), tolerance);
    }
  }
}
