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
