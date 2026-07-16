//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/spatial.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

namespace
{

  template<typename D>
  void addJointAndBody(
    Model & model,
    const JointModelBase<D> & jmodel,
    const Model::JointIndex parent_id,
    const SE3 & joint_placement,
    const std::string & joint_name,
    const Inertia & Y)
  {
    Model::JointIndex idx;

    idx = model.addJoint(parent_id, jmodel, joint_placement, joint_name);
    model.appendBodyToJoint(idx, Y);
  }

  /// @brief Get a predefined  spline joint trajectory.
  /// This was taken from the model opensim gait10dof18mus.osim, and correspond the knee joint.
  void getTrajectory(std::vector<SE3> & ctrlFrames)
  {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    rotation << -0.500004, 0, -0.866023, 0, 1, 0, 0.866023, 0, -0.500004;
    translation << -0.0032, -0.4226, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << -0.173649, 0, -0.984808, 0, 1, 0, 0.984808, 0, -0.173649;
    translation << 0.00179, -0.416947, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.173652, 0, -0.984807, 0, 1, 0, 0.984807, 0, 0.173652;
    translation << 0.00411, -0.411057, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.342021, 0, -0.939692, 0, 1, 0, 0.939692, 0, 0.342021;
    translation << 0.00438827, -0.4082, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.499998, 0, -0.866027, 0, 1, 0, 0.866027, 0, 0.499998;
    translation << 0.0041, -0.405495, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.766044, 0, -0.642788, 0, 1, 0, 0.642788, 0, 0.766044;
    translation << 0.00212, -0.400825, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.866025, 0, -0.5, 0, 1, 0, 0.5, 0, 0.866025;
    translation << 0.000757726, -0.399, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.939693, 0, -0.34202, 0, 1, 0, 0.34202, 0, 0.939693;
    translation << -0.001, -0.3976, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.984808, 0, -0.173648, 0, 1, 0, 0.173648, 0, 0.984808;
    translation << -0.0031, -0.3966, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.987363, 0, 0.158478, 0, 1, 0, -0.158478, 0, 0.987363;
    translation << -0.00513719, -0.395264, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.980591, 0, 0.196066, 0, 1, 0, -0.196066, 0, 0.980591;
    translation << -0.005227, -0.395149, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.94362, 0, 0.33103, 0, 1, 0, -0.33103, 0, 0.94362;
    translation << -0.005435, -0.394792, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.882249, 0, 0.470783, 0, 1, 0, -0.470783, 0, 0.882249;
    translation << -0.005574, -0.394507, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << 0.0493163, 0, 0.998783, 0, 1, 0, -0.998783, 0, 0.0493163;
    translation << -0.005435, -0.394812, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

    rotation << -0.500004, 0, 0.866023, 0, 1, 0, -0.866023, 0, -0.500004;
    translation << -0.00525, -0.396, 0;
    ctrlFrames.push_back(pinocchio::SE3(rotation, translation));
  }

  double bsplineBasis(int index, int degree, const double x, const Eigen::VectorXd & knots)
  {
    assert(0 <= index);
    assert(0 <= degree);
    if (degree == 0)
    {
      if (knots[index] <= x && x < knots[index + 1])
      {
        return 1.;
      }
      else
      {
        if (x == knots[knots.size() - 1] && x == knots[index + 1])
        {
          return 1;
        }
        return 0.;
      }
    }

    const double den1 = knots[index + degree] - knots[index];
    double left = 0.;
    if (den1 != 0.)
    {
      left = ((x - knots[index]) / den1) * bsplineBasis(index, degree - 1, x, knots);
    }

    const double den2 = knots[index + degree + 1] - knots[index + 1];
    double right = 0.;
    if (den2 != 0.)
    {
      right =
        ((knots[index + degree + 1] - x) / den2) * bsplineBasis(index + 1, degree - 1, x, knots);
    }

    return left + right;
  }

  double
  bsplineBasisDerivative(int index, int degree, const double x, const Eigen::VectorXd & knots)
  {
    if (degree == 0)
    {
      return 0.;
    }

    const double den1 = knots[index + degree] - knots[index];
    double term1 = 0.;
    if (den1 != 0.)
    {
      term1 = bsplineBasis(index, degree - 1, x, knots) / den1;
    }

    const double den2 = knots[index + degree + 1] - knots[index + 1];
    double term2 = 0.;
    if (den2 != 0.)
    {
      term2 = bsplineBasis(index + 1, degree - 1, x, knots) / den2;
    }

    return degree * (term1 - term2);
  }

  double
  bsplineBasisDerivative2(int index, int degree, const double x, const Eigen::VectorXd & knots)
  {
    if (degree < 2)
    {
      return 0.;
    }

    const double den1 = knots[index + degree] - knots[index];
    double term1 = 0.;
    if (den1 != 0.)
    {
      term1 = bsplineBasisDerivative(index, degree - 1, x, knots) / den1;
    }

    const double den2 = knots[index + degree + 1] - knots[index + 1];
    double term2 = 0.;
    if (den2 != 0.)
    {
      term2 = bsplineBasisDerivative(index + 1, degree - 1, x, knots) / den2;
    }

    return degree * (term1 - term2);
  }

} // namespace

BOOST_AUTO_TEST_SUITE(JointSpline)

/// @brief Check the joint builder and the guards.
BOOST_AUTO_TEST_CASE(jointBuilder)
{
  size_t degree = 3;

  std::vector<SE3> ctrlFrames;
  for (int k = 0; k < 3; k++)
    ctrlFrames.push_back(SE3::Random());

  BOOST_CHECK_THROW(
    JointModelSplineBuilder()
      .withControlFrameVector(ctrlFrames)
      .withDegree(degree)
      .withOpenUniformKnots(0., 1.)
      .build(),
    std::invalid_argument);

  for (int k = 0; k < 3; k++)
    ctrlFrames.push_back(SE3::Random());

  // Knots vector value should not decrease (t_i <= t_{i+1})
  Eigen::VectorXd knots_non_uniform(degree + ctrlFrames.size() + 1);
  knots_non_uniform << 0., 0.1, 0.08, 0.15, 0.15, 0.3, 0.6, 0.6, 0.7, 1.;

  BOOST_CHECK_THROW(
    JointModelSplineBuilder()
      .withControlFrameVector(ctrlFrames)
      .withDegree(degree)
      .withKnotVector(knots_non_uniform)
      .build(),
    std::invalid_argument);

  // Knot vector value should not be repeated more than degree + 1 times
  Eigen::VectorXd knots_too_many_repeats(degree + ctrlFrames.size() + 1);
  knots_too_many_repeats << 0., 0., 0., 0., 0., 0.3, 0.6, 0.6, 0.7, 1.;

  BOOST_CHECK_THROW(
    JointModelSplineBuilder()
      .withControlFrameVector(ctrlFrames)
      .withDegree(degree)
      .withKnotVector(knots_too_many_repeats)
      .build(),
    std::invalid_argument);
}

/// @brief Test on the knot vector generation
BOOST_AUTO_TEST_CASE(makeKnots)
{
  size_t degree = 3;
  size_t nbFrames = 6;
  double min_q = 10;
  double max_q = 40;

  // Open Uniform
  Eigen::VectorXd generated_knots =
    internal::generateOpenUniformKnots(min_q, max_q, nbFrames, degree);

  // Check size
  BOOST_CHECK(generated_knots.size() == static_cast<Eigen::Index>(degree + nbFrames + 1));

  // Check Values
  Eigen::VectorXd open_uniform(degree + nbFrames + 1);
  open_uniform << 10., 10., 10., 10., 20., 30., 40., 40., 40., 40.;
  BOOST_CHECK(generated_knots.isApprox(open_uniform, 1e-5));

  // Open Uniform
  generated_knots = internal::generateUniformKnots(min_q, max_q, nbFrames, degree);

  // Check size
  BOOST_CHECK(generated_knots.size() == static_cast<Eigen::Index>(degree + nbFrames + 1));

  // Check Values
  Eigen::VectorXd uniform(degree + nbFrames + 1);
  uniform << 10., 13.3333, 16.6667, 20.0, 23.3333, 26.6667, 30., 33.3333, 36.6667, 40.;
  BOOST_CHECK(generated_knots.isApprox(uniform, 1e-5));
}

// Test recursive bsplineBasis function.
// This function is only implemented in unit test, but since
// it's used to test deBoor algorithm it worst testing it.
// Test bsplineBasis node limit.
// We test the degree 0 case, because other degree bound depend of it.
BOOST_AUTO_TEST_CASE(basisFunctionsEdge)
{
  int degree = 0;

  Eigen::VectorXd knotVector(6);
  knotVector << 0., 0.2, 0.4, 0.6, 0.8, 1.;
  BOOST_CHECK_EQUAL(bsplineBasis(0, degree, 0., knotVector), 1.);
  BOOST_CHECK_EQUAL(bsplineBasis(0, degree, 0.2, knotVector), 0.);
  BOOST_CHECK_EQUAL(bsplineBasis(1, degree, 0.2, knotVector), 1.);
  BOOST_CHECK_EQUAL(bsplineBasis(1, degree, 0.4, knotVector), 0.);
  BOOST_CHECK_EQUAL(bsplineBasis(2, degree, 0.4, knotVector), 1.);
  BOOST_CHECK_EQUAL(bsplineBasis(2, degree, 0.6, knotVector), 0.);
  BOOST_CHECK_EQUAL(bsplineBasis(3, degree, 0.6, knotVector), 1.);
  BOOST_CHECK_EQUAL(bsplineBasis(3, degree, 0.8, knotVector), 0.);
  BOOST_CHECK_EQUAL(bsplineBasis(4, degree, 0.8, knotVector), 1.);
  BOOST_CHECK_EQUAL(bsplineBasis(4, degree, 1., knotVector), 1.);
}

// Test recursive bsplineBasis, bsplineDerivative and bsplineDerivative2 functions.
// This function is only implemented in unit test, but since
// it's used to test deBoor algorithm it worst testing it.
// Test on an open uniform knot vector.
BOOST_AUTO_TEST_CASE(basisFunctionsOpenUniform)
{
  int degree = 3;
  int nbCtrlFrames = 6;

  double min_q = 0.0;
  double max_q = 1.;

  auto knotVector = internal::generateOpenUniformKnots(
    min_q, max_q, static_cast<size_t>(nbCtrlFrames), static_cast<size_t>(degree));
  Eigen::Index nKnot = knotVector.size();
  Eigen::VectorXd N = Eigen::VectorXd::Zero(nbCtrlFrames);
  Eigen::VectorXd Nder = Eigen::VectorXd::Zero(nbCtrlFrames);
  Eigen::VectorXd Nder2 = Eigen::VectorXd::Zero(nbCtrlFrames);

  // Index Interval for unite partition [degree; nKnot - 1 - degree]
  for (double q = knotVector[degree]; q <= knotVector[nKnot - 1 - degree]; q += 0.02)
  {
    for (int i = 0; i < nbCtrlFrames; i++)
    {
      N[i] = bsplineBasis(i, degree, q, knotVector);
    }
    BOOST_CHECK_CLOSE(N.sum(), 1.0, 1e-8);
  }

  // Check derivatives
  double h = 1e-5;
  for (double q = min_q + h; q < max_q; q += 0.02)
  {
    for (int i = 0; i < nbCtrlFrames; i++)
    {
      N[i] = bsplineBasis(i, degree, q, knotVector);
      Nder[i] = bsplineBasisDerivative(i, degree, q, knotVector);
      Nder2[i] = bsplineBasisDerivative2(i, degree, q, knotVector);

      double n_plus = bsplineBasis(i, degree, q + h, knotVector);
      double n_minus = bsplineBasis(i, degree, q - h, knotVector);

      // First Derivative Approximation
      double numerical_der = (n_plus - n_minus) / (2.0 * h);
      BOOST_CHECK_SMALL(Nder[i] - numerical_der, 1e-5);

      // Second Derivative Approximation
      double n_mid = N[i];
      double numerical_der2 = (n_plus - 2.0 * n_mid + n_minus) / (h * h);
      BOOST_CHECK_SMALL(Nder2[i] - numerical_der2, 1e-5);
    }
  }
}

// Test recursive bsplineBasis, bsplineDerivative and bsplineDerivative2 functions.
// This function is only implemented in unit test, but since
// it's used to test deBoor algorithm it worst testing it.
// Test on an uniform knot vector.
BOOST_AUTO_TEST_CASE(basisFunctionsUniform)
{
  int degree = 3;
  int nbCtrlFrames = 6;
  double min_q = 0.0;
  double max_q = 10.;

  auto knotVector = internal::generateUniformKnots(
    min_q, max_q, static_cast<size_t>(nbCtrlFrames), static_cast<size_t>(degree));
  Eigen::Index nKnot = knotVector.size();

  Eigen::VectorXd N = Eigen::VectorXd::Zero(nbCtrlFrames);
  Eigen::VectorXd Nder = Eigen::VectorXd::Zero(nbCtrlFrames);
  Eigen::VectorXd Nder2 = Eigen::VectorXd::Zero(nbCtrlFrames);
  // Index Interval for unite partition [degree; len(KnotVector) - degree]
  for (double q = knotVector[degree]; q <= knotVector[nKnot - 1 - degree]; q += 0.05)
  {
    for (int i = 0; i < nbCtrlFrames; i++)
    {
      N[i] = bsplineBasis(i, degree, q, knotVector);
    }
    BOOST_CHECK_CLOSE(N.sum(), 1.0, 1e-8);
  }

  // Check derivatives
  double h = 1e-5;
  for (double q = min_q + h; q < max_q; q += 0.02)
  {
    for (int i = 0; i < nbCtrlFrames; i++)
    {
      N[i] = bsplineBasis(i, degree, q, knotVector);
      Nder[i] = bsplineBasisDerivative(i, degree, q, knotVector);
      Nder2[i] = bsplineBasisDerivative2(i, degree, q, knotVector);

      double n_plus = bsplineBasis(i, degree, q + h, knotVector);
      double n_minus = bsplineBasis(i, degree, q - h, knotVector);

      // First Derivative Approximation
      double numerical_der = (n_plus - n_minus) / (2.0 * h);
      BOOST_CHECK_SMALL(Nder[i] - numerical_der, 1e-5);

      // Second Derivative Approximation
      double n_mid = N[i];
      double numerical_der2 = (n_plus - 2.0 * n_mid + n_minus) / (h * h);
      BOOST_CHECK_SMALL(Nder2[i] - numerical_der2, 1e-5);
    }
  }
}

// Test recursive bsplineBasis, bsplineDerivative and bsplineDerivative2 functions.
// This function is only implemented in unit test, but since
// it's used to test deBoor algorithm it worst testing it.
// Test on a non uniform knot vector.
BOOST_AUTO_TEST_CASE(basisFunctionsNonUniform)
{
  int degree = 3;
  int nbCtrlFrames = 5;
  double min_q = 0.;
  double max_q = 1.0;
  Eigen::Index nKnot = degree + nbCtrlFrames + 1;
  Eigen::VectorXd knotVector(nKnot);
  knotVector << min_q, 0.1, 0.1, 0.25, 0.5, 0.5, 0.8, 0.8, max_q;

  Eigen::VectorXd N = Eigen::VectorXd::Zero(nbCtrlFrames);
  Eigen::VectorXd Nder = Eigen::VectorXd::Zero(nbCtrlFrames);
  Eigen::VectorXd Nder2 = Eigen::VectorXd::Zero(nbCtrlFrames);

  // Index Interval for unite partition [degree; len(KnotVector) -1 - degree]
  for (double q = knotVector[degree]; q <= knotVector[nKnot - 1 - degree]; q += 0.05)
  {
    for (int i = 0; i < nbCtrlFrames; i++)
    {
      N[i] = bsplineBasis(i, degree, q, knotVector);
    }
    BOOST_CHECK_CLOSE(N.sum(), 1.0, 1e-8);
  }

  // Check derivatives
  double h = 1e-5;
  for (double q = min_q + h; q < max_q; q += 0.02)
  {
    for (int i = 0; i < nbCtrlFrames; i++)
    {
      N[i] = bsplineBasis(i, degree, q, knotVector);
      Nder[i] = bsplineBasisDerivative(i, degree, q, knotVector);
      Nder2[i] = bsplineBasisDerivative2(i, degree, q, knotVector);

      double n_plus = bsplineBasis(i, degree, q + h, knotVector);
      double n_minus = bsplineBasis(i, degree, q - h, knotVector);

      // First Derivative Approximation
      double numerical_der = (n_plus - n_minus) / (2.0 * h);
      BOOST_CHECK_SMALL(Nder[i] - numerical_der, 1e-5);

      // Second Derivative is not checked because the knot vector is not C2 everywhere
    }
  }
}
// Test degree 0 deBoorBasis.
BOOST_AUTO_TEST_CASE(deBoorBasis_degree0)
{
  using pinocchio::internal::deBoorBasis;

  const int degree = 0;
  Eigen::VectorXd knots(3);
  knots << 0., 2., 3.;
  Eigen::MatrixXd basis(1, 1);

  deBoorBasis(degree, knots, 0, 0., basis);
  BOOST_CHECK_SMALL(basis(0, 0) - 1., 1e-8);
  deBoorBasis(degree, knots, 0, 1., basis);
  BOOST_CHECK_SMALL(basis(0, 0) - 1., 1e-8);
  deBoorBasis(degree, knots, 0, 1.5, basis);
  BOOST_CHECK_SMALL(basis(0, 0) - 1., 1e-8);
  deBoorBasis(degree, knots, 1, 2., basis);
  BOOST_CHECK_SMALL(basis(0, 0) - 1., 1e-8);
  deBoorBasis(degree, knots, 1, 2.5, basis);
  BOOST_CHECK_SMALL(basis(0, 0) - 1., 1e-8);
  deBoorBasis(degree, knots, 1, 3., basis);
  BOOST_CHECK_SMALL(basis(0, 0) - 1., 1e-8);
}

// Test degree 1 deBoorBasis.
BOOST_AUTO_TEST_CASE(deBoorBasis_degree1)
{
  using pinocchio::internal::deBoorBasis;

  const int degree = 1;
  Eigen::VectorXd knots(5);
  knots << 0., 2., 3., 5., 5.5;
  Eigen::MatrixXd basis(2, 2);

  // Evaluate between 2 and 3
  deBoorBasis(degree, knots, 1, 2., basis);
  BOOST_CHECK_SMALL(basis(1, 0) - 1., 1e-8);
  BOOST_CHECK_SMALL(basis(1, 1) - 0., 1e-8);
  deBoorBasis(degree, knots, 1, 2.5, basis);
  BOOST_CHECK_SMALL(basis(1, 0) - 0.5, 1e-8);
  BOOST_CHECK_SMALL(basis(1, 1) - 0.5, 1e-8);
  deBoorBasis(degree, knots, 1, 3., basis);
  BOOST_CHECK_SMALL(basis(1, 0) - 0., 1e-8);
  BOOST_CHECK_SMALL(basis(1, 1) - 1., 1e-8);

  // Evaluate between 3 and 5
  deBoorBasis(degree, knots, 2, 3., basis);
  BOOST_CHECK_SMALL(basis(1, 0) - 1., 1e-8);
  BOOST_CHECK_SMALL(basis(1, 1) - 0., 1e-8);
  deBoorBasis(degree, knots, 2, 4., basis);
  BOOST_CHECK_SMALL(basis(1, 0) - 0.5, 1e-8);
  BOOST_CHECK_SMALL(basis(1, 1) - 0.5, 1e-8);
  deBoorBasis(degree, knots, 2, 5., basis);
  BOOST_CHECK_SMALL(basis(1, 0) - 0., 1e-8);
  BOOST_CHECK_SMALL(basis(1, 1) - 1., 1e-8);
}

// Test degree 3 deBoorBasis against bsplineBasis.
BOOST_AUTO_TEST_CASE(deBoorBasis_degree3)
{
  using pinocchio::internal::deBoorBasis;
  using pinocchio::internal::getAbsoluteBasis;

  const int degree = 3;
  Eigen::VectorXd knots(9);
  knots << 0., 2., 3., 5., 5.5, 8., 8.5, 10., 11.5;
  Eigen::MatrixXd basis(4, 4);

  deBoorBasis(degree, knots, 3, 5., basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 0, degree) - bsplineBasis(0, degree, 5., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 1, degree) - bsplineBasis(1, degree, 5., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 2, degree) - bsplineBasis(2, degree, 5., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 3, degree) - bsplineBasis(3, degree, 5., knots), 1e-8);

  deBoorBasis(degree, knots, 3, 5.3, basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 0, degree) - bsplineBasis(0, degree, 5.3, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 1, degree) - bsplineBasis(1, degree, 5.3, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 2, degree) - bsplineBasis(2, degree, 5.3, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 3, degree) - bsplineBasis(3, degree, 5.3, knots), 1e-8);

  deBoorBasis(degree, knots, 3, 5.5, basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 0, degree) - bsplineBasis(0, degree, 5.5, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 1, degree) - bsplineBasis(1, degree, 5.5, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 2, degree) - bsplineBasis(2, degree, 5.5, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(3, basis, 3, degree) - bsplineBasis(3, degree, 5.5, knots), 1e-8);

  deBoorBasis(degree, knots, 4, 5.5, basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 1, degree) - bsplineBasis(1, degree, 5.5, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 2, degree) - bsplineBasis(2, degree, 5.5, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 3, degree) - bsplineBasis(3, degree, 5.5, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 4, degree) - bsplineBasis(4, degree, 5.5, knots), 1e-8);

  deBoorBasis(degree, knots, 4, 6., basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 1, degree) - bsplineBasis(1, degree, 6., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 2, degree) - bsplineBasis(2, degree, 6., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 3, degree) - bsplineBasis(3, degree, 6., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 4, degree) - bsplineBasis(4, degree, 6., knots), 1e-8);

  deBoorBasis(degree, knots, 4, 7.3, basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 1, degree) - bsplineBasis(1, degree, 7.3, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 2, degree) - bsplineBasis(2, degree, 7.3, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 3, degree) - bsplineBasis(3, degree, 7.3, knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 4, degree) - bsplineBasis(4, degree, 7.3, knots), 1e-8);

  deBoorBasis(degree, knots, 4, 8., basis);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 1, degree) - bsplineBasis(1, degree, 8., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 2, degree) - bsplineBasis(2, degree, 8., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 3, degree) - bsplineBasis(3, degree, 8., knots), 1e-8);
  BOOST_CHECK_SMALL(
    getAbsoluteBasis(4, basis, 4, degree) - bsplineBasis(4, degree, 8., knots), 1e-8);
}

// Test cumulativeBasisDerivative against bsplineBasisDerivative
BOOST_AUTO_TEST_CASE(cumulativeBasisDerivative)
{
  using pinocchio::internal::cumulativeBasisDerivative;
  using pinocchio::internal::deBoorBasis;

  const int degree = 3;
  Eigen::VectorXd knots(9);
  knots << 0., 2., 3., 5., 5.5, 8., 8.5, 10., 11.5;
  Eigen::MatrixXd basis(4, 4);

  auto computeDerivative = [knots](int start, double q) {
    Eigen::VectorXd res(4);
    for (int i = 0; i < 4; ++i)
    {
      res(i) = bsplineBasisDerivative(i + start, degree, q, knots);
    }
    return res;
  };

  auto derivative_ref = computeDerivative(0, 5.);
  deBoorBasis(degree, knots, 3, 5., basis);
  BOOST_CHECK_SMALL(derivative_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(3).sum() - cumulativeBasisDerivative(3, knots, basis, 1, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(2).sum() - cumulativeBasisDerivative(3, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(1).sum() - cumulativeBasisDerivative(3, knots, basis, 3, degree), 1e-8);

  derivative_ref = computeDerivative(0, 5.3);
  deBoorBasis(degree, knots, 3, 5.3, basis);
  BOOST_CHECK_SMALL(derivative_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(3).sum() - cumulativeBasisDerivative(3, knots, basis, 1, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(2).sum() - cumulativeBasisDerivative(3, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(1).sum() - cumulativeBasisDerivative(3, knots, basis, 3, degree), 1e-8);

  derivative_ref = computeDerivative(1, 5.5);
  deBoorBasis(degree, knots, 4, 5.5, basis);
  BOOST_CHECK_SMALL(derivative_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(3).sum() - cumulativeBasisDerivative(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(2).sum() - cumulativeBasisDerivative(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(1).sum() - cumulativeBasisDerivative(4, knots, basis, 4, degree), 1e-8);

  derivative_ref = computeDerivative(1, 6.);
  deBoorBasis(degree, knots, 4, 6., basis);
  BOOST_CHECK_SMALL(derivative_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(3).sum() - cumulativeBasisDerivative(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(2).sum() - cumulativeBasisDerivative(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(1).sum() - cumulativeBasisDerivative(4, knots, basis, 4, degree), 1e-8);

  derivative_ref = computeDerivative(1, 7.3);
  deBoorBasis(degree, knots, 4, 7.3, basis);
  BOOST_CHECK_SMALL(derivative_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(3).sum() - cumulativeBasisDerivative(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(2).sum() - cumulativeBasisDerivative(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(1).sum() - cumulativeBasisDerivative(4, knots, basis, 4, degree), 1e-8);

  derivative_ref = computeDerivative(1, 8.);
  deBoorBasis(degree, knots, 4, 8., basis);
  BOOST_CHECK_SMALL(derivative_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(3).sum() - cumulativeBasisDerivative(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(2).sum() - cumulativeBasisDerivative(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative_ref.tail(1).sum() - cumulativeBasisDerivative(4, knots, basis, 4, degree), 1e-8);
}

// Test cumulativeBasisDerivative2 against bsplineBasisDerivative2
BOOST_AUTO_TEST_CASE(cumulativeBasisDerivative2)
{
  using pinocchio::internal::cumulativeBasisDerivative2;
  using pinocchio::internal::deBoorBasis;

  const int degree = 3;
  Eigen::VectorXd knots(9);
  knots << 0., 2., 3., 5., 5.5, 8., 8.5, 10., 11.5;
  Eigen::MatrixXd basis(4, 4);

  auto computeDerivative2 = [knots](int start, double q) {
    Eigen::VectorXd res(4);
    for (int i = 0; i < 4; ++i)
    {
      res(i) = bsplineBasisDerivative2(i + start, degree, q, knots);
    }
    return res;
  };

  auto derivative2_ref = computeDerivative2(0, 5.);
  deBoorBasis(degree, knots, 3, 5., basis);
  BOOST_CHECK_SMALL(derivative2_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(3).sum() - cumulativeBasisDerivative2(3, knots, basis, 1, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(2).sum() - cumulativeBasisDerivative2(3, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(1).sum() - cumulativeBasisDerivative2(3, knots, basis, 3, degree), 1e-8);

  derivative2_ref = computeDerivative2(0, 5.3);
  deBoorBasis(degree, knots, 3, 5.3, basis);
  BOOST_CHECK_SMALL(derivative2_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(3).sum() - cumulativeBasisDerivative2(3, knots, basis, 1, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(2).sum() - cumulativeBasisDerivative2(3, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(1).sum() - cumulativeBasisDerivative2(3, knots, basis, 3, degree), 1e-8);

  derivative2_ref = computeDerivative2(1, 5.5);
  deBoorBasis(degree, knots, 4, 5.5, basis);
  BOOST_CHECK_SMALL(derivative2_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(3).sum() - cumulativeBasisDerivative2(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(2).sum() - cumulativeBasisDerivative2(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(1).sum() - cumulativeBasisDerivative2(4, knots, basis, 4, degree), 1e-8);

  derivative2_ref = computeDerivative2(1, 6.);
  deBoorBasis(degree, knots, 4, 6., basis);
  BOOST_CHECK_SMALL(derivative2_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(3).sum() - cumulativeBasisDerivative2(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(2).sum() - cumulativeBasisDerivative2(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(1).sum() - cumulativeBasisDerivative2(4, knots, basis, 4, degree), 1e-8);

  derivative2_ref = computeDerivative2(1, 7.3);
  deBoorBasis(degree, knots, 4, 7.3, basis);
  BOOST_CHECK_SMALL(derivative2_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(3).sum() - cumulativeBasisDerivative2(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(2).sum() - cumulativeBasisDerivative2(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(1).sum() - cumulativeBasisDerivative2(4, knots, basis, 4, degree), 1e-8);

  derivative2_ref = computeDerivative2(1, 8.);
  deBoorBasis(degree, knots, 4, 8., basis);
  BOOST_CHECK_SMALL(derivative2_ref.sum(), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(3).sum() - cumulativeBasisDerivative2(4, knots, basis, 2, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(2).sum() - cumulativeBasisDerivative2(4, knots, basis, 3, degree), 1e-8);
  BOOST_CHECK_SMALL(
    derivative2_ref.tail(1).sum() - cumulativeBasisDerivative2(4, knots, basis, 4, degree), 1e-8);
}

/// @brief Test to make sure the relative motions are correct
BOOST_AUTO_TEST_CASE(relativeMotions)
{
  size_t degree = 3;

  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  std::vector<Motion> relativeMotions;
  for (int i = 0; i < 10; i++)
  {
    relativeMotions.push_back(Motion::Random());
    const SE3 & currentFrame = ctrlFrames.back();
    ctrlFrames.push_back(currentFrame * exp6(relativeMotions.back()));
  }

  auto jmodel = JointModelSplineBuilder()
                  .withControlFrameVector(ctrlFrames)
                  .withDegree(degree)
                  .withOpenUniformKnots(0., 1.)
                  .build();
  // Check size
  BOOST_CHECK(jmodel.relativeMotions.size() == (ctrlFrames.size() - 1));

  // check values
  for (size_t i = 0; i < ctrlFrames.size() - 1; i++)
    BOOST_CHECK(jmodel.relativeMotions[i].isApprox(relativeMotions[i]));
}

/// @brief Test FindSpan on the simplest case (no redundant knot vector).
BOOST_AUTO_TEST_CASE(findSpan_degree_0)
{
  const int degree = 0;
  Eigen::VectorXd knotVector(6);
  knotVector << 0., 0.2, 0.4, 0.6, 0.8, 1.;
  internal::SpanIndexes indexes;

  indexes = internal::FindSpan<double>::run(0., degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 1);

  indexes = internal::FindSpan<double>::run(0.1, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 1);

  indexes = internal::FindSpan<double>::run(0.2, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 1);
  BOOST_CHECK(indexes.end_idx == 2);

  indexes = internal::FindSpan<double>::run(0.5, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 2);
  BOOST_CHECK(indexes.end_idx == 3);

  indexes = internal::FindSpan<double>::run(0.7, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 3);
  BOOST_CHECK(indexes.end_idx == 4);

  indexes = internal::FindSpan<double>::run(0.8, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);

  indexes = internal::FindSpan<double>::run(0.9, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);

  indexes = internal::FindSpan<double>::run(1., degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);
}

/// @brief Test FindSpan with redundant knot vector values.
BOOST_AUTO_TEST_CASE(findSpan_degree_1)
{
  const int degree = 1;
  Eigen::VectorXd knotVector(8);
  knotVector << 0., 0., 0.2, 0.6, 0.6, 0.8, 1., 1.;
  internal::SpanIndexes indexes;

  indexes = internal::FindSpan<double>::run(0., degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 1);
  BOOST_CHECK(indexes.end_idx == 2);

  indexes = internal::FindSpan<double>::run(0.1, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 1);
  BOOST_CHECK(indexes.end_idx == 2);

  indexes = internal::FindSpan<double>::run(0.2, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 2);
  BOOST_CHECK(indexes.end_idx == 3);

  indexes = internal::FindSpan<double>::run(0.5, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 2);
  BOOST_CHECK(indexes.end_idx == 3);

  indexes = internal::FindSpan<double>::run(0.6, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);

  indexes = internal::FindSpan<double>::run(0.7, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);

  indexes = internal::FindSpan<double>::run(0.8, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 5);
  BOOST_CHECK(indexes.end_idx == 6);

  indexes = internal::FindSpan<double>::run(0.9, degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 5);
  BOOST_CHECK(indexes.end_idx == 6);

  indexes = internal::FindSpan<double>::run(1., degree, knotVector);
  BOOST_CHECK(indexes.start_idx == 5);
  BOOST_CHECK(indexes.end_idx == 6);
}

/// @brief Comparing a simple spline joint with a PZ
/// Make sure pose and joint subspace are the same
BOOST_AUTO_TEST_CASE(vsPrismaticZ)
{
  using namespace pinocchio;

  // Spline Joint
  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)));

  auto jmodel = JointModelSplineBuilder()
                  .withControlFrameVector(ctrlFrames)
                  .withDegree(1)
                  .withOpenUniformKnots(0., 1.)
                  .build();

  JointDataSpline jdata = jmodel.createData();
  jmodel.setIndexes(0, 0, 0);

  // Prismatic joint
  JointModelPZ jmodelPz;
  JointDataPZ jdataPz = jmodelPz.createData();
  jmodelPz.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));

  // -------
  q << 0.2;

  jmodel.calc(jdata, q);
  jmodelPz.calc(jdataPz, q);

  BOOST_CHECK(jdata.M.isApprox(jdataPz.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataPz.S.matrix(), 1e-12));

  // -------
  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));
  q << 0.3;
  q_dot << 0.4;

  jmodel.calc(jdata, q, q_dot);
  jmodelPz.calc(jdataPz, q, q_dot);

  BOOST_CHECK(jdata.M.isApprox(jdataPz.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataPz.S.matrix(), 1e-12));
  BOOST_CHECK(jdata.v.isApprox(jdataPz.v, 1e-12));
  BOOST_CHECK(jdata.c.isApprox(jdataPz.c, 1e-12));
}

/// @brief Comparing a simple spline joint with a RX
/// Make sure pose and joint subspace are the same
BOOST_AUTO_TEST_CASE(vsRevoluteX)
{
  using namespace pinocchio;

  // Spline Joint
  Eigen::Matrix3d rotation;
  std::vector<SE3> ctrlFrames;
  Eigen::AngleAxisd Rx(1, Eigen::Vector3d::UnitX());

  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3(Rx.toRotationMatrix(), Eigen::Vector3d(0., 0., 0.)));

  auto jmodel = JointModelSplineBuilder()
                  .withControlFrameVector(ctrlFrames)
                  .withDegree(1)
                  .withOpenUniformKnots(0., 1.)
                  .build();
  JointDataSpline jdata = jmodel.createData();
  jmodel.setIndexes(0, 0, 0);

  // Prismatic joint
  JointModelRX jmodelRx;
  JointDataRX jdataRx = jmodelRx.createData();
  jmodelRx.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));

  // -------
  q << 0.2;

  jmodel.calc(jdata, q);
  jmodelRx.calc(jdataRx, q);

  BOOST_CHECK(jdata.M.isApprox(jdataRx.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataRx.S.matrix(), 1e-12));

  // -------
  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));
  q << 0.3;
  q_dot << 0.4;

  jmodel.calc(jdata, q, q_dot);
  jmodelRx.calc(jdataRx, q, q_dot);

  BOOST_CHECK(jdata.M.isApprox(jdataRx.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataRx.S.matrix(), 1e-12));
  BOOST_CHECK(jdata.v.isApprox(jdataRx.v, 1e-12));
  BOOST_CHECK(jdata.c.isApprox(jdataRx.c, 1e-12));
}

/// @brief Test out rnea vs aba
BOOST_AUTO_TEST_CASE(abaVSrnea)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model modelSpline;

  Inertia inertia(1., Vector3(0., 0., 0.0), Matrix3::Identity());

  std::vector<SE3> ctrlFrames;
  getTrajectory(ctrlFrames);
  addJointAndBody(
    modelSpline,
    JointModelSplineBuilder()
      .withControlFrameVector(ctrlFrames)
      .withDegree(3)
      .withOpenUniformKnots(0., 1.)
      .build(),
    0, SE3::Identity(), "kneeSpline", inertia);
  Data dataSplineRnea(modelSpline);
  Data dataSplineAba(modelSpline);

  Eigen::VectorXd q(modelSpline.nq);
  pinocchio::randomConfiguration(
    modelSpline, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1), q);
  Eigen::VectorXd vq(Eigen::VectorXd::Random(modelSpline.nv));
  Eigen::VectorXd aq(Eigen::VectorXd::Random(modelSpline.nv));
  Eigen::VectorXd tauRnea(1);
  Eigen::VectorXd aAba(1);

  tauRnea = pinocchio::rnea(modelSpline, dataSplineRnea, q, vq, aq);
  aAba = pinocchio::aba(modelSpline, dataSplineAba, q, vq, tauRnea);

  BOOST_CHECK(aq.isApprox(aAba));
}

/// @brief Test S and bias c computation via finite differences
BOOST_AUTO_TEST_CASE(vsFiniteDifference)
{
  using namespace pinocchio;

  typedef typename JointModelSpline::ConfigVector_t CV;
  typedef typename JointModelSpline::TangentVector_t TV;
  typedef typename LieGroup<JointModelSpline>::type LieGroupType;

  std::vector<SE3> ctrlFrames;
  getTrajectory(ctrlFrames);

  auto jmodel = JointModelSplineBuilder()
                  .withControlFrameVector(ctrlFrames)
                  .withDegree(3)
                  .withOpenUniformKnots(0., 1.)
                  .build();
  JointDataSpline jdata = jmodel.createData();

  jmodel.setIndexes(0, 0, 0);

  double eps = 1e-8;
  CV q_ref(1);
  q_ref[0] = 0.6;
  CV q(q_ref);

  const Eigen::DenseIndex nv = jdata.S.nv();
  TV q_dot(nv);
  TV q_dot_ref(nv);
  q_dot.setZero();

  q_dot[0] = eps;
  q_dot_ref[0] = 0.3;

  q = LieGroupType().integrate(q_ref, q_dot);
  // Check S
  {
    jmodel.calc(jdata, q_ref);
    SE3 M_ref(jdata.M);
    Eigen::Matrix<double, 6, JointModelSpline::NV> S(6, JointModelSpline::NV),
      S_ref(jdata.S.matrix());

    jmodel.calc(jdata, q);
    SE3 M_ = jdata.M;

    S.col(0) = log6(M_ref.inverse() * M_).toVector();
    S.col(0) /= eps;

    BOOST_CHECK(S.isApprox(S_ref, 1e-6));
  }
  // Check bias
  {
    jmodel.calc(jdata, q_ref, q_dot_ref);
    const Motion & c_ref = jdata.c;
    Eigen::Matrix<double, 6, JointModelSpline::NV> S_ref(jdata.S.matrix());

    jmodel.calc(jdata, q);
    Eigen::Matrix<double, 6, JointModelSpline::NV> S_(jdata.S.matrix());

    Motion dSdq_fd((S_ - S_ref) / eps);
    Motion c_fd = dSdq_fd * q_dot_ref[0] * q_dot_ref[0];

    BOOST_CHECK(c_ref.isApprox(c_fd, 1e-6));
  }
}

BOOST_AUTO_TEST_SUITE_END()
