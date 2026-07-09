//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/spatial.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

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

// Test bsplineBasis node limit.
// We test the degree 0 case, because other degree bound depend of it.
BOOST_AUTO_TEST_CASE(basisFunctionsEdge)
{
  size_t degree = 0;
  size_t nbCtrlFrames = 5;

  Eigen::VectorXd knotVector((degree + 1) + nbCtrlFrames);
  knotVector << 0., 0.2, 0.4, 0.6, 0.8, 1.;
  BOOST_CHECK_EQUAL(internal::bsplineBasis(0, degree, 0., knotVector), 1.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(0, degree, 0.2, knotVector), 0.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(1, degree, 0.2, knotVector), 1.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(1, degree, 0.4, knotVector), 0.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(2, degree, 0.4, knotVector), 1.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(2, degree, 0.6, knotVector), 0.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(3, degree, 0.6, knotVector), 1.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(3, degree, 0.8, knotVector), 0.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(4, degree, 0.8, knotVector), 1.);
  BOOST_CHECK_EQUAL(internal::bsplineBasis(4, degree, 1., knotVector), 1.);
}

BOOST_AUTO_TEST_CASE(basisFunctionsOpenUniform)
{
  Eigen::Index degree = 3;
  Eigen::Index nbCtrlFrames = 6;

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
    for (Eigen::Index i = 0; i < nbCtrlFrames; i++)
    {
      N[i] =
        internal::bsplineBasis(static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
    }
    BOOST_CHECK_CLOSE(N.sum(), 1.0, 1e-8);
  }
  // Check derivatives
  double h = 1e-5;
  for (double q = min_q + h; q < max_q; q += 0.02)
  {
    for (Eigen::Index i = 0; i < nbCtrlFrames; i++)
    {
      N[i] =
        internal::bsplineBasis(static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
      Nder[i] = internal::bsplineBasisDerivative(
        static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
      Nder2[i] = internal::bsplineBasisDerivative2(
        static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);

      double n_plus = internal::bsplineBasis(
        static_cast<size_t>(i), static_cast<size_t>(degree), q + h, knotVector);
      double n_minus = internal::bsplineBasis(
        static_cast<size_t>(i), static_cast<size_t>(degree), q - h, knotVector);

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

BOOST_AUTO_TEST_CASE(basisFunctionsUniform)
{
  Eigen::Index degree = 3;
  Eigen::Index nbCtrlFrames = 6;
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
    for (Eigen::Index i = 0; i < nbCtrlFrames; i++)
    {
      N[i] =
        internal::bsplineBasis(static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
    }
    BOOST_CHECK_CLOSE(N.sum(), 1.0, 1e-8);
  }

  // Check derivatives
  double h = 1e-5;
  for (double q = min_q + h; q < max_q; q += 0.02)
  {
    for (Eigen::Index i = 0; i < nbCtrlFrames; i++)
    {
      N[i] =
        internal::bsplineBasis(static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
      Nder[i] = internal::bsplineBasisDerivative(
        static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
      Nder2[i] = internal::bsplineBasisDerivative2(
        static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);

      double n_plus = internal::bsplineBasis(
        static_cast<size_t>(i), static_cast<size_t>(degree), q + h, knotVector);
      double n_minus = internal::bsplineBasis(
        static_cast<size_t>(i), static_cast<size_t>(degree), q - h, knotVector);

      // First Derivative Approximation
      double numerical_der = (n_plus - n_minus) / (2.0 * h);
      BOOST_CHECK_SMALL(Nder[i] - numerical_der, 1e-5);

      // Second Derivative Approximation
      double n_mid = N[i];
      double numerical_der2 = (n_plus - 2.0 * n_mid + n_minus) / (h * h);
      BOOST_CHECK_SMALL(Nder2[i] - numerical_der2, 1e-3);
    }
  }
}

BOOST_AUTO_TEST_CASE(basisFunctionsNonUniform)
{
  Eigen::Index degree = 3;
  Eigen::Index nbCtrlFrames = 5;
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
    for (Eigen::Index i = 0; i < nbCtrlFrames; i++)
    {
      N[i] =
        internal::bsplineBasis(static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
    }
    BOOST_CHECK_CLOSE(N.sum(), 1.0, 1e-8);
  }

  // Check derivatives
  double h = 1e-5;
  for (double q = min_q + h; q < max_q; q += 0.02)
  {
    for (Eigen::Index i = 0; i < nbCtrlFrames; i++)
    {
      N[i] =
        internal::bsplineBasis(static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
      Nder[i] = internal::bsplineBasisDerivative(
        static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);
      Nder2[i] = internal::bsplineBasisDerivative2(
        static_cast<size_t>(i), static_cast<size_t>(degree), q, knotVector);

      double n_plus = internal::bsplineBasis(
        static_cast<size_t>(i), static_cast<size_t>(degree), q + h, knotVector);
      double n_minus = internal::bsplineBasis(
        static_cast<size_t>(i), static_cast<size_t>(degree), q - h, knotVector);

      // First Derivative Approximation
      double numerical_der = (n_plus - n_minus) / (2.0 * h);
      BOOST_CHECK_SMALL(Nder[i] - numerical_der, 1e-5);

      // Second Derivative is not checked because the knot vector is not C2 everywhere
    }
  }
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

/// @brief Test the spanning function
BOOST_AUTO_TEST_CASE(findSpan_degree_0)
{
  size_t degree = 0;
  size_t nbCtrlFrames = 5;

  Eigen::VectorXd knotVector((degree + 1) + nbCtrlFrames);
  knotVector << 0., 0.2, 0.4, 0.6, 0.8, 1.;
  Eigen::VectorXd q(1);
  internal::SpanIndexes indexes;

  // N_{i, k}, with i the control point and k the order (degree + 1)
  // Evaluate:
  //  - N_{0, 1}: [t_0(0),   t_1(0.2))
  //  - N_{1, 1}: [t_1(0.2), t_2(0.4))
  //  - N_{2, 1}: [t_2(0.4), t_3(0.6))
  //  - N_{3, 1}: [t_3(0.6), t_4(0.8))
  //  - N_{4, 1}: [t_4(0.8), t_5(1))
  q << 0.1;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // Evaluate N_{0, 1}
  // t_0(0) <= t < t_1(0.2)
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 1);

  q << 0.2;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // Evaluate N_{1, 1}
  // t_1(0.2) <= t < t_2(0.4)
  BOOST_CHECK(indexes.start_idx == 1);
  BOOST_CHECK(indexes.end_idx == 2);

  q << 0.5;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // Evaluate N_{2, 1}
  // t_2(0.4) <= t < t_3(0.6)
  BOOST_CHECK(indexes.start_idx == 2);
  BOOST_CHECK(indexes.end_idx == 3);

  q << 0.7;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // Evaluate N_{3, 1}
  // t_3(0.6) <= t < t_4(0.8)
  BOOST_CHECK(indexes.start_idx == 3);
  BOOST_CHECK(indexes.end_idx == 4);

  q << 0.8;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // Evaluate N_{4, 1}
  // t_4(0.8) <= t < t_5(1)
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);

  q << 0.9;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // Evaluate N_{4, 1}
  // t_4(0.8) <= t < t_5(1)
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);

  q << 0.0;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // 0 edge case
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 1);

  q << 1.0;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  // 1 edge case
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 5);
}

BOOST_AUTO_TEST_CASE(findSpan_degree_1)
{
  size_t degree = 1;
  size_t nbCtrlFrames = 4;

  Eigen::VectorXd knotVector((degree + 1) + nbCtrlFrames);
  knotVector << 0., 0.2, 0.4, 0.6, 0.8, 1.;
  Eigen::VectorXd q(1);
  internal::SpanIndexes indexes;

  // N_{i, k}, with i the control point and k the order (degree + 1)
  // Evaluate:
  //  - N_{0, 2}: [(t_0(0),   t_2(0.4))
  //  - N_{1, 2}: [(t_1(0.2), t_3(0.6))
  //  - N_{2, 2}: [(t_2(0.4), t_4(0.8))
  //  - N_{3, 2}: [(t_3(0.6), t_5(1))
  q << 0.1;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 1);

  q << 0.2;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 2);

  q << 0.5;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 1);
  BOOST_CHECK(indexes.end_idx == 3);

  q << 0.7;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 2);
  BOOST_CHECK(indexes.end_idx == 4);

  q << 0.8;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 3);
  BOOST_CHECK(indexes.end_idx == 4);

  q << 0.9;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 3);
  BOOST_CHECK(indexes.end_idx == 4);
}

BOOST_AUTO_TEST_CASE(findSpan_degree_1_open_non_uniform)
{
  size_t degree = 1;
  size_t nbCtrlFrames = 7;

  Eigen::VectorXd knotVector((degree + 1) + nbCtrlFrames);
  knotVector << 0., 0., 0.2, 0.4, 0.4, 0.6, 0.8, 1., 1.;
  Eigen::VectorXd q(1);
  internal::SpanIndexes indexes;

  // N_{i, k}, with i the control point and k the order (degree + 1)
  // Evaluate:
  //  - N_{0, 2}: [(t_0(0),   t_2(0.2))
  //  - N_{1, 2}: [(t_1(0),   t_3(0.4))
  //  - N_{2, 2}: [(t_2(0.2), t_4(0.4))
  //  - N_{3, 2}: [(t_3(0.4), t_5(0.6))
  //  - N_{4, 2}: [(t_4(0.4), t_6(0.8))
  //  - N_{5, 2}: [(t_5(0.6), t_7(1))
  //  - N_{6, 2}: [(t_6(0.8), t_8(1))
  q << 0.1;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 0);
  BOOST_CHECK(indexes.end_idx == 2);

  q << 0.2;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 1);
  BOOST_CHECK(indexes.end_idx == 3);

  q << 0.5;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 3);
  BOOST_CHECK(indexes.end_idx == 5);

  q << 0.7;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 4);
  BOOST_CHECK(indexes.end_idx == 6);

  q << 0.8;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 5);
  BOOST_CHECK(indexes.end_idx == 7);

  q << 0.9;
  indexes = internal::FindSpan<double, 0>::run(q, degree, nbCtrlFrames, knotVector);
  BOOST_CHECK(indexes.start_idx == 5);
  BOOST_CHECK(indexes.end_idx == 7);
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

// Test deBoorBasis with degree 0.
BOOST_AUTO_TEST_CASE(deBoorBasis_degree0)
{
  using pinocchio::internal::deBoor;

  const size_t degree = 0;
  Eigen::VectorXd knots(3);
  knots << 0., 2., 3.;
  Eigen::VectorXd control_points(2);
  control_points << 5., 0.;
  Eigen::VectorXd workspace(1);

  BOOST_CHECK_SMALL(deBoor(0, degree, 0., knots, control_points, workspace) - 5., 1e-8);
  BOOST_CHECK_SMALL(deBoor(0, degree, 1., knots, control_points, workspace) - 5., 1e-8);
  BOOST_CHECK_SMALL(deBoor(0, degree, 1.5, knots, control_points, workspace) - 5., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 2., knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 2.5, knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 3., knots, control_points, workspace) - 0., 1e-8);

  control_points << 0., 7.;
  BOOST_CHECK_SMALL(deBoor(0, degree, 0., knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(0, degree, 1., knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(0, degree, 1.5, knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 2., knots, control_points, workspace) - 7., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 2.5, knots, control_points, workspace) - 7., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 3., knots, control_points, workspace) - 7., 1e-8);
}

// Test deBoorBasis with degree 1.
BOOST_AUTO_TEST_CASE(deBoorBasis_degree1)
{
  using pinocchio::internal::deBoor;

  const size_t degree = 1;
  Eigen::VectorXd knots(5);
  knots << 0., 2., 3., 5., 5.5;
  Eigen::VectorXd control_points(3);
  control_points << 2., 0., 0.;
  Eigen::VectorXd workspace(2);

  BOOST_CHECK_SMALL(
    deBoor(1, degree, 2., knots, control_points, workspace) - (1. * 2. + 0. * 0.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(1, degree, 2.5, knots, control_points, workspace) - (0.5 * 2. + 0.5 * 0.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(1, degree, 3., knots, control_points, workspace) - (0. * 2. + 1. * 0.), 1e-8);
  BOOST_CHECK_SMALL(deBoor(2, degree, 3.5, knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(2, degree, 4., knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(2, degree, 5., knots, control_points, workspace) - 0., 1e-8);

  control_points << 0., 3., 0.;
  BOOST_CHECK_SMALL(
    deBoor(1, degree, 2., knots, control_points, workspace) - (1. * 0. + 0. * 3.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(1, degree, 2.5, knots, control_points, workspace) - (0.5 * 0. + 0.5 * 3.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(1, degree, 3., knots, control_points, workspace) - (0. * 0. + 1. * 3.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(2, degree, 3.5, knots, control_points, workspace) - (0.75 * 3. + 0.25 * 0.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(2, degree, 4., knots, control_points, workspace) - (0.5 * 3. + .5 * 0.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(2, degree, 5., knots, control_points, workspace) - (0. * 3. + 1. * 0.), 1e-8);

  control_points << 0., 0., 4.;
  BOOST_CHECK_SMALL(deBoor(1, degree, 2., knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 2.5, knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(deBoor(1, degree, 3., knots, control_points, workspace) - 0., 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(2, degree, 3.5, knots, control_points, workspace) - (0.75 * 0. + 0.25 * 4.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(2, degree, 4., knots, control_points, workspace) - (0.5 * 0. + .5 * 4.), 1e-8);
  BOOST_CHECK_SMALL(
    deBoor(2, degree, 5., knots, control_points, workspace) - (0. * 0. + 1. * 4.), 1e-8);
}

// test deBoorBasis with degree 3 against bsplineBasis.
BOOST_AUTO_TEST_CASE(deBoorBasis_degree3)
{
  using pinocchio::internal::bsplineBasis;
  using pinocchio::internal::deBoor;

  const size_t degree = 3;
  Eigen::VectorXd knots(8);
  knots << 0., 2., 3., 5., 5.5, 8., 8.5, 10.;
  Eigen::VectorXd control_points(4);
  control_points << 1., 0., 0., 0.;
  Eigen::VectorXd workspace(4);

  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace) - bsplineBasis(0, degree, 5., knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.3, knots, control_points, workspace) - bsplineBasis(0, degree, 5.3, knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace) - bsplineBasis(0, degree, 5.5, knots),
    1e-8);

  control_points << 0., 1., 0., 0.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace) - bsplineBasis(1, degree, 5., knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.3, knots, control_points, workspace) - bsplineBasis(1, degree, 5.3, knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace) - bsplineBasis(1, degree, 5.5, knots),
    1e-8);

  control_points << 0., 0., 1., 0.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace) - bsplineBasis(2, degree, 5., knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.3, knots, control_points, workspace) - bsplineBasis(2, degree, 5.3, knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace) - bsplineBasis(2, degree, 5.5, knots),
    1e-8);

  control_points << 0., 0., 0., 1.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace) - bsplineBasis(3, degree, 5., knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.3, knots, control_points, workspace) - bsplineBasis(3, degree, 5.3, knots),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace) - bsplineBasis(3, degree, 5.5, knots),
    1e-8);
}

// test deBoorCumBasisSparse with degree 3 against manual cumulative
// basis computed with deBoorBasis
BOOST_AUTO_TEST_CASE(deBoorCumBasisSparse_degree3)
{
  using pinocchio::internal::deBoor;
  using pinocchio::internal::deBoorCumBasisSparse;

  const size_t degree = 3;
  Eigen::VectorXd knots(8);
  knots << 0., 2., 3., 5., 5.5, 8., 8.5, 10.;
  Eigen::VectorXd control_points(4);
  Eigen::VectorXd workspace_deboor(4);
  Eigen::MatrixXd workspace_deboor_cum(4, 4);

  control_points << 1., 1., 1., 1.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5., 0, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.2, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.2, 0, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.5, 0, knots, workspace_deboor_cum),
    1e-8);

  control_points << 0., 1., 1., 1.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5., 1, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.2, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.2, 1, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.5, 1, knots, workspace_deboor_cum),
    1e-8);

  control_points << 0., 0., 1., 1.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5., 2, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.2, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.2, 2, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.5, 2, knots, workspace_deboor_cum),
    1e-8);

  control_points << 0., 0., 0., 1.;
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5., knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5., 3, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.2, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.2, 3, knots, workspace_deboor_cum),
    1e-8);
  BOOST_CHECK_SMALL(
    deBoor(3, degree, 5.5, knots, control_points, workspace_deboor)
      - deBoorCumBasisSparse(3, degree, 5.5, 3, knots, workspace_deboor_cum),
    1e-8);
}

BOOST_AUTO_TEST_SUITE_END()
