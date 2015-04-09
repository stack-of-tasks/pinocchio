/*
 * Unittest of the RNE algorithm. The code simply test that the algorithm does
 * not cause any serious errors. The numerical values are not cross validated
 * in any way.
 *
 */

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/simulation/compute-all-terms.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CATTests
#include <boost/test/unit_test.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

#include <iostream>

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif


BOOST_AUTO_TEST_SUITE ( ComputeAllTerms )

BOOST_AUTO_TEST_CASE ( test_against_algo )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidSimple(model);
  se3::Data data(model); data.M.fill (0.);
  se3::Data data_other(model); data_other.M.fill (0.);

  VectorXd q (VectorXd::Random(model.nq));
  VectorXd v (VectorXd::Random(model.nv));

  // -------
  q.setZero ();
  v.setZero ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  computeJacobians(model,data_other,q);

  is_matrix_absolutely_closed (data.nle, data_other.nle);
  is_matrix_absolutely_closed (data.M.triangularView<Eigen::Upper>(), data_other.M.triangularView<Eigen::Upper>());
  is_matrix_absolutely_closed (data.J, data_other.J);

  // -------
  q.setZero ();
  v.setOnes ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  computeJacobians(model,data_other,q);

  is_matrix_absolutely_closed (data.nle, data_other.nle);
  is_matrix_absolutely_closed (data.M.triangularView<Eigen::Upper>(), data_other.M.triangularView<Eigen::Upper>());
  is_matrix_absolutely_closed (data.J, data_other.J);

  // -------
  q.setOnes ();
  v.setOnes ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  computeJacobians(model,data_other,q);

  is_matrix_absolutely_closed (data.nle, data_other.nle);
  is_matrix_absolutely_closed (data.M.triangularView<Eigen::Upper>(), data_other.M.triangularView<Eigen::Upper>());
  is_matrix_absolutely_closed (data.J, data_other.J);

  // -------
  q.setRandom ();
  v.setRandom ();

  computeAllTerms(model,data,q,v);

  nonLinearEffects(model,data_other,q,v);
  crba(model,data_other,q);
  computeJacobians(model,data_other,q);

  is_matrix_absolutely_closed (data.nle, data_other.nle);
  is_matrix_absolutely_closed (data.M.triangularView<Eigen::Upper>(), data_other.M.triangularView<Eigen::Upper>());
  is_matrix_absolutely_closed (data.J, data_other.J);
}

BOOST_AUTO_TEST_SUITE_END ()
