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
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE NLETests
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <iostream>

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif

inline void is_matrix_closed (const Eigen::MatrixXd & M1,
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
      BOOST_CHECK_CLOSE (M1 (i,j), M2 (i,j), tolerance);
    }
  }
}


BOOST_AUTO_TEST_SUITE ( NLE )

BOOST_AUTO_TEST_CASE ( test_against_rnea )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidSimple(model);
  se3::Data data_nle(model);
  se3::Data data_rnea(model);

  VectorXd q (VectorXd::Random(model.nq));
  VectorXd v (VectorXd::Random(model.nv));

  VectorXd tau_nle (VectorXd::Zero (model.nv));
  VectorXd tau_rnea (VectorXd::Zero (model.nv));

  // -------
  q.setZero ();
  v.setZero ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  is_matrix_closed (tau_nle, tau_rnea);

  // -------
  q.setZero ();
  v.setOnes ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  is_matrix_closed (tau_nle, tau_rnea);

  // -------
  q.setOnes ();
  v.setOnes ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  is_matrix_closed (tau_nle, tau_rnea);

  // -------
  q.setRandom ();
  v.setRandom ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  is_matrix_closed (tau_nle, tau_rnea);
}

BOOST_AUTO_TEST_SUITE_END ()

//int main()
//{
//#ifdef __SSE3__
//  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
//#endif
//
//  using namespace Eigen;
//  using namespace se3;
//
//  se3::Model model; buildModels::humanoidSimple(model);
//  
//  se3::Data data(model);
//
//  VectorXd q (VectorXd::Random(model.nq));
//  VectorXd v (VectorXd::Random(model.nv));
// 
//#ifdef NDEBUG
//  int NBT = 10000;
//#else
//  int NBT = 1;
//  std::cout << "(the time score in debug mode is not relevant)  " ;
//#endif
//
//  StackTicToc timer(StackTicToc::US); timer.tic();
//  SMOOTH(NBT)
//    {
//      nonLinearEffects(model,data,q,v);
//    }
//  timer.toc(std::cout,NBT);
//
//  return 0;
//}
