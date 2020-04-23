//
// Copyright (c) 2016-2020 CNRS INRIA
//

#include "utils/model-generator.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/math/quaternion.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( integration_test )
{
  Model model; buildAllJointsModel(model);

  std::vector<Eigen::VectorXd> qs(2);
  std::vector<Eigen::VectorXd> qdots(2);
  std::vector<Eigen::VectorXd> results(2);

  //
  // Test Case 0 : Integration of a config with zero velocity
  //
  qs[0] = Eigen::VectorXd::Ones(model.nq);
  normalize(model,qs[0]);
 
  qdots[0] = Eigen::VectorXd::Zero(model.nv);
  results[0] = integrate(model,qs[0],qdots[0]);

  BOOST_CHECK_MESSAGE(results[0].isApprox(qs[0], 1e-12), "integration of full body with zero velocity - wrong results");
}

BOOST_AUTO_TEST_CASE ( interpolate_test )
{
  Model model; buildAllJointsModel(model);

  Eigen::VectorXd q0(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
  Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));

  Eigen::VectorXd q01_0 = interpolate(model,q0,q1,0.0);
  BOOST_CHECK_MESSAGE(isSameConfiguration(model,q01_0,q0), "interpolation: q01_0 != q0");

  Eigen::VectorXd q01_1 = interpolate(model,q0,q1,1.0);
  BOOST_CHECK_MESSAGE(isSameConfiguration(model,q01_1,q1), "interpolation: q01_1 != q1");
}

BOOST_AUTO_TEST_CASE ( diff_integration_test )
{
  Model model; buildAllJointsModel(model);
  
  std::vector<Eigen::VectorXd> qs(2);
  std::vector<Eigen::VectorXd> vs(2);
  std::vector<Eigen::MatrixXd> results(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  std::vector<Eigen::MatrixXd> results_fd(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  
  qs[0] = Eigen::VectorXd::Ones(model.nq);
  normalize(model,qs[0]);
  
  vs[0] = Eigen::VectorXd::Zero(model.nv);
  vs[1] = Eigen::VectorXd::Ones(model.nv);
  dIntegrate(model,qs[0],vs[0],results[0],ARG0);
  
  Eigen::VectorXd q_fd(model.nq), v_fd(model.nv); v_fd.setZero();
  const double eps = 1e-8;
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_fd[k] = eps;
    q_fd = integrate(model,qs[0],v_fd);
    results_fd[0].col(k) = difference(model,qs[0],q_fd)/eps;
    v_fd[k] = 0.;
  }
  BOOST_CHECK(results[0].isIdentity(sqrt(eps)));
  BOOST_CHECK(results[0].isApprox(results_fd[0],sqrt(eps)));
  
  dIntegrate(model,qs[0],vs[0],results[1],ARG1);
  BOOST_CHECK(results[1].isApprox(results[0]));
  
  dIntegrate(model,qs[0],vs[1],results[0],ARG0);
  Eigen::VectorXd q_fd_intermediate(model.nq);
  Eigen::VectorXd q0_plus_v = integrate(model,qs[0],vs[1]);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_fd[k] = eps;
    q_fd_intermediate = integrate(model,qs[0],v_fd);
    q_fd = integrate(model,q_fd_intermediate,vs[1]);
    results_fd[0].col(k) = difference(model,q0_plus_v,q_fd)/eps;
    v_fd[k] = 0.;
  }
  
  BOOST_CHECK(results[0].isApprox(results_fd[0],sqrt(eps)));
  
  dIntegrate(model,qs[0],vs[1],results[1],ARG1);
  v_fd = vs[1];
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_fd[k] += eps;
    q_fd = integrate(model,qs[0],v_fd);
    results_fd[1].col(k) = difference(model,q0_plus_v,q_fd)/eps;
    v_fd[k] -= eps;
  }
  
  BOOST_CHECK(results[1].isApprox(results_fd[1],sqrt(eps)));
}

BOOST_AUTO_TEST_CASE ( diff_difference_test )
{
  Model model; buildAllJointsModel(model);
  
  std::vector<Eigen::VectorXd> qs(2);
  std::vector<Eigen::VectorXd> vs(2);
  std::vector<Eigen::MatrixXd> results(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  std::vector<Eigen::MatrixXd> results_fd(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  
  qs[0] = Eigen::VectorXd::Random(model.nq);
  normalize(model,qs[0]);
  const Eigen::VectorXd & q0 = qs[0];
  qs[1] = Eigen::VectorXd::Random(model.nq);
  normalize(model,qs[1]);
  const Eigen::VectorXd & q1 = qs[1];
  
  vs[0] = Eigen::VectorXd::Zero(model.nv);
  vs[1] = Eigen::VectorXd::Ones(model.nv);
  dDifference(model,q0,q1,results[0],ARG0);
  
  Eigen::VectorXd q_fd(model.nq), v_fd(model.nv); v_fd.setZero();
  const double eps = 1e-8;
  const Eigen::VectorXd v_ref = difference(model,q0,q1);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_fd[k] = eps;
    q_fd = integrate(model,q0,v_fd);
    results_fd[0].col(k) = (difference(model,q_fd,q1) - v_ref)/eps;
    v_fd[k] = 0.;
  }
  BOOST_CHECK(results[0].isApprox(results_fd[0],sqrt(eps)));
  
  dDifference(model,q0,q0,results[0],ARG0);
  BOOST_CHECK((-results[0]).isIdentity());
  
  dDifference(model,q0,q0,results[1],ARG1);
  BOOST_CHECK(results[1].isIdentity());
  
  dDifference(model,q0,q1,results[1],ARG1);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_fd[k] = eps;
    q_fd = integrate(model,q1,v_fd);
    results_fd[1].col(k) = (difference(model,q0,q_fd) - v_ref)/eps;
    v_fd[k] = 0.;
  }
  BOOST_CHECK(results[1].isApprox(results_fd[1],sqrt(eps)));
}

BOOST_AUTO_TEST_CASE ( diff_difference_vs_diff_integrate )
{
  Model model; buildAllJointsModel(model);
  
  std::vector<Eigen::VectorXd> qs(2);
  std::vector<Eigen::VectorXd> vs(2);
  std::vector<Eigen::MatrixXd> results(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  std::vector<Eigen::MatrixXd> results_fd(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  
  Eigen::VectorXd q0 = Eigen::VectorXd::Random(model.nq);
  normalize(model,q0);
  
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd q1 = integrate(model,q0,v);
  
  Eigen::VectorXd v_diff = difference(model,q0,q1);
  BOOST_CHECK(v_diff.isApprox(v));
  
  Eigen::MatrixXd J_int_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd J_int_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
  dIntegrate(model,q0,v,J_int_dq,ARG0);
  dIntegrate(model,q0,v,J_int_dv,ARG1);
  
  Eigen::MatrixXd J_diff_dq0 = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd J_diff_dq1 = Eigen::MatrixXd::Zero(model.nv,model.nv);
  dDifference(model,q0,q1,J_diff_dq0,ARG0);
  dDifference(model,q0,q1,J_diff_dq1,ARG1);
  
  BOOST_CHECK(J_int_dq.isApprox(Eigen::MatrixXd(-(J_int_dv * J_diff_dq0))));
  BOOST_CHECK(Eigen::MatrixXd(J_int_dv * J_diff_dq1).isIdentity());
}


BOOST_AUTO_TEST_CASE ( dIntegrate_assignementop_test )
{
  Model model; buildAllJointsModel(model);
  
  std::vector<Eigen::MatrixXd> results(3,Eigen::MatrixXd::Zero(model.nv,model.nv));
  
  Eigen::VectorXd qs = Eigen::VectorXd::Ones(model.nq);
  normalize(model,qs);
  
  Eigen::VectorXd vs = Eigen::VectorXd::Random(model.nv);

  //SETTO
  dIntegrate(model,qs,vs,results[0],ARG0);
  dIntegrate(model,qs,vs,results[1],ARG0,SETTO);
  BOOST_CHECK(results[0].isApprox(results[1]));

  //ADDTO
  results[1] = Eigen::MatrixXd::Random(model.nv, model.nv);
  results[2] = results[1];
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG0,SETTO);
  dIntegrate(model,qs,vs,results[1],ARG0,ADDTO);
  BOOST_CHECK(results[1].isApprox(results[2] + results[0]));

  //RMTO
  results[1] = Eigen::MatrixXd::Random(model.nv, model.nv);
  results[2] = results[1];
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG0,SETTO);
  dIntegrate(model,qs,vs,results[1],ARG0,RMTO);
  BOOST_CHECK(results[1].isApprox(results[2] - results[0]));

  //SETTO
  results[0].setZero();
  results[1].setZero();
  dIntegrate(model,qs,vs,results[0],ARG1);
  dIntegrate(model,qs,vs,results[1],ARG1,SETTO);
  BOOST_CHECK(results[0].isApprox(results[1]));

  //ADDTO
  results[1] = Eigen::MatrixXd::Random(model.nv, model.nv);
  results[2] = results[1];
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG1,SETTO);
  dIntegrate(model,qs,vs,results[1],ARG1,ADDTO);
  BOOST_CHECK(results[1].isApprox(results[2] + results[0]));

  //RMTO
  results[1] = Eigen::MatrixXd::Random(model.nv, model.nv);
  results[2] = results[1];
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG1,SETTO);
  dIntegrate(model,qs,vs,results[1],ARG1,RMTO);
  BOOST_CHECK(results[1].isApprox(results[2] - results[0]));

  //Transport
  std::vector<Eigen::MatrixXd> J(2,Eigen::MatrixXd::Zero(model.nv,2*model.nv));
  J[0] = Eigen::MatrixXd::Random(model.nv, 2*model.nv);
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG0,SETTO);
  dIntegrateTransport(model,qs,vs,J[0],J[1],ARG0);
  BOOST_CHECK(J[1].isApprox(results[0] * J[0]));

  J[0] = Eigen::MatrixXd::Random(model.nv, 2*model.nv);
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG1,SETTO);
  dIntegrateTransport(model,qs,vs,J[0],J[1],ARG1);
  BOOST_CHECK(J[1].isApprox(results[0] * J[0]));  

  //TransportInPlace
  J[1] = Eigen::MatrixXd::Random(model.nv, 2*model.nv);
  J[0] = J[1];
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG0,SETTO);
  dIntegrateTransport(model,qs,vs,J[1],ARG0);
  BOOST_CHECK(J[1].isApprox(results[0] * J[0]));

  J[1] = Eigen::MatrixXd::Random(model.nv, 2*model.nv);
  J[0] = J[1];
  results[0].setZero();
  dIntegrate(model,qs,vs,results[0],ARG1,SETTO);
  dIntegrateTransport(model,qs,vs,J[1],ARG1);
  BOOST_CHECK(J[1].isApprox(results[0] * J[0]));  

}

BOOST_AUTO_TEST_CASE ( integrate_difference_test )
{
 Model model; buildAllJointsModel(model);

 Eigen::VectorXd q0(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
 Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
 Eigen::VectorXd qdot(Eigen::VectorXd::Random(model.nv));

 BOOST_CHECK_MESSAGE(isSameConfiguration(model, integrate(model, q0, difference(model, q0,q1)), q1), "Integrate (difference) - wrong results");

 BOOST_CHECK_MESSAGE(difference(model, q0, integrate(model,q0, qdot)).isApprox(qdot),"difference (integrate) - wrong results");
}


BOOST_AUTO_TEST_CASE ( neutral_configuration_test )
{
  Model model; buildAllJointsModel(model);

  Eigen::VectorXd expected(model.nq);
  expected << 0,0,0,0,0,0,1,
              0,0,0,1,
              0,0,1,0,
              0,
              0,
              0,
              0,
              0,0,0,
              0,0,0;


  Eigen::VectorXd neutral_config = neutral(model);
  BOOST_CHECK_MESSAGE(neutral_config.isApprox(expected, 1e-12), "neutral configuration - wrong results");
}

BOOST_AUTO_TEST_CASE ( distance_configuration_test )
{
  Model model; buildAllJointsModel(model);
  
  Model::ConfigVectorType q0 = neutral(model);
  Model::ConfigVectorType q1(integrate (model, q0, Model::TangentVectorType::Ones(model.nv)));

  double dist = distance(model,q0,q1);
  
  BOOST_CHECK_MESSAGE(dist > 0., "distance - wrong results");
  BOOST_CHECK_SMALL(dist-difference(model,q0,q1).norm(), 1e-12);
}

BOOST_AUTO_TEST_CASE ( squared_distance_test )
{
  Model model; buildAllJointsModel(model);

  Eigen::VectorXd q0(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
  Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));

  double dist = distance(model,q0,q1);
  Eigen::VectorXd squaredDistance_ = squaredDistance(model,q0,q1);

  BOOST_CHECK_SMALL(dist-math::sqrt(squaredDistance_.sum()), 1e-12);
}

BOOST_AUTO_TEST_CASE ( uniform_sampling_test )
{
  Model model; buildAllJointsModel(model);

  model.lowerPositionLimit = -1 * Eigen::VectorXd::Ones(model.nq);
  model.upperPositionLimit = Eigen::VectorXd::Ones(model.nq);
  Eigen::VectorXd q1(randomConfiguration(model));
  
  for (int i = 0; i < model.nq; ++i)
  {
    BOOST_CHECK_MESSAGE(q1[i] >= model.lowerPositionLimit[i] && q1[i] <= model.upperPositionLimit[i], " UniformlySample : Generated config not in bounds");
  }
}

BOOST_AUTO_TEST_CASE ( normalize_test )
{
  Model model; buildAllJointsModel(model);

  Eigen::VectorXd q (Eigen::VectorXd::Ones(model.nq));
  pinocchio::normalize(model, q);

  BOOST_CHECK(q.head<3>().isApprox(Eigen::VectorXd::Ones(3)));
  BOOST_CHECK(fabs(q.segment<4>(3).norm() - 1) < Eigen::NumTraits<double>::epsilon()); // quaternion of freeflyer
  BOOST_CHECK(fabs(q.segment<4>(7).norm() - 1) < Eigen::NumTraits<double>::epsilon()); // quaternion of spherical joint
  const int n = model.nq - 7 - 4 - 4; // free flyer + spherical + planar
  BOOST_CHECK(q.tail(n).isApprox(Eigen::VectorXd::Ones(n)));
}

BOOST_AUTO_TEST_CASE ( integrateCoeffWiseJacobian_test )
{
  Model model; buildModels::humanoidRandom(model);
  
  Eigen::VectorXd q(Eigen::VectorXd::Ones(model.nq));
  pinocchio::normalize(model, q);
  
  Eigen::MatrixXd jac(model.nq,model.nv); jac.setZero();
 
  integrateCoeffWiseJacobian(model,q,jac);
  
 
  Eigen::MatrixXd jac_fd(model.nq,model.nv);
  Eigen::VectorXd q_plus;
  const double eps = 1e-8;
  
  Eigen::VectorXd v_eps(Eigen::VectorXd::Zero(model.nv));
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] = eps;
    q_plus = integrate(model,q,v_eps);
    jac_fd.col(k) = (q_plus - q)/eps;
    
    v_eps[k] = 0.;
  }
  BOOST_CHECK(jac.isApprox(jac_fd,sqrt(eps)));
}

BOOST_AUTO_TEST_SUITE_END ()
