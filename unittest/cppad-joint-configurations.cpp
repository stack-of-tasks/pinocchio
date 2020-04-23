//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/autodiff/cppad.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "utils/model-generator.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_joint_configuration)
{
  typedef double Scalar;
  using CppAD::AD;
  using CppAD::NearEqual;
  
  typedef AD<Scalar> ADScalar;

  typedef pinocchio::ModelTpl<Scalar> Model;

  Model model; buildAllJointsModel(model);
  Eigen::VectorXd q2 = Eigen::VectorXd::Random(model.nq);
  Eigen::VectorXd q1 = Eigen::VectorXd::Random(model.nq);
  normalize(model,q1);
  normalize(model,q2);
  
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  std::vector<Eigen::VectorXd> results_q(2,Eigen::VectorXd::Zero(model.nq));
  std::vector<Eigen::VectorXd> results_v(2,Eigen::VectorXd::Zero(model.nv));
  
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::ConfigVectorType ADConfigVectorType;
  ADModel ad_model = model.cast<ADScalar>();
  ADConfigVectorType ad_q1(model.nq);
  ADConfigVectorType ad_q2(model.nq);
  ADConfigVectorType ad_v(model.nv);

  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorX;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> VectorXAD;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,Eigen::Dynamic> MatrixXAD;
  
  //Integrate
  {
    VectorXAD ad_x(model.nq+model.nv);
    ad_x << q1.cast<ADScalar>(), v.cast<ADScalar>();
    CppAD::Independent(ad_x);
    ad_q1 = ad_x.head(model.nq);
    ad_v = ad_x.tail(model.nv);
    
    VectorXAD ad_y(model.nq);
    pinocchio::integrate(ad_model,ad_q1,ad_v,ad_y);
    CppAD::ADFun<Scalar> ad_fun(ad_x,ad_y);
    
    CPPAD_TESTVECTOR(Scalar) x_eval((size_t)(ad_x.size()));
    Eigen::Map<VectorX>(x_eval.data(),ad_x.size()) << q1,v;
    CPPAD_TESTVECTOR(Scalar) y_eval((size_t)(ad_y.size()));
    y_eval = ad_fun.Forward(0,x_eval);
    results_q[0] = Eigen::Map<VectorX>(y_eval.data(),ad_y.size());
    
    pinocchio::integrate(model,q1,v,results_q[1]);
    BOOST_CHECK(results_q[0].isApprox(results_q[1]));
    
    Eigen::Map<VectorX>(x_eval.data(),ad_x.size()) << q1,VectorX::Zero(model.nv);
    y_eval = ad_fun.Forward(0,x_eval);
    results_q[0] = Eigen::Map<VectorX>(y_eval.data(),ad_y.size());
    BOOST_CHECK(results_q[0].isApprox(q1));
  }

  //Difference
  {
    VectorXAD ad_x(model.nq+model.nq);
    ad_x << q1.cast<ADScalar>(), q2.cast<ADScalar>();
    CppAD::Independent(ad_x);
    ad_q1 = ad_x.head(model.nq);
    ad_q2 = ad_x.tail(model.nq);
    
    VectorXAD ad_y(model.nv);
    pinocchio::difference(ad_model,ad_q1,ad_q2,ad_y);
    CppAD::ADFun<Scalar> ad_fun(ad_x,ad_y);
    
    CPPAD_TESTVECTOR(Scalar) x_eval((size_t)(ad_x.size()));
    Eigen::Map<VectorX>(x_eval.data(),ad_x.size()) << q1,q2;
    CPPAD_TESTVECTOR(Scalar) y_eval((size_t)(ad_y.size()));
    y_eval = ad_fun.Forward(0,x_eval);
    results_v[0] = Eigen::Map<VectorX>(y_eval.data(),ad_y.size());
    
    pinocchio::difference(model,q1,q2,results_v[1]);
    BOOST_CHECK(results_v[0].isApprox(results_v[1]));
    
    Eigen::Map<VectorX>(x_eval.data(),ad_x.size()) << q1,q1;
    y_eval = ad_fun.Forward(0,x_eval);
    results_v[0] = Eigen::Map<VectorX>(y_eval.data(),ad_y.size());
    BOOST_CHECK(results_v[0].isZero());
  }

  //dDifference
  std::vector<MatrixX> results_J0(2,MatrixX::Zero(model.nv,model.nv));
  std::vector<MatrixX> results_J1(2,MatrixX::Zero(model.nv,model.nv));
  {
    VectorXAD ad_x(model.nq+model.nq);
    ad_x << q1.cast<ADScalar>(), q2.cast<ADScalar>();
    CppAD::Independent(ad_x);
    ad_q1 = ad_x.head(model.nq);
    ad_q2 = ad_x.tail(model.nq);
    
    MatrixXAD ad_y(2*model.nv,model.nv);
    pinocchio::dDifference(ad_model,ad_q1,ad_q2,ad_y.topRows(model.nv),pinocchio::ARG0);
    pinocchio::dDifference(ad_model,ad_q1,ad_q2,ad_y.bottomRows(model.nv),pinocchio::ARG1);
    VectorXAD ad_y_flatten = Eigen::Map<VectorXAD>(ad_y.data(),ad_y.size());
    CppAD::ADFun<Scalar> ad_fun(ad_x,ad_y_flatten);
    
    CPPAD_TESTVECTOR(Scalar) x_eval((size_t)(ad_x.size()));
    Eigen::Map<VectorX>(x_eval.data(),ad_x.size()) << q1, q2;
    CPPAD_TESTVECTOR(Scalar) y_eval((size_t)(ad_y.size()));
    y_eval = ad_fun.Forward(0,x_eval);
    results_J0[0] = Eigen::Map<MatrixX>(y_eval.data(),ad_y.rows(),ad_y.cols()).topRows(model.nv);
    results_J1[0] = Eigen::Map<MatrixX>(y_eval.data(),ad_y.rows(),ad_y.cols()).bottomRows(model.nv);
    
    // w.r.t q1
    pinocchio::dDifference(model,q1,q2,results_J0[1],pinocchio::ARG0);
    BOOST_CHECK(results_J0[0].isApprox(results_J0[1]));
    
    // w.r.t q2
    pinocchio::dDifference(model,q1,q2,results_J1[1],pinocchio::ARG1);
    BOOST_CHECK(results_J1[0].isApprox(results_J1[1]));
    
    Eigen::Map<VectorX>(x_eval.data(),ad_x.size()) << q1, q1;
    y_eval = ad_fun.Forward(0,x_eval);
    results_J0[0] = Eigen::Map<MatrixX>(y_eval.data(),ad_y.rows(),ad_y.cols()).topRows(model.nv);
    results_J1[0] = Eigen::Map<MatrixX>(y_eval.data(),ad_y.rows(),ad_y.cols()).bottomRows(model.nv);
    
    BOOST_CHECK((-results_J0[0]).isIdentity());
    BOOST_CHECK(results_J1[0].isIdentity());
  }

}

BOOST_AUTO_TEST_SUITE_END()
