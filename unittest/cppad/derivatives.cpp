//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/autodiff/cppad.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_rnea_derivatives)
{
  using CppAD::AD;
  using CppAD::NearEqual;
  
  typedef double Scalar;
  typedef AD<Scalar> ADScalar;
  
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;
  
  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);
  
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  
  // Sample random configuration
  typedef Model::ConfigVectorType ConfigVectorType;
  typedef Model::TangentVectorType TangentVectorType;
  ConfigVectorType q(model.nq);
  q = pinocchio::randomConfiguration(model);

  TangentVectorType v(TangentVectorType::Random(model.nv));
  TangentVectorType a(TangentVectorType::Random(model.nv));
  
  Eigen::MatrixXd rnea_partial_dq(model.nv,model.nv); rnea_partial_dq.setZero();
  Eigen::MatrixXd rnea_partial_dv(model.nv,model.nv); rnea_partial_dv.setZero();
  Eigen::MatrixXd rnea_partial_da(model.nv,model.nv); rnea_partial_da.setZero();
  
  pinocchio::computeRNEADerivatives(model,data,q,v,a,
                              rnea_partial_dq,
                              rnea_partial_dv,
                              rnea_partial_da);
  
  rnea_partial_da.triangularView<Eigen::StrictlyLower>()
  = rnea_partial_da.transpose().triangularView<Eigen::StrictlyLower>();
  
  typedef ADModel::ConfigVectorType ADConfigVectorType;
  typedef ADModel::TangentVectorType ADTangentVectorType;
  
  ADConfigVectorType ad_q = q.cast<ADScalar>();
  ADTangentVectorType ad_dq = ADTangentVectorType::Zero(model.nv);
  ADTangentVectorType ad_v = v.cast<ADScalar>();
  ADTangentVectorType ad_a = a.cast<ADScalar>();
  
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> VectorXAD;
  pinocchio::crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Data::TangentVectorType tau = pinocchio::rnea(model,data,q,v,a);
  
  // dtau_dq
  {
    CppAD::Independent(ad_dq);
    ADConfigVectorType ad_q_plus = pinocchio::integrate(ad_model,ad_q,ad_dq);
    pinocchio::rnea(ad_model,ad_data,ad_q_plus,ad_v,ad_a);
    
    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.tau;
    
    CppAD::ADFun<Scalar> ad_fun(ad_dq,Y);
    
    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1).setZero();
    
    CPPAD_TESTVECTOR(Scalar) tau = ad_fun.Forward(0,x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(tau.data(),model.nv,1).isApprox(data.tau));
    
    CPPAD_TESTVECTOR(Scalar) dtau_dq = ad_fun.Jacobian(x);
    Data::MatrixXs dtau_dq_mat = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(dtau_dq.data(),model.nv,model.nv);
    BOOST_CHECK(dtau_dq_mat.isApprox(rnea_partial_dq));
  }
  
  // dtau_dv
  {
    CppAD::Independent(ad_v);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);

    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.tau;

    CppAD::ADFun<Scalar> ad_fun(ad_v,Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = v;

    CPPAD_TESTVECTOR(Scalar) tau = ad_fun.Forward(0,x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(tau.data(),model.nv,1).isApprox(data.tau));

    CPPAD_TESTVECTOR(Scalar) dtau_dv = ad_fun.Jacobian(x);
    Data::MatrixXs dtau_dv_mat = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(dtau_dv.data(),model.nv,model.nv);
    BOOST_CHECK(dtau_dv_mat.isApprox(rnea_partial_dv));
  }
  
  // dtau_da
  {
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);
    
    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.tau;
    
    CppAD::ADFun<Scalar> ad_fun(ad_a,Y);
    
    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = a;
    
    CPPAD_TESTVECTOR(Scalar) tau = ad_fun.Forward(0,x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(tau.data(),model.nv,1).isApprox(data.tau));
    
    CPPAD_TESTVECTOR(Scalar) dtau_da = ad_fun.Jacobian(x);
    Data::MatrixXs dtau_da_mat = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(dtau_da.data(),model.nv,model.nv);
    BOOST_CHECK(dtau_da_mat.isApprox(rnea_partial_da));
    BOOST_CHECK(dtau_da_mat.isApprox(data.M));
  }
  
}

BOOST_AUTO_TEST_CASE(test_aba_derivatives)
{
  using CppAD::AD;
  using CppAD::NearEqual;
  
  typedef double Scalar;
  typedef AD<Scalar> ADScalar;
  
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;
  
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;
  
  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);
  
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  
  // Sample random configuration
  typedef Model::ConfigVectorType ConfigVectorType;
  typedef Model::TangentVectorType TangentVectorType;
  ConfigVectorType q(model.nq);
  q = pinocchio::randomConfiguration(model);
  
  TangentVectorType v(TangentVectorType::Random(model.nv));
  TangentVectorType tau(TangentVectorType::Random(model.nv));
  
  Eigen::MatrixXd aba_partial_dq(model.nv,model.nv); aba_partial_dq.setZero();
  Eigen::MatrixXd aba_partial_dv(model.nv,model.nv); aba_partial_dv.setZero();
  Eigen::MatrixXd aba_partial_dtau(model.nv,model.nv); aba_partial_dtau.setZero();
  
  pinocchio::computeABADerivatives(model,data,q,v,tau,
                             aba_partial_dq,
                             aba_partial_dv,
                             aba_partial_dtau);
  
  aba_partial_dtau.triangularView<Eigen::StrictlyLower>()
  = aba_partial_dtau.transpose().triangularView<Eigen::StrictlyLower>();
  
  typedef ADModel::ConfigVectorType ADConfigVectorType;
  typedef ADModel::TangentVectorType ADTangentVectorType;
  
  ADConfigVectorType ad_q = q.cast<ADScalar>();
  ADTangentVectorType ad_dq = ADTangentVectorType::Zero(model.nv);
  ADTangentVectorType ad_v = v.cast<ADScalar>();
  ADTangentVectorType ad_tau = tau.cast<ADScalar>();
  
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> VectorXAD;
  pinocchio::computeMinverse(model,data,q);
  data.Minv.triangularView<Eigen::StrictlyLower>()
  = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  
  Data::TangentVectorType ddq = pinocchio::aba(model,data,q,v,tau);
  
  // dddq_dq
  {
    CppAD::Independent(ad_dq);
    ADConfigVectorType ad_q_plus = pinocchio::integrate(ad_model,ad_q,ad_dq);
    pinocchio::aba(ad_model,ad_data,ad_q_plus,ad_v,ad_tau);
    
    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.ddq;
    
    CppAD::ADFun<Scalar> ad_fun(ad_dq,Y);
    
    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1).setZero();
    
    CPPAD_TESTVECTOR(Scalar) ddq = ad_fun.Forward(0,x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(ddq.data(),model.nv,1).isApprox(data.ddq));
    
    CPPAD_TESTVECTOR(Scalar) ddq_dq = ad_fun.Jacobian(x);
    Data::MatrixXs ddq_dq_mat = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(ddq_dq.data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dq_mat.isApprox(aba_partial_dq));
  }
  
  // dddq_dv
  {
    CppAD::Independent(ad_v);
    pinocchio::aba(ad_model,ad_data,ad_q,ad_v,ad_tau);
    
    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.ddq;
    
    CppAD::ADFun<Scalar> ad_fun(ad_v,Y);
    
    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = v;
    
    CPPAD_TESTVECTOR(Scalar) ddq = ad_fun.Forward(0,x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(ddq.data(),model.nv,1).isApprox(data.ddq));
    
    CPPAD_TESTVECTOR(Scalar) ddq_dv = ad_fun.Jacobian(x);
    Data::MatrixXs ddq_dv_mat = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(ddq_dv.data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dv_mat.isApprox(aba_partial_dv));
  }
  
  // dddq_da
  {
    CppAD::Independent(ad_tau);
    pinocchio::aba(ad_model,ad_data,ad_q,ad_v,ad_tau);
    
    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.ddq;
    
    CppAD::ADFun<Scalar> ad_fun(ad_tau,Y);
    
    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = tau;
    
    CPPAD_TESTVECTOR(Scalar) ddq = ad_fun.Forward(0,x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(ddq.data(),model.nv,1).isApprox(data.ddq));
    
    CPPAD_TESTVECTOR(Scalar) ddq_dtau = ad_fun.Jacobian(x);
    Data::MatrixXs ddq_dtau_mat = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(ddq_dtau.data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dtau_mat.isApprox(aba_partial_dtau));
    BOOST_CHECK(ddq_dtau_mat.isApprox(data.Minv));
  }
  
}

BOOST_AUTO_TEST_SUITE_END()
