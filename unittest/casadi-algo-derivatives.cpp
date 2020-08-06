//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/autodiff/casadi.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <casadi/casadi.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_integrate)
{
  typedef double Scalar;
  typedef casadi::SX ADScalar;
  
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;
  
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;
  
  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);
  
  typedef Model::ConfigVectorType ConfigVector;
  typedef Model::TangentVectorType TangentVector;
  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector a(TangentVector::Random(model.nv));
  
  typedef ADModel::ConfigVectorType ConfigVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  
  pinocchio::rnea(model,data,q,v,a);
  
  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  casadi::SX cs_v_int = casadi::SX::sym("v_inc", model.nv);
  
  ConfigVectorAD q_ad(model.nq), v_int_ad(model.nv), q_int_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
  v_int_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_v_int).data(),model.nv,1);
  
  pinocchio::integrate(ad_model,q_ad,v_int_ad,q_int_ad);
  casadi::SX cs_q_int(model.nq,1);
  pinocchio::casadi::copy(q_int_ad,cs_q_int);
  
  std::cout << "cs_q_int:" << cs_q_int << std::endl;
  casadi::Function eval_integrate("eval_integrate",
                                  casadi::SXVector {cs_q,cs_v_int},
                                  casadi::SXVector {cs_q_int});
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
  
  std::vector<double> v_int_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_int_vec.data(),model.nv,1).setZero();
  casadi::DM q_int_res = eval_integrate(casadi::DMVector {q_vec,v_int_vec})[0];
  
  Data::ConfigVectorType q_int_vec = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(q_int_res).data(),model.nq,1);
  
  ConfigVector q_plus(model.nq);
  pinocchio::integrate(model,q,TangentVector::Zero(model.nv),q_plus);
  
  std::cout << "q_int_vec: " << q_int_vec.transpose() << std::endl;
  BOOST_CHECK(q_plus.isApprox(q_int_vec));
}
  
BOOST_AUTO_TEST_CASE(test_rnea_derivatives)
{
  typedef double Scalar;
  typedef casadi::SX ADScalar;
  
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;
  
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;
  
  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);
  
  typedef Model::ConfigVectorType ConfigVector;
  typedef Model::TangentVectorType TangentVector;
  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector a(TangentVector::Random(model.nv));
  
  typedef ADModel::ConfigVectorType ConfigVectorAD;
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  
  pinocchio::rnea(model,data,q,v,a);
  
  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  casadi::SX cs_v_int = casadi::SX::sym("v_inc", model.nv);
  
  ConfigVectorAD q_ad(model.nq), v_int_ad(model.nv), q_int_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
  v_int_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_v_int).data(),model.nv,1);
  
  pinocchio::integrate(ad_model,q_ad,v_int_ad,q_int_ad);
  casadi::SX cs_q_int(model.nq,1);
  pinocchio::casadi::copy(q_int_ad,cs_q_int);
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
  
  std::vector<double> v_int_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_int_vec.data(),model.nv,1).setZero();
  
  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
  
  casadi::SX cs_a = casadi::SX::sym("a", model.nv);
  TangentVectorAD a_ad(model.nv);
  a_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_a).data(),model.nv,1);
  
  rnea(ad_model,ad_data,q_int_ad,v_ad,a_ad);
  casadi::SX cs_tau(model.nv,1);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    cs_tau(k) = ad_data.tau[k];
  }
  casadi::Function eval_rnea("eval_rnea",
                             casadi::SXVector {cs_q,cs_v_int, cs_v, cs_a},
                             casadi::SXVector {cs_tau});
  
  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v;
  
  std::vector<double> a_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(a_vec.data(),model.nv,1) = a;
  
  // check return value
  casadi::DM tau_res = eval_rnea(casadi::DMVector {q_vec,v_int_vec,v_vec,a_vec})[0];
  std::cout << "tau_res = " << tau_res << std::endl;
  Data::TangentVectorType tau_vec = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(tau_res).data(),model.nv,1);
  
  BOOST_CHECK(data.tau.isApprox(tau_vec));
  
  // compute references
  Data::MatrixXs dtau_dq_ref(model.nv,model.nv), dtau_dv_ref(model.nv,model.nv), dtau_da_ref(model.nv,model.nv);
  dtau_dq_ref.setZero(); dtau_dv_ref.setZero(); dtau_da_ref.setZero();
  
  pinocchio::computeRNEADerivatives(model,data,q,v,a,dtau_dq_ref,dtau_dv_ref,dtau_da_ref);
  dtau_da_ref.triangularView<Eigen::StrictlyLower>() = dtau_da_ref.transpose().triangularView<Eigen::StrictlyLower>();
  
  // check with respect to q+dq
  casadi::SX dtau_dq = jacobian(cs_tau, cs_v_int);
  casadi::Function eval_dtau_dq("eval_dtau_dq",
                                casadi::SXVector {cs_q,cs_v_int, cs_v, cs_a},
                                casadi::SXVector {dtau_dq});
  
  casadi::DM dtau_dq_res = eval_dtau_dq(casadi::DMVector {q_vec,v_int_vec,v_vec,a_vec})[0];
  std::vector<double> dtau_dq_vec(static_cast< std::vector<double> >(dtau_dq_res));
  BOOST_CHECK(Eigen::Map<Data::MatrixXs>(dtau_dq_vec.data(),model.nv,model.nv).isApprox(dtau_dq_ref));
  
  // check with respect to v+dv
  casadi::SX dtau_dv = jacobian(cs_tau, cs_v);
  casadi::Function eval_dtau_dv("eval_dtau_dv",
                                casadi::SXVector {cs_q,cs_v_int, cs_v, cs_a},
                                casadi::SXVector {dtau_dv});
  
  casadi::DM dtau_dv_res = eval_dtau_dv(casadi::DMVector {q_vec,v_int_vec,v_vec,a_vec})[0];
  std::vector<double> dtau_dv_vec(static_cast< std::vector<double> >(dtau_dv_res));
  BOOST_CHECK(Eigen::Map<Data::MatrixXs>(dtau_dv_vec.data(),model.nv,model.nv).isApprox(dtau_dv_ref));
  
  // check with respect to a+da
  casadi::SX dtau_da = jacobian(cs_tau, cs_a);
  casadi::Function eval_dtau_da("eval_dtau_da",
                                casadi::SXVector {cs_q,cs_v_int, cs_v, cs_a},
                                casadi::SXVector {dtau_da});
  
  casadi::DM dtau_da_res = eval_dtau_da(casadi::DMVector {q_vec,v_int_vec,v_vec,a_vec})[0];
  std::vector<double> dtau_da_vec(static_cast< std::vector<double> >(dtau_da_res));
  BOOST_CHECK(Eigen::Map<Data::MatrixXs>(dtau_da_vec.data(),model.nv,model.nv).isApprox(dtau_da_ref));
  
  // call RNEA derivatives in Casadi
  casadi::SX cs_dtau_dq(model.nv,model.nv);
  casadi::SX cs_dtau_dv(model.nv,model.nv);
  casadi::SX cs_dtau_da(model.nv,model.nv);
  
  computeRNEADerivatives(ad_model,ad_data,q_ad,v_ad,a_ad);
  ad_data.M.triangularView<Eigen::StrictlyLower>()
  = ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  pinocchio::casadi::copy(ad_data.dtau_dq,cs_dtau_dq);
  pinocchio::casadi::copy(ad_data.dtau_dv,cs_dtau_dv);
  pinocchio::casadi::copy(ad_data.M,cs_dtau_da);
  
  casadi::Function eval_rnea_derivatives_dq("eval_rnea_derivatives_dq",
                                            casadi::SXVector {cs_q, cs_v, cs_a},
                                            casadi::SXVector {cs_dtau_dq});
  
  casadi::DM dtau_dq_res_direct = eval_rnea_derivatives_dq(casadi::DMVector {q_vec,v_vec,a_vec})[0];
  Data::MatrixXs dtau_dq_res_direct_map = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(dtau_dq_res_direct).data(),model.nv,model.nv);
  BOOST_CHECK(dtau_dq_ref.isApprox(dtau_dq_res_direct_map));
  
  casadi::Function eval_rnea_derivatives_dv("eval_rnea_derivatives_dv",
                                            casadi::SXVector {cs_q, cs_v, cs_a},
                                            casadi::SXVector {cs_dtau_dv});
  
  casadi::DM dtau_dv_res_direct = eval_rnea_derivatives_dv(casadi::DMVector {q_vec,v_vec,a_vec})[0];
  Data::MatrixXs dtau_dv_res_direct_map = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(dtau_dv_res_direct).data(),model.nv,model.nv);
  BOOST_CHECK(dtau_dv_ref.isApprox(dtau_dv_res_direct_map));
  
  casadi::Function eval_rnea_derivatives_da("eval_rnea_derivatives_da",
                                            casadi::SXVector {cs_q, cs_v, cs_a},
                                            casadi::SXVector {cs_dtau_da});
  
  casadi::DM dtau_da_res_direct = eval_rnea_derivatives_da(casadi::DMVector {q_vec,v_vec,a_vec})[0];
  Data::MatrixXs dtau_da_res_direct_map = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(dtau_da_res_direct).data(),model.nv,model.nv);
  BOOST_CHECK(dtau_da_ref.isApprox(dtau_da_res_direct_map));
}
  
  BOOST_AUTO_TEST_CASE(test_aba)
  {
    typedef double Scalar;
    typedef casadi::SX ADScalar;

    typedef pinocchio::ModelTpl<Scalar> Model;
    typedef Model::Data Data;

    typedef pinocchio::ModelTpl<ADScalar> ADModel;
    typedef ADModel::Data ADData;

    Model model;
    pinocchio::buildModels::humanoidRandom(model);
    model.lowerPositionLimit.head<3>().fill(-1.);
    model.upperPositionLimit.head<3>().fill(1.);
    Data data(model);

    typedef Model::ConfigVectorType ConfigVector;
    typedef Model::TangentVectorType TangentVector;
    ConfigVector q(model.nq);
    q = pinocchio::randomConfiguration(model);
    TangentVector v(TangentVector::Random(model.nv));
    TangentVector tau(TangentVector::Random(model.nv));

    typedef ADModel::ConfigVectorType ConfigVectorAD;
    typedef ADModel::TangentVectorType TangentVectorAD;
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    pinocchio::aba(model,data,q,v,tau);

    casadi::SX cs_q = casadi::SX::sym("q", model.nq);
    casadi::SX cs_v_int = casadi::SX::sym("v_inc", model.nv);
    ConfigVectorAD q_ad(model.nq), v_int_ad(model.nv), q_int_ad(model.nq);
    q_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
    v_int_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_v_int).data(),model.nv,1);
    
    pinocchio::integrate(ad_model,q_ad,v_int_ad,q_int_ad);
    casadi::SX cs_q_int(model.nq,1);
    pinocchio::casadi::copy(q_int_ad,cs_q_int);
    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
    
    std::vector<double> v_int_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_int_vec.data(),model.nv,1).setZero();

    casadi::SX cs_v = casadi::SX::sym("v", model.nv);
    TangentVectorAD v_ad(model.nv);
    v_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);

    casadi::SX cs_tau = casadi::SX::sym("tau", model.nv);
    TangentVectorAD tau_ad(model.nv);
    tau_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_tau).data(),model.nv,1);

    // ABA
    aba(ad_model,ad_data,q_int_ad,v_ad,tau_ad);
    casadi::SX cs_ddq(model.nv,1);
    for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
      cs_ddq(k) = ad_data.ddq[k];
    casadi::Function eval_aba("eval_aba",
                              casadi::SXVector {cs_q, cs_v_int, cs_v, cs_tau},
                              casadi::SXVector {cs_ddq});

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v;

    std::vector<double> tau_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(tau_vec.data(),model.nv,1) = tau;

    casadi::DM ddq_res = eval_aba(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
    Data::TangentVectorType ddq_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res).data(),
                                                            model.nv,1);

    BOOST_CHECK(ddq_mat.isApprox(data.ddq));
    
    // compute references
    Data::MatrixXs ddq_dq_ref(model.nv,model.nv), ddq_dv_ref(model.nv,model.nv), ddq_dtau_ref(model.nv,model.nv);
    ddq_dq_ref.setZero(); ddq_dv_ref.setZero(); ddq_dtau_ref.setZero();
    
    pinocchio::computeABADerivatives(model,data,q,v,tau,ddq_dq_ref,ddq_dv_ref,ddq_dtau_ref);
    ddq_dtau_ref.triangularView<Eigen::StrictlyLower>()
    = ddq_dtau_ref.transpose().triangularView<Eigen::StrictlyLower>();
    
    // check with respect to q+dq
    casadi::SX ddq_dq = jacobian(cs_ddq, cs_v_int);
    casadi::Function eval_ddq_dq("eval_ddq_dq",
                                  casadi::SXVector {cs_q,cs_v_int,cs_v,cs_tau},
                                  casadi::SXVector {ddq_dq});
    
    casadi::DM ddq_dq_res = eval_ddq_dq(casadi::DMVector {q_vec,v_int_vec,v_vec,tau_vec})[0];
    std::vector<double> ddq_dq_vec(static_cast< std::vector<double> >(ddq_dq_res));
    BOOST_CHECK(Eigen::Map<Data::MatrixXs>(ddq_dq_vec.data(),model.nv,model.nv).isApprox(ddq_dq_ref));
    
    // check with respect to v+dv
    casadi::SX ddq_dv = jacobian(cs_ddq, cs_v);
    casadi::Function eval_ddq_dv("eval_ddq_dv",
                                  casadi::SXVector {cs_q,cs_v_int, cs_v, cs_tau},
                                  casadi::SXVector {ddq_dv});
    
    casadi::DM ddq_dv_res = eval_ddq_dv(casadi::DMVector {q_vec,v_int_vec,v_vec,tau_vec})[0];
    std::vector<double> ddq_dv_vec(static_cast< std::vector<double> >(ddq_dv_res));
    BOOST_CHECK(Eigen::Map<Data::MatrixXs>(ddq_dv_vec.data(),model.nv,model.nv).isApprox(ddq_dv_ref));
    
    // check with respect to a+da
    casadi::SX ddq_dtau = jacobian(cs_ddq, cs_tau);
    casadi::Function eval_ddq_da("eval_ddq_da",
                                  casadi::SXVector {cs_q,cs_v_int, cs_v, cs_tau},
                                  casadi::SXVector {ddq_dtau});
    
    casadi::DM ddq_dtau_res = eval_ddq_da(casadi::DMVector {q_vec,v_int_vec,v_vec,tau_vec})[0];
    std::vector<double> ddq_dtau_vec(static_cast< std::vector<double> >(ddq_dtau_res));
    BOOST_CHECK(Eigen::Map<Data::MatrixXs>(ddq_dtau_vec.data(),model.nv,model.nv).isApprox(ddq_dtau_ref));
    
    // call ABA derivatives in Casadi
    casadi::SX cs_ddq_dq(model.nv,model.nv);
    casadi::SX cs_ddq_dv(model.nv,model.nv);
    casadi::SX cs_ddq_dtau(model.nv,model.nv);
    
    computeABADerivatives(ad_model,ad_data,q_ad,v_ad,tau_ad);
    ad_data.Minv.triangularView<Eigen::StrictlyLower>()
    = ad_data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    
    pinocchio::casadi::copy(ad_data.ddq_dq,cs_ddq_dq);
    pinocchio::casadi::copy(ad_data.ddq_dv,cs_ddq_dv);
    pinocchio::casadi::copy(ad_data.Minv,cs_ddq_dtau);
    
    casadi::Function eval_aba_derivatives_dq("eval_aba_derivatives_dq",
                                              casadi::SXVector {cs_q, cs_v, cs_tau},
                                              casadi::SXVector {cs_ddq_dq});
    
    casadi::DM ddq_dq_res_direct = eval_aba_derivatives_dq(casadi::DMVector {q_vec,v_vec,tau_vec})[0];
    Data::MatrixXs ddq_dq_res_direct_map = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dq_res_direct).data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dq_ref.isApprox(ddq_dq_res_direct_map));
    
    casadi::Function eval_aba_derivatives_dv("eval_aba_derivatives_dv",
                                              casadi::SXVector {cs_q, cs_v, cs_tau},
                                              casadi::SXVector {cs_ddq_dv});
    
    casadi::DM ddq_dv_res_direct = eval_aba_derivatives_dv(casadi::DMVector {q_vec,v_vec,tau_vec})[0];
    Data::MatrixXs ddq_dv_res_direct_map = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dv_res_direct).data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dv_ref.isApprox(ddq_dv_res_direct_map));
    
    casadi::Function eval_aba_derivatives_dtau("eval_aba_derivatives_dtau",
                                              casadi::SXVector {cs_q, cs_v, cs_tau},
                                              casadi::SXVector {cs_ddq_dtau});
    
    casadi::DM ddq_dtau_res_direct = eval_aba_derivatives_dtau(casadi::DMVector {q_vec,v_vec,tau_vec})[0];
    Data::MatrixXs ddq_dtau_res_direct_map = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dtau_res_direct).data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dtau_ref.isApprox(ddq_dtau_res_direct_map));
  }

BOOST_AUTO_TEST_SUITE_END()
