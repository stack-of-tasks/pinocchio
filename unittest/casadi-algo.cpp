//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/autodiff/casadi.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_jacobian)
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
  
  typedef ADModel::ConfigVectorType ConfigVectorAD;
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  
  Model::Index joint_id = model.existJointName("rarm2")?model.getJointId("rarm2"):(Model::Index)(model.njoints-1);
  Data::Matrix6x jacobian_local(6,model.nv), jacobian_world(6,model.nv);
  jacobian_local.setZero(); jacobian_world.setZero();
  
  BOOST_CHECK(jacobian_local.isZero() && jacobian_world.isZero());
  
  pinocchio::computeJointJacobians(model,data,q);
  pinocchio::getJointJacobian(model,data,joint_id,pinocchio::WORLD,jacobian_world);
  pinocchio::getJointJacobian(model,data,joint_id,pinocchio::LOCAL,jacobian_local);
  
  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  for(Eigen::DenseIndex k = 0; k < model.nq; ++k)
  {
    q_ad[k] = cs_q(k);
  }
  std::cout << "q =\n " << q_ad << std::endl;
  
  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_ad[k] = cs_v(k);
  }
  std::cout << "v =\n " << v_ad << std::endl;
  
  pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad);
  typedef pinocchio::MotionTpl<ADScalar> MotionAD;
  MotionAD & v_local = ad_data.v[(size_t)joint_id];
  MotionAD v_world = ad_data.oMi[(size_t)joint_id].act(v_local);
  
  casadi::SX cs_v_local(6,1), cs_v_world(6,1);
  for(Eigen::DenseIndex k = 0; k < 6; ++k)
  {
    cs_v_local(k) = v_local.toVector()[k];
    cs_v_world(k) = v_world.toVector()[k];
  }
  std::cout << "v_local = " << cs_v_local << std::endl;
  std::cout << "v_world = " << cs_v_world << std::endl;

  casadi::Function eval_velocity_local("eval_velocity_local",
                                       casadi::SXVector {cs_q, cs_v},
                                       casadi::SXVector {cs_v_local});
  std::cout << "eval_velocity_local = " << eval_velocity_local << std::endl;

  casadi::Function eval_velocity_world("eval_velocity_world",
                                       casadi::SXVector {cs_q, cs_v},
                                       casadi::SXVector {cs_v_world});
  std::cout << "eval_velocity_world = " << eval_velocity_world << std::endl;

  casadi::SX dv_dv_local = jacobian(cs_v_local, cs_v);
  casadi::Function eval_jacobian_local("eval_jacobian_local",
                                       casadi::SXVector {cs_q,cs_v},
                                       casadi::SXVector {dv_dv_local});
  std::cout << "eval_jacobian_local = " << eval_jacobian_local << std::endl;
  
  casadi::SX dv_dv_world = jacobian(cs_v_world, cs_v);
  casadi::Function eval_jacobian_world("eval_jacobian_world",
                                       casadi::SXVector {cs_q,cs_v},
                                       casadi::SXVector {dv_dv_world});
  
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
  
  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v;
  
  casadi::DMVector v_local_res = eval_velocity_local(casadi::DMVector {q_vec,v_vec});
  casadi::DMVector J_local_res = eval_jacobian_local(casadi::DMVector {q_vec,v_vec});
  std::cout << "J_local_res:" << J_local_res << std::endl;
  
  std::vector<double> v_local_vec(static_cast< std::vector<double> >(v_local_res[0]));
  BOOST_CHECK((jacobian_local*v).isApprox(Eigen::Map<pinocchio::Motion::Vector6>(v_local_vec.data())));
  
  casadi::DMVector v_world_res = eval_velocity_world(casadi::DMVector {q_vec,v_vec});
  casadi::DMVector J_world_res = eval_jacobian_world(casadi::DMVector {q_vec,v_vec});
  
  std::vector<double> v_world_vec(static_cast< std::vector<double> >(v_world_res[0]));
  BOOST_CHECK((jacobian_world*v).isApprox(Eigen::Map<pinocchio::Motion::Vector6>(v_world_vec.data())));
  
  Data::Matrix6x J_local_mat(6,model.nv), J_world_mat(6,model.nv);
  
  std::vector<double> J_local_vec(static_cast< std::vector<double> >(J_local_res[0]));
  J_local_mat = Eigen::Map<Data::Matrix6x>(J_local_vec.data(),6,model.nv);
  BOOST_CHECK(jacobian_local.isApprox(J_local_mat));

  std::vector<double> J_world_vec(static_cast< std::vector<double> >(J_world_res[0]));
  J_world_mat = Eigen::Map<Data::Matrix6x>(J_world_vec.data(),6,model.nv);
  BOOST_CHECK(jacobian_world.isApprox(J_world_mat));
}
  
  BOOST_AUTO_TEST_CASE(test_fk)
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
    
    pinocchio::forwardKinematics(model,data,q);
    
    typedef ADModel::ConfigVectorType ConfigVectorAD;
    typedef ADModel::TangentVectorType TangentVectorAD;
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);
    
    casadi::SX cs_q = casadi::SX::sym("q", model.nq);
    ConfigVectorAD q_ad(model.nq);
    pinocchio::casadi::copy(cs_q,q_ad);
    
    casadi::SX cs_v = casadi::SX::sym("v", model.nv);
    TangentVectorAD v_ad(model.nv);
    pinocchio::casadi::copy(cs_v,v_ad);
    
    casadi::SX cs_a = casadi::SX::sym("a", model.nv);
    TangentVectorAD a_ad(model.nv);
    pinocchio::casadi::copy(cs_a,a_ad);
    
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad, a_ad);
    pinocchio::updateGlobalPlacements(ad_model, ad_data);
    pinocchio::updateFramePlacements(ad_model, ad_data);
//    typedef pinocchio::MotionTpl<ADScalar> MotionAD;
  }
  
BOOST_AUTO_TEST_CASE(test_rnea)
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
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
  
  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
  
  casadi::SX cs_a = casadi::SX::sym("a", model.nv);
  TangentVectorAD a_ad(model.nv);
  a_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_a).data(),model.nv,1);
  
  rnea(ad_model,ad_data,q_ad,v_ad,a_ad);
  casadi::SX tau_ad(model.nv,1);
  //    Eigen::Map<TangentVectorAD>(tau_ad->data(),model.nv,1)
  //    = ad_data.tau;
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
    tau_ad(k) = ad_data.tau[k];
  casadi::Function eval_rnea("eval_rnea",
                             casadi::SXVector {cs_q, cs_v, cs_a},
                             casadi::SXVector {tau_ad});
  
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
  
  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v;
  
  std::vector<double> a_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(a_vec.data(),model.nv,1) = a;
  casadi::DM tau_res = eval_rnea(casadi::DMVector {q_vec,v_vec,a_vec})[0];
  std::cout << "tau_res = " << tau_res << std::endl;
  Data::TangentVectorType tau_vec = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(tau_res).data(),model.nv,1);
  
  BOOST_CHECK(data.tau.isApprox(tau_vec));
}
  
BOOST_AUTO_TEST_CASE(test_crba)
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
  
  pinocchio::crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::rnea(model,data,q,v,a);
  
  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
  
  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
  
  casadi::SX cs_a = casadi::SX::sym("a", model.nv);
  TangentVectorAD a_ad(model.nv);
  a_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_a).data(),model.nv,1);
  
  // RNEA
  rnea(ad_model,ad_data,q_ad,v_ad,a_ad);
  casadi::SX cs_tau(model.nv,1);
  //    Eigen::Map<TangentVectorAD>(tau_ad->data(),model.nv,1)
  //    = ad_data.tau;
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
    cs_tau(k) = ad_data.tau[k];
  casadi::Function eval_rnea("eval_rnea",
                             casadi::SXVector {cs_q, cs_v, cs_a},
                             casadi::SXVector {cs_tau});
  // CRBA
  crba(ad_model,ad_data,q_ad);
  ad_data.M.triangularView<Eigen::StrictlyLower>()
  = ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();
  casadi::SX M_ad(model.nv,model.nv);
  for(Eigen::DenseIndex j = 0; j < model.nv; ++j)
  {
    for(Eigen::DenseIndex i = 0; i < model.nv; ++i)
    {
      M_ad(i,j) = ad_data.M(i,j);
    }
  }
  
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
  
  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v;
  
  std::vector<double> a_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(a_vec.data(),model.nv,1) = a;
  
  casadi::Function eval_crba("eval_crba",
                             casadi::SXVector {cs_q},
                             casadi::SXVector {M_ad});
  casadi::DM M_res = eval_crba(casadi::DMVector {q_vec})[0];
  Data::MatrixXs M_mat = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(M_res).data(),
                                                    model.nv,model.nv);
  
  BOOST_CHECK(data.M.isApprox(M_mat));
  
  casadi::SX dtau_da = jacobian(cs_tau,cs_a);
  casadi::Function eval_dtau_da("eval_dtau_da",
                                casadi::SXVector {cs_q,cs_v,cs_a},
                                casadi::SXVector {dtau_da});
  casadi::DM dtau_da_res = eval_dtau_da(casadi::DMVector {q_vec, v_vec, a_vec})[0];
  Data::MatrixXs dtau_da_mat = Eigen::Map<Data::MatrixXs>(static_cast< std::vector<double> >(dtau_da_res).data(),
                                                          model.nv,model.nv);
  BOOST_CHECK(data.M.isApprox(dtau_da_mat));
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
    ConfigVectorAD q_ad(model.nq);
    q_ad = Eigen::Map<ConfigVectorAD>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
    
    casadi::SX cs_v = casadi::SX::sym("v", model.nv);
    TangentVectorAD v_ad(model.nv);
    v_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
    
    casadi::SX cs_tau = casadi::SX::sym("tau", model.nv);
    TangentVectorAD tau_ad(model.nv);
    tau_ad = Eigen::Map<TangentVectorAD>(static_cast< std::vector<ADScalar> >(cs_tau).data(),model.nv,1);
    
    // ABA
    aba(ad_model,ad_data,q_ad,v_ad,tau_ad);
    casadi::SX cs_ddq(model.nv,1);
    for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
      cs_ddq(k) = ad_data.ddq[k];
    casadi::Function eval_aba("eval_aba",
                              casadi::SXVector {cs_q, cs_v, cs_tau},
                              casadi::SXVector {cs_ddq});

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q;
    
    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v;
    
    std::vector<double> tau_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(tau_vec.data(),model.nv,1) = tau;
    
    casadi::DM ddq_res = eval_aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
    Data::TangentVectorType ddq_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res).data(),
                                                            model.nv,1);

    BOOST_CHECK(ddq_mat.isApprox(data.ddq));
  }

BOOST_AUTO_TEST_SUITE_END()
