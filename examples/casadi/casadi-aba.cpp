#include "pinocchio/autodiff/casadi.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  typedef double Scalar;
  typedef ::casadi::SX ADScalar;

  typedef ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  // Create a random humanoid model
  Model model;
  buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  // Pick up random configuration, velocity and acceleration vectors.
  Eigen::VectorXd q(model.nq);
  q = randomConfiguration(model);
  Eigen::VectorXd v(Eigen::VectorXd::Random(model.nv));
  Eigen::VectorXd tau(Eigen::VectorXd::Random(model.nv));

  // Create CasADi model and data from model
  typedef ADModel::ConfigVectorType ConfigVectorAD;
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  // Create symbolic CasADi vectors
  ::casadi::SX cs_q = ::casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);

  ::casadi::SX cs_v = ::casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);

  ::casadi::SX cs_tau = ::casadi::SX::sym("tau", model.nv);
  TangentVectorAD tau_ad(model.nv);
  tau_ad =
    Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

  // Build CasADi function
  aba(ad_model, ad_data, q_ad, v_ad, tau_ad);
  ::casadi::SX a_ad(model.nv, 1);

  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    a_ad(k) = ad_data.ddq[k];

  ::casadi::Function eval_aba(
    "eval_aba", ::casadi::SXVector{cs_q, cs_v, cs_tau}, ::casadi::SXVector{a_ad});

  // Evaluate CasADi expression with real value
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<Eigen::VectorXd>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<Eigen::VectorXd>(v_vec.data(), model.nv, 1) = v;

  std::vector<double> tau_vec((size_t)model.nv);
  Eigen::Map<Eigen::VectorXd>(tau_vec.data(), model.nv, 1) = tau;

  ::casadi::DM a_casadi_res = eval_aba(::casadi::DMVector{q_vec, v_vec, tau_vec})[0];
  Data::TangentVectorType a_casadi_vec = Eigen::Map<Data::TangentVectorType>(
    static_cast<std::vector<double>>(a_casadi_res).data(), model.nv, 1);

  // Eval ABA using classic Pinocchio model
  pinocchio::aba(model, data, q, v, tau);

  // Print both results
  std::cout << "pinocchio double:\n" << "\ta = " << data.ddq.transpose() << std::endl;
  std::cout << "pinocchio CasADi:\n" << "\ta = " << a_casadi_vec.transpose() << std::endl;
}
