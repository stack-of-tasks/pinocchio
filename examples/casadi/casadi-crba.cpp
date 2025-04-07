#include "pinocchio/autodiff/casadi.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/crba.hpp"
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

  // Create CasADi model and data from model
  typedef ADModel::ConfigVectorType ConfigVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  // Create symbolic CasADi vectors
  ::casadi::SX cs_q = ::casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);

  // Build CasADi function
  crba(ad_model, ad_data, q_ad, pinocchio::Convention::WORLD);
  ad_data.M.triangularView<Eigen::StrictlyLower>() =
    ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ::casadi::SX M_ad(model.nv, model.nv);
  for (Eigen::DenseIndex j = 0; j < model.nv; ++j)
  {
    for (Eigen::DenseIndex i = 0; i < model.nv; ++i)
    {
      M_ad(i, j) = ad_data.M(i, j);
    }
  }

  ::casadi::Function eval_crba("eval_crba", ::casadi::SXVector{cs_q}, ::casadi::SXVector{M_ad});

  // Evaluate CasADi expression with real value
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<Eigen::VectorXd>(q_vec.data(), model.nq, 1) = q;

  ::casadi::DM M_res = eval_crba(::casadi::DMVector{q_vec})[0];
  Data::MatrixXs M_mat =
    Eigen::Map<Data::MatrixXs>(static_cast<std::vector<double>>(M_res).data(), model.nv, model.nv);

  // Eval CRBA using classic Pinocchio model
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  // Print both results
  std::cout << "pinocchio double:\n" << "\tM =\n" << data.M << std::endl;
  std::cout << "pinocchio CasADi:\n" << "\tM =\n" << M_mat.transpose() << std::endl;
}
