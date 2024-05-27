#include "pinocchio/autodiff/cppad.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/sample-models.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

using namespace CppAD;

int main(void)
{
  using CppAD::AD;
  using namespace pinocchio;

  typedef ModelTpl<AD<double>> ADModel;
  typedef DataTpl<AD<double>> ADData;
  typedef Eigen::Matrix<AD<double>, Eigen::Dynamic, 1> ADVectorXs;

  Model model;
  buildModels::humanoidRandom(model);
  ADModel ad_model = model.cast<AD<double>>();
  ADData ad_data(ad_model);

  int nq = ad_model.nq;
  int nv = ad_model.nv;

  /***************************************************************************
   *                               the model
   **************************************************************************/
  ADVectorXs ad_Q, ad_V, ad_A, ad_Y;

  ad_Q = ADVectorXs(nq);
  ad_Q = pinocchio::neutral(ad_model);
  ad_V = ADVectorXs(nv);
  ad_V.setOnes();
  ad_A = ADVectorXs(nv);
  ad_A.setZero();

  // independent variable vector
  Independent(ad_V);
  ad_Y = pinocchio::rnea(ad_model, ad_data, ad_Q, ad_V, ad_A);
  // the model tape
  ADFun<double> fun(ad_V, ad_Y);

  Eigen::VectorXd dv = Eigen::VectorXd(nv);
  dv.setOnes();
  Eigen::VectorXd dtau_dv = fun.Jacobian(dv);

  /***************************************************************************
   *                    comparison to analytical derivatives
   **************************************************************************/
  Data data(model);
  Eigen::VectorXd Q(nq), V(nv), A(nv);
  Q = pinocchio::neutral(model);
  V.setOnes();
  A.setZero();
  computeRNEADerivatives(model, data, Q, V, A);

  std::cout << "Analytical derivatives" << std::endl;
  std::cout << data.dtau_dv << std::endl;
  std::cout << "Automatic differentiation" << std::endl;
  std::cout << dtau_dv.reshaped(nv, nv).transpose() << std::endl;

  if (data.dtau_dv.isApprox(dtau_dv.reshaped(nv, nv).transpose()))
  {
    std::cout << "Comparison from automatic differention to analytical derivatives succesful"
              << std::endl;
  }
  else
  {
    std::cout << "Comparison from automatic differention to analytical derivatives NOT succesful"
              << std::endl;
  }
}
