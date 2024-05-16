#include "pinocchio/codegen/cppadcg.hpp"

#include <iosfwd>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/sample-models.hpp>
#include <pinocchio/algorithm/rnea.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main(void)
{
  // use a special object for source code generation
  using CGD = CG<double>;
  using ADCG = AD<CGD>;
  using namespace pinocchio;

  typedef ModelTpl<ADCG> ADModel;
  typedef DataTpl<ADCG> ADData;
  typedef Eigen::Matrix<ADCG, Eigen::Dynamic, 1> ADVectorXs;

  Model model;
  buildModels::humanoidRandom(model);
  ADModel ad_model = model.cast<ADCG>();
  ADData ad_data(ad_model);

  /***************************************************************************
   *                               the model
   **************************************************************************/

  // independent variable vector

  ADVectorXs ad_X, ad_Y;
  ad_X = ADVectorXs(ad_model.nq + ad_model.nv);
  ad_Y = ADVectorXs(ad_model.nv);
  ADVectorXs zeros = ADVectorXs::Zero(ad_model.nv);
  Independent(ad_X);
  pinocchio::rnea(
    ad_model, ad_data, ad_X.head(ad_model.nq), ad_X.segment(ad_model.nq, ad_model.nv), zeros);
  ad_Y = ad_data.tau;
  ADFun<CGD> fun(ad_X, ad_Y); // the model tape

  /***************************************************************************
   *                        Generate the C source code
   **************************************************************************/

  /**
   * start the special steps for source code generation for a Jacobian
   */
  CodeHandler<double> handler;
  CppAD::vector<CGD> indVars(65);
  handler.makeVariables(indVars);

  CppAD::vector<CGD> jac = fun.Jacobian(indVars);

  LanguageC<double> langC("double");
  LangCDefaultVariableNameGenerator<double> nameGen;

  std::ostringstream code;
  handler.generateCode(code, langC, jac, nameGen);
  std::cout << code.str();
}
