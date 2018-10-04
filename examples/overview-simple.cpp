#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"

int main()
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  Eigen::VectorXd q = se3::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

  rnea(model,data,q,v,a);
}
  
