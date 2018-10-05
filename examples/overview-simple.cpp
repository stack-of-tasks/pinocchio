#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
int main()
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  Eigen::VectorXd q = se3::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  const Eigen::VectorXd & rnea = se3::rnea(model,data,q,v,a);
  std::cout << rnea.transpose() << std::endl;
}
