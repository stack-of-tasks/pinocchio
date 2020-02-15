#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <iostream>

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  Model model;
  pinocchio::buildModels::humanoid(model);

  Model::VectorXs q = randomConfiguration(model);
  Model::VectorXs v = Model::VectorXs::Random(model.nv);
  Model::VectorXs a = Model::VectorXs::Random(model.nv);

  Data data(model);
  rnea(model,data,q,v,a);

  return 0;
}
