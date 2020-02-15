#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <string>

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  Model model;
  pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);

  std::cout << "model.nq: " << model.nq << std::endl;
  std::cout << "model.nv: " << model.nv << std::endl;

  return 0;
}
