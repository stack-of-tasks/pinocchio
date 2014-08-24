#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"

int main(int argc, const char**argv)
{
  std::string filename = "/home/nmansard/src/rbdl/rbdl_evaluate_performances/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];

  se3::Model model = se3::buildModel(filename);

  return 0;
}
