#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"

int main(int argc, const char**argv)
{
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];

#ifndef NDEBUG
  std::cout << "Parse filename \"" << filename << "\"" << std::endl;
#endif
  se3::Model model = se3::urdf::buildModel(filename);

  return 0;
}
