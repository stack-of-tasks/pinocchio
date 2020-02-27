//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/codegen/cppadcg.hpp" // this file should be included first before all the others!
#include "pinocchio/algorithm/crba.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"

#include <iostream>

int main(int argc, const char ** argv)
{
  using namespace pinocchio;
  using namespace Eigen;
  
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/ur_description/urdf/ur5_robot.urdf");
  if(argc>1) filename = argv[1];
  
  std::cout << "Opening file: " << filename << std::endl;
  
  // Load the model
  Model model; pinocchio::urdf::buildModel(filename, model);
  
  CodeGenCRBA<double> crba_code_gen(model);

  // Generate the lib if it does not exist and load it afterwards.
  crba_code_gen.initLib();
  crba_code_gen.loadLib();

  // Use it with a random configuration samples in the bounds of the joint limits
  VectorXd q = randomConfiguration(model);
  crba_code_gen.evalFunction(q);

  // Retrieve the result
  MatrixXd & M = crba_code_gen.M;

  // And make it symmetric if needed
  M.template triangularView<Eigen::StrictlyLower>() = M.transpose().template triangularView<Eigen::StrictlyLower>();

  // You can check the result with the classic CRBA
  Data data_check(model);
  crba(model,data_check,q);

  data_check.M.triangularView<Eigen::StrictlyLower>() = data_check.M.transpose().triangularView<Eigen::StrictlyLower>();

  const MatrixXd & M_check = data_check.M;
  if(M_check.isApprox(M)) {
    std::cout << "Super! The two results are the same." << std::endl;
    return 0;
  }
  else {
    std::cout << "Not Super! The results do not match." << std::endl;
    return -1;
  }

}
