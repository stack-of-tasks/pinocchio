#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char ** argv)
{
  using namespace pinocchio;
  
  // You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/others/robots/ur_description/urdf/ur5_robot.urdf") : argv[1];
  
  // Load the URDF model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  
  // Build a data related to model
  Data data(model);
  
  // Sample a random joint configuration as well as random joint velocity and acceleration
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd(model.nv);
  
  // Allocate result container
  Eigen::MatrixXd djoint_torque_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_torque_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_torque_da = Eigen::MatrixXd::Zero(model.nv,model.nv);
  
  // Computes the inverse dynamics (RNEA) derivatives for all the joints of the robot
  computeRNEADerivatives(model, data, q, v, a, djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
  
  // Get access to the joint torque
  std::cout << "Joint torque: " << data.tau.transpose() << std::endl;
}
