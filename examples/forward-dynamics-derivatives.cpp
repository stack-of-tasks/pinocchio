#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

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
  
  // Sample a random joint configuration as well as random joint velocity and torque
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd(model.nv);
  
  // Allocate result container
  Eigen::MatrixXd djoint_acc_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_acc_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_acc_dtau = Eigen::MatrixXd::Zero(model.nv,model.nv);
  
  // Computes the forward dynamics (ABA) derivatives for all the joints of the robot
  computeABADerivatives(model, data, q, v, tau, djoint_acc_dq, djoint_acc_dv, djoint_acc_dtau);
  
  // Get access to the joint acceleration
  std::cout << "Joint acceleration: " << data.ddq.transpose() << std::endl;
}
