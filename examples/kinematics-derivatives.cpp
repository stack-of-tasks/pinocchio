#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"

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
  
  // Computes the kinematics derivatives for all the joints of the robot
  computeForwardKinematicsDerivatives(model, data, q, v, a);
  
  // Retrieve the kinematics derivatives of a specific joint, expressed in the LOCAL frame of the joints.
  JointIndex joint_id = (JointIndex)(model.njoints-1);
  Data::Matrix6x v_partial_dq(6,model.nv), a_partial_dq(6,model.nv), a_partial_dv(6,model.nv), a_partial_da(6,model.nv);
  v_partial_dq.setZero();
  a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  getJointAccelerationDerivatives(model,data,joint_id,LOCAL,v_partial_dq,
                                  a_partial_dq,a_partial_dv,a_partial_da);
  
  // Remark: we are not directly computing the quantity v_partial_dv as it is also equal to a_partial_da.
  
  // But we can also expressed the same quantities in the frame centered on the end-effector joint, but expressed in the axis aligned with the world frame.
  getJointAccelerationDerivatives(model,data,joint_id,WORLD,v_partial_dq,
                                  a_partial_dq,a_partial_dv,a_partial_da);
  
}
