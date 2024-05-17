// Copyright 2023 Inria
// SPDX-License-Identifier: BSD-2-Clause

/*
 * In this short script, we show how to compute inverse dynamics (RNEA), i.e.
 * the vector of joint torques corresponding to a given motion.
 */

#include <iostream>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own
// directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char ** argv)
{
  using namespace pinocchio;

  // Change to your own URDF file here, or give a path as command-line argument
  const std::string urdf_filename = (argc <= 1)
                                      ? PINOCCHIO_MODEL_DIR
                                          + std::string("/example-robot-data/robots/"
                                                        "ur_description/urdf/ur5_robot.urdf")
                                      : argv[1];

  // Load the URDF model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  // Build a data frame associated with the model
  Data data(model);

  // Sample a random joint configuration, joint velocities and accelerations
  Eigen::VectorXd q = randomConfiguration(model);      // in rad for the UR5
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // in rad/s for the UR5
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // in rad/s² for the UR5

  // Computes the inverse dynamics (RNEA) for all the joints of the robot
  Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

  // Print out to the vector of joint torques (in N.m)
  std::cout << "Joint torques: " << data.tau.transpose() << std::endl;
  return 0;
}
