#include "pinocchio/math/multiprecision.hpp"

#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include <boost/multiprecision/cpp_dec_float.hpp>

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
  
  // Define Model and Data for multiprecision types
  typedef boost::multiprecision::cpp_dec_float_100 float_100;
  typedef ModelTpl<float_100> ModelMulti;
  typedef DataTpl<float_100> DataMulti;
  
  ModelMulti model_multi = model.cast<float_100>();
  DataMulti data_multi(model_multi);
  
  // Sample a random joint configuration as well as random joint velocity and acceleration
  ModelMulti::ConfigVectorType q_multi = randomConfiguration(model_multi);
  ModelMulti::TangentVectorType v_multi = ModelMulti::TangentVectorType::Random(model.nv);
  ModelMulti::TangentVectorType a_multi = ModelMulti::TangentVectorType::Random(model.nv);
  
  Model::ConfigVectorType q = q_multi.cast<double>();
  Model::TangentVectorType v = v_multi.cast<double>();
  Model::TangentVectorType a = a_multi.cast<double>();
  
  // Computes the inverse dynamics (aka RNEA)
  rnea(model, data, q, v, a);
  rnea(model_multi , data_multi , q_multi , v_multi , a_multi);
  
  // Get access to the joint torque with standard or multiprecision arithmetic and print sufficient decimals for both precisions
  std::cout << "Joint torque standard arithmetic:\n" << std::setprecision(std::numeric_limits<float_100>::max_digits10) << data.tau << std::endl;
  std::cout << "Joint torque multiprecision arithmetic:\n" << std::setprecision(std::numeric_limits<float_100>::max_digits10) << data_multi.tau << std::endl;
}

