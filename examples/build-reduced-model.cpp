#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <iostream>
#include <algorithm>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

template<typename T>
bool is_in_vector(const std::vector<T> & vector, const T & elt)
{
  return vector.end() != std::find(vector.begin(), vector.end(), elt);
}

int main(int argc, char ** argv)
{
  using namespace pinocchio;

  // You should change here to set up your own URDF file or just pass it as an argument of this
  // example.
  const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf")
                : argv[1];

  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  // Create a list of joint to lock
  std::vector<std::string> list_of_joints_to_lock_by_name;
  list_of_joints_to_lock_by_name.push_back("elbow_joint");
  list_of_joints_to_lock_by_name.push_back("wrist_3_joint"); // It can be in the wrong order
  list_of_joints_to_lock_by_name.push_back("wrist_2_joint");
  list_of_joints_to_lock_by_name.push_back("blabla"); // Joint not in the model

  // Print the list of joints to remove + retrieve the joint id
  std::vector<JointIndex> list_of_joints_to_lock_by_id;
  for (std::vector<std::string>::const_iterator it = list_of_joints_to_lock_by_name.begin();
       it != list_of_joints_to_lock_by_name.end(); ++it)
  {
    const std::string & joint_name = *it;
    if (model.existJointName(joint_name)) // do not consider joint that are not in the model
      list_of_joints_to_lock_by_id.push_back(model.getJointId(joint_name));
    else
      std::cout << "joint: " << joint_name << " does not belong to the model" << std::endl;
  }

  // Sample any random configuration
  Eigen::VectorXd q_rand = randomConfiguration(model);
  //  std::cout << "q_rand: " << q_rand.transpose() << std::endl;
  // But should be also a neutral configuration
  Eigen::VectorXd q_neutral = neutral(model);
  PINOCCHIO_UNUSED_VARIABLE(q_neutral);
  //  std::cout << "q_neutral: " << q_neutral.transpose() << std::endl;

  std::cout << "\n\nFIRST CASE: BUILD A REDUCED MODEL FROM A LIST OF JOINT TO LOCK" << std::endl;
  // Build the reduced model from the list of lock joints
  Model reduced_model = buildReducedModel(model, list_of_joints_to_lock_by_id, q_rand);

  // Print the list of joints in the original model
  std::cout << "List of joints in the original model:" << std::endl;
  for (JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
    std::cout << "\t- " << model.names[joint_id] << std::endl;

  // Print the list of joints in the reduced model
  std::cout << "List of joints in the reduced model:" << std::endl;
  for (JointIndex joint_id = 1; joint_id < reduced_model.joints.size(); ++joint_id)
    std::cout << "\t- " << reduced_model.names[joint_id] << std::endl;

  std::cout << "\n\nSECOND CASE: BUILD A REDUCED MODEL FROM A LIST OF JOINT TO KEEP UNLOCKED"
            << std::endl;
  // The same thing, but this time with an input list of joint to keep
  std::vector<std::string> list_of_joints_to_keep_unlocked_by_name;
  list_of_joints_to_keep_unlocked_by_name.push_back("shoulder_pan_joint");
  list_of_joints_to_keep_unlocked_by_name.push_back("shoulder_lift_joint");
  list_of_joints_to_keep_unlocked_by_name.push_back("wrist_1_joint");

  std::vector<JointIndex> list_of_joints_to_keep_unlocked_by_id;
  for (std::vector<std::string>::const_iterator it =
         list_of_joints_to_keep_unlocked_by_name.begin();
       it != list_of_joints_to_keep_unlocked_by_name.end(); ++it)
  {
    const std::string & joint_name = *it;
    if (model.existJointName(joint_name))
      list_of_joints_to_keep_unlocked_by_id.push_back(model.getJointId(joint_name));
    else
      std::cout << "joint: " << joint_name << " does not belong to the model";
  }

  // Transform the list into a list of joints to lock
  list_of_joints_to_lock_by_id.clear();
  for (JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
  {
    const std::string joint_name = model.names[joint_id];
    if (is_in_vector(list_of_joints_to_keep_unlocked_by_name, joint_name))
      continue;
    else
    {
      list_of_joints_to_lock_by_id.push_back(joint_id);
    }
  }

  // Build the reduced model from the list of lock joints
  Model reduced_model2 = buildReducedModel(model, list_of_joints_to_lock_by_id, q_rand);

  // Print the list of joints in the second reduced model
  std::cout << "List of joints in the second reduced model:" << std::endl;
  for (JointIndex joint_id = 1; joint_id < reduced_model2.joints.size(); ++joint_id)
    std::cout << "\t- " << reduced_model2.names[joint_id] << std::endl;
}
