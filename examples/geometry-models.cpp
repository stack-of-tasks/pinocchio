#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char ** argv)
{
  using namespace pinocchio;

  const std::string model_path = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/others/robots") : argv[1];
  const std::string mesh_dir = model_path;
  const std::string urdf_filename = model_path + "/ur_description/urdf/ur5_robot.urdf";

  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  GeometryModel collision_model;
  pinocchio::urdf::buildGeom(model, urdf_filename, COLLISION, collision_model, mesh_dir);
  GeometryModel visual_model;
  pinocchio::urdf::buildGeom(model, urdf_filename, VISUAL, visual_model, mesh_dir);
  std::cout << "model name: " << model.name << std::endl;

  // Create data required by the algorithms
  Data data(model);
  GeometryData collision_data(collision_model);
  GeometryData visual_data(visual_model);

  // Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  std::cout << "q: " << q.transpose() << std::endl;

  // Perform the forward kinematics over the kinematic tree
  forwardKinematics(model,data,q);

  // Update Geometry models
  updateGeometryPlacements(model, data, collision_model, collision_data);
  updateGeometryPlacements(model, data, visual_model, visual_data);

  // Print out the placement of each joint of the kinematic tree
  std::cout << "\nJoint placements:" << std::endl;
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;

  // Print out the placement of each collision geometry object
  std::cout << "\nCollision object placements:" << std::endl;
  for(GeomIndex geom_id = 0; geom_id < (GeomIndex)collision_model.ngeoms; ++geom_id)
    std::cout << geom_id << ": "
              << std::fixed << std::setprecision(2)
              << collision_data.oMg[geom_id].translation().transpose()
              << std::endl;

  // Print out the placement of each visual geometry object
  std::cout << "\nVisual object placements:" << std::endl;
  for(GeomIndex geom_id = 0; geom_id < (GeomIndex)visual_model.ngeoms; ++geom_id)
    std::cout << geom_id << ": "
              << std::fixed << std::setprecision(2)
              << visual_data.oMg[geom_id].translation().transpose()
              << std::endl;
}
