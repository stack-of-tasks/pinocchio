#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own modeldirectory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;
  const std::string robots_model_path = PINOCCHIO_MODEL_DIR + std::string("/others/robots/");
  
  // You should change here to set up your own URDF file
  const std::string urdf_filename = robots_model_path + std::string("talos_data/urdf/talos_reduced.urdf");
  // You should change here to set up your own SRDF file
  const std::string srdf_filename = robots_model_path + std::string("talos_data/srdf/talos.srdf");
  
  // Load the URDF model contained in urdf_filename
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  
  // Build the data associated to the model
  Data data(model);
  
  // Load the geometries associated to model which are contained in the URDF file
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, robots_model_path);
  
  // Add all possible collision pairs and remove the ones collected in the SRDF file
  geom_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename);
  
  // Build the data associated to the geom_model
  GeometryData geom_data(geom_model); // contained the intermediate computations, like the placement of all the geometries with respect to the world frame
  
  // Load the reference configuration of the robots (this one should be collision free)
  pinocchio::srdf::loadReferenceConfigurations(model,srdf_filename); // the reference configuratio stored in the SRDF file is called half_sitting
  
  const Model::ConfigVectorType & q = model.referenceConfigurations["half_sitting"];
  
  // And test all the collision pairs
  computeCollisions(model,data,geom_model,geom_data,q);
  
  // Print the status of all the collision pairs
  for(size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
  {
    const CollisionPair & cp = geom_model.collisionPairs[k];
    const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
    
    std::cout << "collision pair: " << cp.first << " , " << cp.second << " - collision: ";
    std::cout << (cr.isCollision() ? "yes" : "no") << std::endl;
  }
  
  // If you want to stop as soon as a collision is encounter, just add false for the final default argument stopAtFirstCollision
  computeCollisions(model,data,geom_model,geom_data,q,true);
  
  // And if you to check only one collision pair, e.g. the third one, at the neutral element of the Configuration Space of the robot
  const PairIndex pair_id = 2;
  const Model::ConfigVectorType q_neutral = neutral(model);
  updateGeometryPlacements(model, data, geom_model, geom_data, q_neutral); // performs a forward kinematics over the whole kinematics model + update the placement of all the geometries contained inside geom_model
  computeCollision(geom_model, geom_data, pair_id);
  
  return 0;
}
