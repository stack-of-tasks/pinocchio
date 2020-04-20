//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeKinematics()
    {
      using namespace Eigen;
      bp::def("updateGlobalPlacements",
              &updateGlobalPlacements<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data"),
              "Updates the global placements of all the frames of the kinematic "
              "tree and store the results in data according to the relative placements of the joints.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n");
      
      bp::def("forwardKinematics",
              &forwardKinematics<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","data","q"),
              "Compute the global placements of all the joints of the kinematic "
              "tree and store the results in data.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n");
      
      bp::def("forwardKinematics",
              &forwardKinematics<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","data","q","v"),
              "Compute the global placements and local spatial velocities of all the joints of the kinematic "
              "tree and store the results in data.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n");
      
      bp::def("forwardKinematics",
              &forwardKinematics<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
              bp::args("model","data","q","v","a"),
              "Compute the global placements, local spatial velocities and spatial accelerations of all the joints of the kinematic "
              "tree and store the results in data.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\ta: the joint acceleration vector (size model.nv)\n");
    }
    
  } // namespace python
} // namespace pinocchio
