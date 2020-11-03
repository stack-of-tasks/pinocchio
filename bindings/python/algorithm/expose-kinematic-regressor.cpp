//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeKinematicRegressor()
    {
      using namespace Eigen;

      bp::def("computeJointKinematicRegressor",
              (Data::Matrix6x (*)(const Model &, const Data &, const JointIndex, const ReferenceFrame, const SE3 &))&computeJointKinematicRegressor<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data","joint_id","reference_frame","placement"),
              "Computes the kinematic regressor that links the joint placements variations of the whole kinematic tree to the placement variation of the frame rigidly attached to the joint and given by its placement w.r.t. to the joint frame.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tjoint_id: index of the joint\n"
              "\treference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)\n"
              "\tplacement: relative placement to the joint frame\n");
      
      bp::def("computeJointKinematicRegressor",
              (Data::Matrix6x (*)(const Model &, const Data &, const JointIndex, const ReferenceFrame))&computeJointKinematicRegressor<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data","joint_id","reference_frame"),
              "Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree to the placement variation of the joint given as input.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tjoint_id: index of the joint\n"
              "\treference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)\n");

      bp::def("computeFrameKinematicRegressor",
              (Data::Matrix6x (*)(const Model &, Data &, const FrameIndex, const ReferenceFrame))&computeFrameKinematicRegressor<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data","frame_id","reference_frame"),
              "Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree to the placement variation of the frame given as input.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tframe_id: index of the frame\n"
              "\treference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)\n");
    }
    
  } // namespace python
} // namespace pinocchio
