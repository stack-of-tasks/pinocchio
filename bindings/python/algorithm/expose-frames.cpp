//
// Copyright (c) 2015-2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace pinocchio
{
  namespace python
  {
    
    static Data::Matrix6x get_frame_jacobian_proxy(const Model & model,
                                                   Data & data,
                                                   const Model::FrameIndex frame_id,
                                                   ReferenceFrame rf)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      getFrameJacobian(model, data, frame_id, rf, J);
      
      return J;
    }
    
    static Data::Matrix6x frame_jacobian_proxy(const Model & model,
                                               Data & data,
                                               const Eigen::VectorXd & q,
                                               const Model::FrameIndex frame_id
                                               )
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      frameJacobian(model, data, q, frame_id, J);
  
      return J;
    }


    static Data::Matrix6x
    get_frame_jacobian_time_variation_proxy(const Model & model,
                                            Data & data,
                                            Model::FrameIndex jointId,
                                            ReferenceFrame rf)
    {
      Data::Matrix6x dJ(6,model.nv); dJ.setZero();
      getFrameJacobianTimeVariation(model,data,jointId,rf,dJ);
      
      return dJ;
    }

    static Data::Matrix6x frame_jacobian_time_variation_proxy(const Model & model,
                                                              Data & data,
                                                              const Eigen::VectorXd & q,
                                                              const Eigen::VectorXd & v,
                                                              const Model::FrameIndex frame_id,
                                                              ReferenceFrame rf
                                                              )
    {
      computeJointJacobiansTimeVariation(model,data,q,v);
      updateFramePlacements(model,data);
  
      return get_frame_jacobian_time_variation_proxy(model, data, frame_id, rf);
    }
    
    void exposeFramesAlgo()
    {
      using namespace Eigen;
      bp::def("updateFramePlacements",
              &updateFramePlacements<double,0,JointCollectionDefaultTpl>,
              bp::args("Model","Data"),
              "Computes the placements of all the operational frames according to the current joint placement stored in data"
              "and puts the results in data.");

      bp::def("updateFramePlacement",
              &updateFramePlacement<double,0,JointCollectionDefaultTpl>,
              bp::args("Model","Data","Operational frame ID (int)"),
              "Computes the placement of the given operational frames according to the current joint placement stored in data,"
              "puts the results in data and returns it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("getFrameVelocity",
              &getFrameVelocity<double,0,JointCollectionDefaultTpl>,
              bp::args("Model","Data","Operational frame ID (int)"),
              "Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system."
              "Fist or second order forwardKinematics should be called first.");

      bp::def("getFrameAcceleration",
              &getFrameAcceleration<double,0,JointCollectionDefaultTpl>,
              bp::args("Model","Data","Operational frame ID (int)"),
              "Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system."
              "Second order forwardKinematics should be called first.");
      
      bp::def("framesForwardKinematics",
              &framesForwardKinematics<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Update first the placement of the joints according to the given configuration value."
              "And computes the placements of all the operational frames"
              "and put the results in data.");
      
      bp::def("frameJacobian",
              &frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Operational frame ID (int)"),
              "Computes the Jacobian of the frame given by its ID."
              "The columns of the Jacobian are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.");
      
      bp::def("getFrameJacobian",
              &get_frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Operational frame ID (int)",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Computes the Jacobian of the frame given by its ID either in the local or the world frames."
              "The columns of the Jacobian are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.\n"
              "Be aware that computeJointJacobians and framesKinematics must have been called first.");

      bp::def("frameJacobianTimeVariation",
              (Data::Matrix6x (*)(const Model &, Data &, const Eigen::VectorXd &,const Eigen::VectorXd &, const Model::FrameIndex, ReferenceFrame))&frame_jacobian_time_variation_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",                       
                       "Operational frame ID (int)",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Computes the Jacobian Time Variation of the frame given by its ID either in the local or the world frames."
              "The columns of the Jacobian time variation are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.");      

      bp::def("getFrameJacobianTimeVariation",get_frame_jacobian_time_variation_proxy,
              bp::args("Model, the model of the kinematic tree",
                       "Data, the data associated to the model where the results are stored",
                       "Frame ID, the index of the frame.",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Returns the Jacobian time variation of a specific frame (specified by Frame ID) expressed either in the world or the local frame."
              "You have to call computeJointJacobiansTimeVariation and framesKinematics first."
              "If rf is set to LOCAL, it returns the jacobian time variation associated to the frame index. Otherwise, it returns the jacobian time variation of the frame coinciding with the world frame.");

    }
  } // namespace python
} // namespace pinocchio
