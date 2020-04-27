//
// Copyright (c) 2015-2020 CNRS INRIA
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
    
    static Data::Matrix6x compute_frame_jacobian_proxy(const Model & model,
                                                       Data & data,
                                                       const Eigen::VectorXd & q,
                                                       Model::FrameIndex frame_id)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      computeFrameJacobian(model, data, q, frame_id, J);
  
      return J;
    }
    
    static Data::Matrix6x compute_frame_jacobian_proxy(const Model & model,
                                                       Data & data,
                                                       const Eigen::VectorXd & q,
                                                       Model::FrameIndex frame_id,
                                                       ReferenceFrame reference_frame)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      computeFrameJacobian(model, data, q, frame_id, reference_frame, J);
  
      return J;
    }

    static Data::Matrix6x get_frame_jacobian_time_variation_proxy(const Model & model,
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
                                                              const ReferenceFrame rf)
    {
      computeJointJacobiansTimeVariation(model,data,q,v);
      updateFramePlacements(model,data);
  
      return get_frame_jacobian_time_variation_proxy(model, data, frame_id, rf);
    }
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(getFrameVelocity_overload, (getFrameVelocity<double,0,JointCollectionDefaultTpl>), 3, 4)
    BOOST_PYTHON_FUNCTION_OVERLOADS(getFrameAcceleration_overload, (getFrameAcceleration<double,0,JointCollectionDefaultTpl>), 3, 4)
    BOOST_PYTHON_FUNCTION_OVERLOADS(getFrameClassicalAcceleration_overload, (getFrameClassicalAcceleration<double,0,JointCollectionDefaultTpl>), 3, 4)

    void exposeFramesAlgo()
    {
      using namespace Eigen;
      
      bp::def("updateFramePlacements",
              &updateFramePlacements<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data"),
              "Computes the placements of all the operational frames according to the current joint placement stored in data"
              "and puts the results in data.");

      bp::def("updateFramePlacement",
              &updateFramePlacement<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data","frame_id"),
              "Computes the placement of the given operational frame (frame_id) according to the current joint placement stored in data, stores the results in data and returns it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("getFrameVelocity",
              &getFrameVelocity<double,0,JointCollectionDefaultTpl>,
              getFrameVelocity_overload(
                bp::args("model","data","frame_id","reference_frame"),
                "Returns the spatial velocity of the frame expressed in the coordinate system given by reference_frame.\n"
                "forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial velocity stored in data.v"));

      bp::def("getFrameAcceleration",
              &getFrameAcceleration<double,0,JointCollectionDefaultTpl>,
              getFrameAcceleration_overload(
                bp::args("model","data","frame_id","reference_frame"),
                "Returns the spatial acceleration of the frame expressed in the coordinate system given by reference_frame.\n"
                "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a ."));

      bp::def("getFrameClassicalAcceleration",
              &getFrameClassicalAcceleration<double,0,JointCollectionDefaultTpl>,
              getFrameClassicalAcceleration_overload(
                bp::args("model","data","frame_id","reference_frame"),
                "Returns the \"classical\" acceleration of the frame expressed in the coordinate system given by reference_frame.\n"
                "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a ."));

      bp::def("framesForwardKinematics",
              &framesForwardKinematics<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","data","q"),
              "Calls first the forwardKinematics(model,data,q) and then update the Frame placement quantities (data.oMf).");
      
      bp::def("computeFrameJacobian",
              (Data::Matrix6x (*)(const Model &, Data &, const Eigen::VectorXd &, Model::FrameIndex, ReferenceFrame))&compute_frame_jacobian_proxy,
              bp::args("model","data","q","frame_id","reference_frame"),
              "Computes the Jacobian of the frame given by its frame_id in the coordinate system given by reference_frame.\n");
      
      bp::def("computeFrameJacobian",
              (Data::Matrix6x (*)(const Model &, Data &, const Eigen::VectorXd &, Model::FrameIndex))&compute_frame_jacobian_proxy,
              bp::args("model","data","q","frame_id"),
              "Computes the Jacobian of the frame given by its frame_id.\n"
              "The columns of the Jacobian are expressed in the coordinates system of the Frame itself.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the joint velocity.");
      
      bp::def("getFrameJacobian",
              &get_frame_jacobian_proxy,
              bp::args("model","data","frame_id","reference_frame"),
              "Computes the Jacobian of the frame given by its ID either in the local or the world frames.\n"
              "The columns of the Jacobian are expressed in the LOCAL frame coordinates system.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the joint velocity.\n"
              "computeJointJacobians(model,data,q) and updateFramePlacements(model,data) must have been called first.");

      bp::def("frameJacobianTimeVariation",&frame_jacobian_time_variation_proxy,
              bp::args("model","data","q","v","frame_id","reference_frame"),
              "Computes the Jacobian Time Variation of the frame given by its frame_id either in the reference frame provided by reference_frame.\n");

      bp::def("getFrameJacobianTimeVariation",get_frame_jacobian_time_variation_proxy,
              bp::args("model","data","frame_id","reference_frame"),
              "Returns the Jacobian time variation of the frame given by its frame_id either in the reference frame provided by reference_frame.\n"
              "You have to call computeJointJacobiansTimeVariation(model,data,q,v) and updateFramePlacements(model,data) first.");

    }
  
  } // namespace python

} // namespace pinocchio
