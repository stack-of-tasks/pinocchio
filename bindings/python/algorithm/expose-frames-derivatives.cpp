//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"

#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    bp::tuple getFrameVelocityDerivatives_proxy(const Model & model,
                                                Data & data,
                                                const Model::FrameIndex frame_id,
                                                ReferenceFrame rf)
    {
      typedef Data::Matrix6x Matrix6x;
      
      Matrix6x partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6,model.nv));
      
      getFrameVelocityDerivatives(model,data,frame_id,rf,
                                  partial_dq,partial_dv);
      
      return bp::make_tuple(partial_dq,partial_dv);
    }
    
    bp::tuple getFrameAccelerationDerivatives_proxy(const Model & model,
                                                    Data & data,
                                                    const Model::FrameIndex frame_id,
                                                    ReferenceFrame rf)
    {
      typedef Data::Matrix6x Matrix6x;
      
      Matrix6x v_partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6,model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6,model.nv));
      
      getFrameAccelerationDerivatives(model,data,frame_id,rf,
                                      v_partial_dq,a_partial_dq,
                                      a_partial_dv,a_partial_da);

      return bp::make_tuple(v_partial_dq,a_partial_dq,a_partial_dv,a_partial_da);
    }
    
    void exposeFramesDerivatives()
    {
      using namespace Eigen;
      
      bp::def("getFrameVelocityDerivatives",
              getFrameVelocityDerivatives_proxy,
              bp::args("model","data","frame_id","reference_frame"),
              "Computes the partial derivatives of the spatial velocity of a given frame with respect to\n"
              "the joint configuration and velocity and returns them as a tuple.\n"
              "The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.\n"
              "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tframe_id: index of the frame\n"
              "\treference_frame: reference frame in which the resulting derivatives are expressed\n");
      
      bp::def("getFrameAccelerationDerivatives",
              getFrameAccelerationDerivatives_proxy,
              bp::args("model","data","frame_id","reference_frame"),
              "Computes the partial derivatives of the spatial acceleration of a given frame with respect to\n"
              "the joint configuration, velocity and acceleration and returns them as a tuple.\n"
              "The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.\n"
              "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tframe_id: index of the frame\n"
              "\treference_frame: reference frame in which the resulting derivatives are expressed\n");

    }
    
  } // namespace python
} // namespace pinocchio
