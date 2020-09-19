//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"

#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    bp::tuple getJointVelocityDerivatives_proxy(const Model & model,
                                                Data & data,
                                                const Model::JointIndex jointId,
                                                ReferenceFrame rf)
    {
      typedef Data::Matrix6x Matrix6x;
      
      Matrix6x partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6,model.nv));
      
      getJointVelocityDerivatives(model,data,jointId,rf,
                                  partial_dq,partial_dv);
      
      return bp::make_tuple(partial_dq,partial_dv);
    }
    
    bp::tuple getJointAccelerationDerivatives_proxy(const Model & model,
                                                    Data & data,
                                                    const Model::JointIndex jointId,
                                                    ReferenceFrame rf)
    {
      typedef Data::Matrix6x Matrix6x;
      
      Matrix6x v_partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6,model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6,model.nv));
      
      getJointAccelerationDerivatives(model,data,jointId,rf,
                                      v_partial_dq,a_partial_dq,
                                      a_partial_dv,a_partial_da);

      return bp::make_tuple(v_partial_dq,a_partial_dq,a_partial_dv,a_partial_da);
    }
  
    Data::Matrix3x getCoMVelocityDerivatives_proxy(const Model & model,
                                                   Data & data)
    {
      typedef Data::Matrix3x Matrix3x;
      Matrix3x partial_dq(Matrix3x::Zero(3,model.nv));
      getCenterOfMassVelocityDerivatives(model,data,partial_dq);
      return partial_dq;
    }
    
    
    void exposeKinematicsDerivatives()
    {
      using namespace Eigen;
      
      bp::def("computeForwardKinematicsDerivatives",
              &computeForwardKinematicsDerivatives<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
              bp::args("model","data","q","v","a"),
              "Computes all the terms required to compute the derivatives of the placement, spatial velocity and acceleration\n"
              "for any joint of the model.\n"
              "The results are stored in data.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\ta: the joint acceleration vector (size model.nv)\n");
      
      bp::def("getJointVelocityDerivatives",
              getJointVelocityDerivatives_proxy,
              bp::args("model","data","joint_id","reference_frame"),
              "Computes the partial derivatives of the spatial velocity of a given joint with respect to\n"
              "the joint configuration and velocity and returns them as a tuple.\n"
              "The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.\n"
              "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tjoint_id: index of the joint\n"
              "\treference_frame: reference frame in which the resulting derivatives are expressed\n");
      
      bp::def("getJointAccelerationDerivatives",
              getJointAccelerationDerivatives_proxy,
              bp::args("model","data","joint_id","reference_frame"),
              "Computes the partial derivatives of the spatial acceleration of a given joint with respect to\n"
              "the joint configuration, velocity and acceleration and returns them as a tuple.\n"
              "The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.\n"
              "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tjoint_id: index of the joint\n"
              "\treference_frame: reference frame in which the resulting derivatives are expressed\n");

      bp::def("getCenterOfMassVelocityDerivatives",
              getCoMVelocityDerivatives_proxy,
              bp::args("model","data"),
              "Computes the partial derivaties of the center of mass velocity with respect to\n"
              "the joint configuration.\n"
              "You must first call computeForwardKinematicsDerivatives and centerOfMass(q,v) "
              "before calling this function.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n");

    }
    
    
    
  } // namespace python
} // namespace pinocchio
