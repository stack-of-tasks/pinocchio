//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

namespace pinocchio
{
  namespace python
  {
    
    static Data::Matrix6x
    compute_jacobian_proxy(const Model & model,
                           Data & data,
                           const Eigen::VectorXd & q,
                           Model::JointIndex jointId)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      computeJointJacobian(model,data,q,jointId,J);
      
      return J;
    }
    
    static Data::Matrix6x
    get_jacobian_proxy(const Model & model,
                       Data & data,
                       Model::JointIndex jointId,
                       ReferenceFrame rf)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      getJointJacobian(model,data,jointId,rf,J);
      
      return J;
    }
    
    static Data::Matrix6x
    get_jacobian_time_variation_proxy(const Model & model,
                                      Data & data,
                                      Model::JointIndex jointId,
                                      ReferenceFrame rf)
    {
      Data::Matrix6x dJ(6,model.nv); dJ.setZero();
      getJointJacobianTimeVariation(model,data,jointId,rf,dJ);
      
      return dJ;
    }
  
    void exposeJacobian()
    {
      using namespace Eigen;
      
      bp::def("computeJointJacobians",
              &computeJointJacobians<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","data","q"),
              "Computes the full model Jacobian, i.e. the stack of all the motion subspaces expressed in the coordinate world frame.\n"
              "The result is accessible through data.J. This function computes also the forward kinematics of the model.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeJointJacobians",
              &computeJointJacobians<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data"),
              "Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.\n"
              "The result is accessible through data.J. This function assumes that forward kinematics (pinocchio.forwardKinematics) has been called first.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("computeJointJacobian",compute_jacobian_proxy,
              bp::args("model","data","q","joint_id"),
              "Computes the Jacobian of a specific joint frame expressed in the local frame of the joint according to the given input configuration.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tjoint_id: index of the joint\n");

      bp::def("getJointJacobian",get_jacobian_proxy,
              bp::args("model","data","joint_id","reference_frame"),
              "Computes the jacobian of a given given joint according to the given entries in data.\n"
              "If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local coordinate system of the joint.\n"
              "If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in the coordinate system of the frame centered on the joint, but aligned with the WORLD axes.\n"
              "If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate system of the frame associated to the WORLD.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tjoint_id: index of the joint\n"
              "\treference_frame: reference frame in which the resulting derivatives are expressed\n");
      
      bp::def("computeJointJacobiansTimeVariation",computeJointJacobiansTimeVariation<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","data","q","v"),
              "Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v. It also computes the joint Jacobian of the model (similar to computeJointJacobians)."
              "The result is accessible through data.dJ and data.J.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("getJointJacobianTimeVariation",get_jacobian_time_variation_proxy,
              bp::args("model","data","joint_id","reference_frame"),
              "Computes the Jacobian time variation of a specific joint expressed in the requested frame provided by the value of reference_frame."
              "You have to call computeJointJacobiansTimeVariation first. This function also computes the full model Jacobian contained in data.J.\n"
              "If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local coordinate system of the joint.\n"
              "If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in the coordinate system of the frame centered on the joint, but aligned with the WORLD axes.\n"
              "If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate system of the frame associated to the WORLD.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tjoint_id: index of the joint\n"
              "\treference_frame: reference frame in which the resulting derivatives are expressed\n");
    }
    
  } // namespace python
} // namespace pinocchio
