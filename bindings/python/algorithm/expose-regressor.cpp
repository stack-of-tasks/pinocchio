//
// Copyright (c) 2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio
{
  namespace python
  {

    Eigen::MatrixXd bodyRegressor_proxy(const Motion & v, const Motion & a)
    {
      return bodyRegressor(v,a);
    }

    Eigen::MatrixXd jointBodyRegressor_proxy(const Model & model, Data & data, const JointIndex jointId)
    {
      return jointBodyRegressor(model,data,jointId);
    }

    Eigen::MatrixXd frameBodyRegressor_proxy(const Model & model, Data & data, const FrameIndex frameId)
    {
      return frameBodyRegressor(model,data,frameId);
    }

    void exposeRegressor()
    {
      using namespace Eigen;

      bp::def("computeStaticRegressor",
              &computeStaticRegressor<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Compute the static regressor that links the inertia parameters of the system to its center of mass position,\n"
              "put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("bodyRegressor",
              &bodyRegressor_proxy,
              bp::args("velocity","acceleration"),
              "Computes the regressor for the dynamic parameters of a single rigid body.\n"
              "The result is such that "
              "Ia + v x Iv = bodyRegressor(v,a) * I.toDynamicParameters()");

      bp::def("jointBodyRegressor",
              &jointBodyRegressor_proxy,
              bp::args("Model","Data",
                       "jointId (int)"),
              "Compute the regressor for the dynamic parameters of a rigid body attached to a given joint.\n"
              "This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.");

      bp::def("frameBodyRegressor",
              &frameBodyRegressor_proxy,
              bp::args("Model","Data",
                       "frameId (int)"),
              "Computes the regressor for the dynamic parameters of a rigid body attached to a given frame.\n"
              "This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.");

      bp::def("computeJointTorqueRegressor",
              &computeJointTorqueRegressor<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"
                       "Acceleration a (size Model::nv)"),
              "Compute the joint torque regressor that links the joint torque "
              "to the dynamic parameters of each link according to the current the robot motion,\n"
              "put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());
    }
    
  } // namespace python
} // namespace pinocchio
