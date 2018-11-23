//
// Copyright (c) 2015-2016,2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <boost/python/overloads.hpp>

namespace pinocchio
{
  namespace python
  {
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(jacobianCenterOfMassUpdate_overload,jacobianCenterOfMass,3,4)

    BOOST_PYTHON_FUNCTION_OVERLOADS(jacobianCenterOfMassNoUpdate_overload,jacobianCenterOfMass,2,3)
    
    static SE3::Vector3
    com_0_proxy(const Model& model,
                Data & data,
                const Eigen::VectorXd & q)
    {
      return centerOfMass(model,data,q,true);
    }
    
    static SE3::Vector3
    com_1_proxy(const Model& model,
                Data & data,
                const Eigen::VectorXd & q,
                const Eigen::VectorXd & v)
    {
      return centerOfMass(model,data,q,v,true);
    }
    
    static SE3::Vector3
    com_2_proxy(const Model & model,
                Data & data,
                const Eigen::VectorXd & q,
                const Eigen::VectorXd & v,
                const Eigen::VectorXd & a)
    {
      return centerOfMass(model,data,q,v,a,true);
    }
    
    void exposeCOM()
    {
      using namespace Eigen;
      
      bp::def("centerOfMass",com_0_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)"),
              "Compute the center of mass, putting the result in Data and return it.");
      
      bp::def("centerOfMass",com_1_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Computes the center of mass position and velocuty by storing the result in Data"
              "and returns the center of mass position of the full model expressed in the world frame.");
      
      bp::def("centerOfMass",com_2_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint acceleration a (size Model::nv)"),
              "Computes the center of mass position, velocity and acceleration by storing the result in Data"
              "and returns the center of mass position of the full model expressed in the world frame.");
      
      bp::def("jacobianCenterOfMass",
              (const Data::Matrix3x & (*)(const Model &, Data &, const Eigen::MatrixBase<Eigen::VectorXd> &, bool))&jacobianCenterOfMass<double,0,JointCollectionDefaultTpl,VectorXd>,
              jacobianCenterOfMassUpdate_overload(bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees"),
              "Computes the jacobian of the center of mass, puts the result in Data and return it.")[
              bp::return_value_policy<bp::return_by_value>()]);
      
            bp::def("jacobianCenterOfMass",
              (const Data::Matrix3x & (*)(const Model &, Data &, bool))&jacobianCenterOfMass<double,0,JointCollectionDefaultTpl>,
              jacobianCenterOfMassNoUpdate_overload(bp::args("Model","Data",
                       "computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees"),
              "Computes the jacobian of the center of mass, puts the result in Data and return it.")[
              bp::return_value_policy<bp::return_by_value>()]);
    }
    
  } // namespace python
} // namespace pinocchio
