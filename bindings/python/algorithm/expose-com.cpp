//
// Copyright (c) 2015-2016,2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <boost/python/overloads.hpp>

namespace se3
{
  namespace python
  {
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(jacobianCenterOfMass_overload,jacobianCenterOfMass,3,5)
    
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
      
      bp::def("jacobianCenterOfMass",&jacobianCenterOfMass<JointCollectionDefault,VectorXd>,
              jacobianCenterOfMass_overload(bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)"),
              "Computes the jacobian of the center of mass, puts the result in Data and return it.")[
              bp::return_value_policy<bp::return_by_value>()]);
      
    }
    
  } // namespace python
} // namespace se3
