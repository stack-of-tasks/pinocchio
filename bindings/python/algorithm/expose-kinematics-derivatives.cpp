//
// Copyright (c) 2018 CNRS
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
#include "pinocchio/algorithm/kinematics-derivatives.hpp"

#include <boost/python/tuple.hpp>

namespace se3
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
      
      if(rf == WORLD)
        getJointVelocityDerivatives<WORLD>(model,data,jointId,
                                           partial_dq,partial_dv);
      else if(rf == LOCAL)
        getJointVelocityDerivatives<LOCAL>(model,data,jointId,
                                           partial_dq,partial_dv);
      else
        assert(false && "must never happened - wrong Reference Frame input");
      
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
      
      if(rf == WORLD)
        getJointAccelerationDerivatives<WORLD>(model,data,jointId,
                                               v_partial_dq,a_partial_dq,
                                               a_partial_dv,a_partial_da);
      else if(rf == LOCAL)
        getJointAccelerationDerivatives<LOCAL>(model,data,jointId,
                                               v_partial_dq,a_partial_dq,
                                               a_partial_dv,a_partial_da);
      else
        assert(false && "must never happened - wrong Reference Frame input");
      
      return bp::make_tuple(v_partial_dq,a_partial_dq,a_partial_dv,a_partial_da);
    }
    
    void exposeKinematicsDerivatives()
    {
      
      bp::def("computeForwardKinematicsDerivatives",
              computeForwardKinematicsDerivatives,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)"),
              "Computes all the terms required to compute the derivatives of the placement, spatial velocity and acceleration\n"
              "for any joint of the model.\n"
              "The results are stored in data.");
      
      bp::def("getJointVelocityDerivatives",
              getJointVelocityDerivatives_proxy,
              bp::args("Model","Data",
                       "jointId",
                       "Reference Frame (either WORLD or LOCAL)"),
              "Computes the partial derivaties of the spatial velocity of a given with respect to\n"
              "the joint configuration and velocity and returns them as a tuple.\n"
              "The Jacobians can be either expressed in the LOCAL frame of the joint or in the WORLD coordinate frame depending on the value of the Reference Frame.\n"
              "You must first call computForwardKinematicsDerivatives before calling this function");
      
      
      
      bp::def("getJointAccelerationDerivatives",
              getJointAccelerationDerivatives_proxy,
              bp::args("Model","Data",
                       "jointId",
                       "Reference Frame (either WORLD or LOCAL)"),
              "Computes the partial derivaties of the spatial acceleration of a given with respect to\n"
              "the joint configuration, velocity and acceleration and returns them as a tuple.\n"
              "The Jacobians can be either expressed in the LOCAL frame of the joint or in the WORLD coordinate frame depending on the value of the Reference Frame.\n"
              "You must first call computForwardKinematicsDerivatives before calling this function");
    }
    
    
    
  } // namespace python
} // namespace se3
