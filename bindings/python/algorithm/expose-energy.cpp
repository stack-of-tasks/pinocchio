//
// Copyright (c) 2015-2016 CNRS
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
#include "pinocchio/algorithm/energy.hpp"

namespace se3
{
  namespace python
  {
    static double kineticEnergy_proxy(const ModelHandler & model,
                                      DataHandler & data,
                                      const VectorXd_fx & q,
                                      const VectorXd_fx & v,
                                      const bool update_kinematics = true)
    {
      return kineticEnergy(*model,*data,q,v,update_kinematics);
    }
    
    static double potentialEnergy_proxy(const ModelHandler & model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const bool update_kinematics = true)
    {
      return potentialEnergy(*model,*data,q,update_kinematics);
    }
    
    void exposeEnergy()
    {
      bp::def("kineticEnergy",kineticEnergy_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Update kinematics (bool)"),
              "Computes the kinematic energy of the model for the "
              "given joint configuration and velocity and stores the result "
              " in data.kinetic_energy. By default, the kinematics of model is updated.");
      
      bp::def("potentialEnergy",potentialEnergy_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Update kinematics (bool)"),
              "Computes the potential energy of the model for the "
              "given the joint configuration and stores the result "
              " in data.potential_energy. By default, the kinematics of model is updated.");
    }
    
  } // namespace python
} // namespace se3
