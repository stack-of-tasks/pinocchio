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
#include "pinocchio/algorithm/aba-derivatives.hpp"

namespace se3
{
  namespace python
  {
    void computeABADerivativesDefault(const Model & model, Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v,
                                      const Eigen::VectorXd & tau)
    {
      computeABADerivatives(model,data,q,v,tau);
    }
    
    void exposeABADerivatives()
    {
      using namespace Eigen;
      typedef container::aligned_vector<Force> ForceAlignedVector;
      
      bp::def("computeABADerivatives",
              computeABADerivativesDefault,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Torque tau (size Model::nv)"),
              "Computes the ABA derivatives, put the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
              "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
              "velocity and torque vectors.");
    }
    
    
    
  } // namespace python
} // namespace se3
