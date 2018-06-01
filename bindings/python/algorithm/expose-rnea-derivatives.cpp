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
#include "pinocchio/algorithm/rnea-derivatives.hpp"

namespace se3
{
  namespace python
  {
    void computeRNEADerivativesDefault(const Model & model, Data & data,
                                       const Eigen::VectorXd & q,
                                       const Eigen::VectorXd & v,
                                       const Eigen::VectorXd & a)
    {
      computeRNEADerivatives(model,data,q,v,a);
      // Symmetrize M
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void exposeRNEADerivatives()
    {
      using namespace Eigen;
      typedef container::aligned_vector<Force> ForceAlignedVector;
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivativesDefault,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)"),
              "Computes the RNEA derivatives, put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
    }
    
    
    
  } // namespace python
} // namespace se3
