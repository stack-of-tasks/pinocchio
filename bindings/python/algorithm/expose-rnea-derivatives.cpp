//
// Copyright (c) 2018 CNRS INRIA
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
    
    typedef container::aligned_vector<Force> ForceAlignedVector;
    void computeRNEADerivatives(const Model & model, Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v,
                                      const Eigen::VectorXd & a)
    {
      se3::computeRNEADerivatives(model,data,q,v,a);
      // Symmetrize M
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void computeRNEADerivatives_fext(const Model & model, Data & data,
                                     const Eigen::VectorXd & q,
                                     const Eigen::VectorXd & v,
                                     const Eigen::VectorXd & a,
                                     const ForceAlignedVector & fext)
    {
      se3::computeRNEADerivatives(model,data,q,v,a,fext);
      // Symmetrize M
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void exposeRNEADerivatives()
    {
      using namespace Eigen;
      typedef container::aligned_vector<Force> ForceAlignedVector;
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)"),
              "Computes the RNEA derivatives, put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives_fext,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)",
                       "fext: vector external forces (size model.njoints)"),
              "Computes the RNEA derivatives with external contact foces,\n"
              "put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
    }
    
    
    
  } // namespace python
} // namespace se3
