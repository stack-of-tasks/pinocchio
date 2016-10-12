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
#include "pinocchio/algorithm/aba.hpp"

namespace se3
{
  namespace python
  {
    
    static Eigen::MatrixXd aba_proxy(const Model & model,
                                     Data & data,
                                     const Eigen::VectorXd & q,
                                     const Eigen::VectorXd & v,
                                     const Eigen::VectorXd & tau)
    {
      aba(model,*data,q,v,tau);
      return data->ddq;
    }
    
    void exposeABA()
    {
      bp::def("aba",aba_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint torque tau (size Model::nv)"),
              "Compute ABA, put the result in Data::ddq and return it.");
      
    }
    
  } // namespace python
} // namespace se3
