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
#include "pinocchio/algorithm/rnea.hpp"

namespace se3
{
  namespace python
  {
    static Eigen::VectorXd rnea_proxy(const ModelHandler& model,
                                      DataHandler & data,
                                      const VectorXd_fx & q,
                                      const VectorXd_fx & v,
                                      const VectorXd_fx & a)
    {
      return rnea(*model,*data,q,v,a);
    }
    
    static Eigen::VectorXd nle_proxy(const ModelHandler& model,
                                     DataHandler & data,
                                     const VectorXd_fx & q,
                                     const VectorXd_fx & v)
    {
      return nonLinearEffects(*model,*data,q,v);
    }
    
    void exposeRNEA()
    {
      bp::def("rnea",rnea_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)"),
              "Compute the RNEA, put the result in Data and return it.");
      
      bp::def("nle",nle_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), put the result in Data and return it.");
      
    }
    
  } // namespace python
} // namespace se3
