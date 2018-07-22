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
#include "pinocchio/algorithm/regressor.hpp"

namespace se3
{
  namespace python
  {
    
    void exposeRegressor()
    {
      using namespace Eigen;

      bp::def("computeStaticRegressor",
              &regressor::computeStaticRegressor<JointCollectionDefault,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Compute the static regressor that links the inertia parameters of the system to its center of mass position\n"
              ",put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace se3
