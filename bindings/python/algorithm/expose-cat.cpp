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
#include "pinocchio/algorithm/compute-all-terms.hpp"

namespace se3
{
  namespace python
  {
    static void computeAllTerms_proxy(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v)
    {
      data.M.fill(0);
      computeAllTerms(model,data,q,v);
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void exposeCAT()
    {
      bp::def("computeAllTerms",computeAllTerms_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute all the terms M, non linear effects and Jacobians in"
              "in the same loop and put the results in data.");
    }
  } // namespace python
} // namespace se3
