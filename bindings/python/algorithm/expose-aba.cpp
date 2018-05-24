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
    
    const Data::RowMatrixXd &
    computeMinverse_proxy(const Model & model, Data & data, const Eigen::VectorXd & q)
    {
      computeMinverse(model,data,q);
      data.Minv.triangularView<Eigen::StrictlyLower>() =
      data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
      return data.Minv;
    }
    
    void exposeABA()
    {
      typedef Eigen::VectorXd DataVector;
      typedef container::aligned_vector<Force> ForceAlignedVector;

      bp::def("aba",
              (const DataVector& (*)(const Model&, Data&, const DataVector&, const DataVector&, const DataVector&)) &aba,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint torque tau (size Model::nv)"),
              "Compute ABA, put the result in Data::ddq and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("aba",
              (const DataVector& (*)(const Model&, Data&, const DataVector&, const DataVector&, const DataVector&, const ForceAlignedVector&))&aba,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint torque tau (size Model::nv)",
                       "Vector of external forces expressed in the local frame of each joint (size Model::njoints)"),
              "Compute ABA with external forces, put the result in Data::ddq and return it.",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("computeMinverse",
              &computeMinverse_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)"),
              "Computes the inverse of the joint space inertia matrix using a variant of the Articulated Body algorithm.\n"
              "The result is stored in data.Minv.",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace se3
