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
    
    void exposeRNEA()
    {
      using namespace Eigen;
      typedef container::aligned_vector<Force> ForceAlignedVector;
      
      bp::def("rnea",
              (const VectorXd & (*)(const Model &, Data &, const VectorXd &, const VectorXd &, const VectorXd &))&rnea,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)"),
              "Compute the RNEA, put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("rnea",
              (const VectorXd & (*)(const Model &, Data &, const VectorXd &, const VectorXd &, const VectorXd &, const ForceAlignedVector &))&rnea,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)",
                       "Vector of external forces expressed in the local frame of each joint (size Model::njoints)"),
              "Compute the RNEA with external forces, put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());
      

      bp::def("nonLinearEffects",
              (const VectorXd & (*)(const Model &, Data &, const VectorXd &, const VectorXd &))&nonLinearEffects,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeGeneralizedGravity",
              (const VectorXd & (*)(const Model &, Data &, const VectorXd &))&computeGeneralizedGravity,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Compute the generalized gravity contribution, put the result in Data::g and return it",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeCoriolisMatrix",
              (const MatrixXd & (*)(const Model &, Data &, const VectorXd &, const VectorXd &))&computeCoriolisMatrix,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the Coriolis Matrix, put the result in Data::C and return it",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace se3
