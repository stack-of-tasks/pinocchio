//
// Copyright (c) 2015-2016,2018 CNRS
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
#include "pinocchio/algorithm/centroidal.hpp"

namespace se3
{
  namespace python
  {
    
    void exposeCentroidal()
    {
      
      bp::def("ccrba",ccrba,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite Rigid Body Inertia, puts the result in Data and returns the centroidal mapping.",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("dccrba",dccrba,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Computes the time derivative of the centroidal momentum matrix Ag in terms of q and v. It computes also the same information than ccrtba for the same price.",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace se3
