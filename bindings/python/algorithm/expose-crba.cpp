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
#include "pinocchio/algorithm/crba.hpp"

namespace se3
{
  namespace python
  {
    static Eigen::MatrixXd crba_proxy(const ModelHandler& model,
                                      DataHandler & data,
                                      const VectorXd_fx & q)
    {
      data->M.fill(0);
      crba(*model,*data,q);
      data->M.triangularView<Eigen::StrictlyLower>()
      = data->M.transpose().triangularView<Eigen::StrictlyLower>();
      return data->M;
    }
    
    static Data::Matrix6x ccrba_proxy(const ModelHandler& model,
                                      DataHandler & data,
                                      const VectorXd_fx & q,
                                      const VectorXd_fx & v)
    {
      ccrba(*model,*data,q,v);
      return data->Ag;
    }
    
    void exposeCRBA()
    {
      bp::def("crba",crba_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)"),
              "Computes CRBA, put the result in Data and return it.");
      
      bp::def("ccrba",ccrba_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite Rigid Body Inertia, puts the result in Data and returns the centroidal mapping.");
      
    }
    
  } // namespace python
} // namespace se3
