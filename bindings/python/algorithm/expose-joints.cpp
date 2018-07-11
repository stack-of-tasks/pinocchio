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
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace se3
{
  namespace python
  {

    static Eigen::VectorXd normalize_proxy(const Model & model,
                                           const Eigen::VectorXd & config)
    {
      Eigen::VectorXd q(config);
      normalize(model,q);
      return q;
    }

    void exposeJointsAlgo()
    {
      using namespace Eigen;
      
      bp::def("integrate",
              (VectorXd (*)(const Model &, const VectorXd &, const VectorXd &))&integrate,
              bp::args("Model",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Integrate the model for a tangent vector during one unit time .");
      
      bp::def("interpolate",
              (VectorXd (*)(const Model &, const VectorXd &, const VectorXd &, const double))&interpolate,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)",
                       "Double u"),
              "Interpolate the model between two configurations.");
      
      bp::def("difference",
              (VectorXd (*)(const Model &, const VectorXd &, const VectorXd &))&difference,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Difference between two configurations, ie. the tangent vector that must be integrated during one unit time"
              "to go from q1 to q2");
      
      bp::def("squaredDistance",
              (VectorXd (*)(const Model &, const VectorXd &, const VectorXd &))&squaredDistance,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Squared distance vector between two configurations.");
      
      bp::def("distance",
              (double (*)(const Model &, const VectorXd &, const VectorXd &))&distance,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Distance between two configurations.");
      
      bp::def("randomConfiguration",
              (VectorXd (*)(const Model &))&randomConfiguration,
              bp::arg("Model"),
              "Generate a random configuration in the bounds given by the lower and upper limits contained in model.");
      
      bp::def("randomConfiguration",
              (VectorXd (*)(const Model &, const VectorXd &, const VectorXd &))&randomConfiguration,
              bp::args("Model",
                       "Joint lower limits (size Model::nq)",
                       "Joint upper limits (size Model::nq)"),
              "Generate a random configuration in the bounds given by the Joint lower and upper limits arguments.");
      
      bp::def("normalize",normalize_proxy,
              bp::args("Model",
                       "Configuration q (size Model::nq)"),
              "return the configuration normalized ");
      
      bp::def("isSameConfiguration",
              (bool (*)(const Model &, const VectorXd &, const VectorXd &, const double&))&isSameConfiguration,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)",
                       "Precision"),
              "Return true if two configurations are equivalent");
    }
    
  } // namespace python
} // namespace se3
