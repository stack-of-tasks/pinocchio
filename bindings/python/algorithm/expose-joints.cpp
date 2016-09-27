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
    static Eigen::VectorXd integrate_proxy(const ModelHandler & model,
                                           const Eigen::VectorXd & q,
                                           const Eigen::VectorXd & v)
    {
      return integrate(*model,q,v);
    }
    
    static Eigen::VectorXd interpolate_proxy(const ModelHandler & model,
                                             const Eigen::VectorXd & q1,
                                             const Eigen::VectorXd & q2,
                                             const double u)
    {
      return interpolate(*model,q1,q2,u);
    }
    
    static Eigen::VectorXd differentiate_proxy(const ModelHandler & model,
                                               const Eigen::VectorXd & q1,
                                               const Eigen::VectorXd & q2)
    {
      return differentiate(*model,q1,q2);
    }
    
    static Eigen::VectorXd distance_proxy(const ModelHandler & model,
                                          const Eigen::VectorXd & q1,
                                          const Eigen::VectorXd & q2)
    {
      return distance(*model,q1,q2);
    }
    
    static Eigen::VectorXd randomConfiguration_proxy(const ModelHandler & model,
                                                     const Eigen::VectorXd & lowerPosLimit,
                                                     const Eigen::VectorXd & upperPosLimit)
    {
      return randomConfiguration(*model, lowerPosLimit, upperPosLimit);
    }

    static void normalize_proxy(const ModelHandler & model,
                                Eigen::VectorXd & config)
    {
      Eigen::VectorXd q(config);
      normalize(*model, q);
      config = q;
    }

    static bool isSameConfiguration_proxy(const ModelHandler & model,
                                          const Eigen::VectorXd & q1,
                                          const Eigen::VectorXd & q2)
    {
      return isSameConfiguration(*model, q1, q2);
    }
    
    void exposeJointsAlgo()
    {
      bp::def("integrate",integrate_proxy,
              bp::args("Model",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Integrate the model for a tangent vector during one unit time .");
      
      bp::def("interpolate",interpolate_proxy,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)",
                       "Double u"),
              "Interpolate the model between two configurations.");
      bp::def("differentiate",differentiate_proxy,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Difference between two configurations, ie. the tangent vector that must be integrated during one unit time"
              "to go from q1 to q2");
      bp::def("distance",distance_proxy,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Distance between two configurations ");
      bp::def("randomConfiguration",randomConfiguration_proxy,
              bp::args("Model",
                       "Joint lower limits (size Model::nq)",
                       "Joint upper limits (size Model::nq)"),
              "Generate a random configuration ensuring provied joint limits are respected ");
      bp::def("normalize",normalize_proxy,
              bp::args("Model",
                       "Configuration q (size Model::nq)"),
              "return the configuration normalized ");
      bp::def("isSameConfiguration",isSameConfiguration_proxy,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Return true if two configurations are equivalent");
    }
    
  } // namespace python
} // namespace se3
