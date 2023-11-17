//
// Copyright (c) 2015-2023 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

#include "pinocchio/algorithm/reachable-workspace.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    bp::tuple reachableWorkspace_(const context::Model & model,
                                 const context::VectorXs & q,
                                 const double time_horizon, 
                                 const int frame_id,
                                 const int n_samples=5,
                                 const int facet_dims=3)
    {
      pinocchio::ReachableSetResults res;
      pinocchio::ReachableSetParams param;
      param.n_samples = n_samples;
      param.facet_dims = facet_dims;

      pinocchio::reachableWorkspace(model, q, time_horizon, frame_id, res, param);
      return bp::make_tuple(res.vertex, res.faces);
    }
    
    void exposeReachableWorkspace()
    {
        using namespace Eigen;

        bp::def("reachableWorkspace",
              &reachableWorkspace_,
              bp::args("model", "q","time_horizon","frame_id", "n_samples", "facet_dims"),
              "Computes the reachable workspace on a fixed time horizon. For more information, please see https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\ttime_horizon: time horizon for which the polytope will be computed\n"
              "\tframe_id: frame for which the polytope should be computed\n\n"
              "Returns:\n \t(vertex, faces)");
    }
  }
}