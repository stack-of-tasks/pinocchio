//
// Copyright (c) 2015-2023 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

#include "pinocchio/extra/reachable-workspace.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

#ifndef PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE
    bp::tuple reachableWorkspaceHull_(
      const context::Model & model,
      const context::VectorXs & q0,
      const double time_horizon,
      const int frame_id,
      const int n_samples = 5,
      const int facet_dims = 3)
    {
      pinocchio::ReachableSetResults res;
      pinocchio::ReachableSetParams param;
      param.n_samples = n_samples;
      param.facet_dims = facet_dims;

      pinocchio::reachableWorkspaceHull(model, q0, time_horizon, frame_id, res, param);
      return bp::make_tuple(res.vertex, res.faces);
    }

    static context::Data::MatrixXs reachableWorkspace_(
      const context::Model & model,
      const context::VectorXs & q0,
      const double time_horizon,
      const int frame_id,
      const int n_samples = 5,
      const int facet_dims = 3)
    {
      pinocchio::ReachableSetParams param;
      param.n_samples = n_samples;
      param.facet_dims = facet_dims;
      context::Data::MatrixXs vertex;

      pinocchio::reachableWorkspace(model, q0, time_horizon, frame_id, vertex, param);

      return vertex;
    }

  #ifdef PINOCCHIO_WITH_HPP_FCL
    bp::tuple reachableWorkspaceWithCollisionsHull_(
      const context::Model & model,
      const GeometryModel & geom_model,
      const context::VectorXs & q0,
      const double time_horizon,
      const int frame_id,
      const int n_samples = 5,
      const int facet_dims = 3)
    {
      pinocchio::ReachableSetResults res;
      pinocchio::ReachableSetParams param;
      param.n_samples = n_samples;
      param.facet_dims = facet_dims;

      pinocchio::reachableWorkspaceWithCollisionsHull(
        model, geom_model, q0, time_horizon, frame_id, res, param);
      return bp::make_tuple(res.vertex, res.faces);
    }
    static context::Data::MatrixXs reachableWorkspaceWithCollisions_(
      const context::Model & model,
      const GeometryModel & geom_model,
      const context::VectorXs & q0,
      const double time_horizon,
      const int frame_id,
      const int n_samples = 5,
      const int facet_dims = 3)
    {

      pinocchio::ReachableSetParams param;
      param.n_samples = n_samples;
      param.facet_dims = facet_dims;

      context::Data::MatrixXs vertex;

      pinocchio::reachableWorkspaceWithCollisions(
        model, geom_model, q0, time_horizon, frame_id, vertex, param);
      return vertex;
    }
  #endif // PINOCCHIO_WITH_HPP_FCL
#endif   // PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE

    void exposeReachableWorkspace()
    {
#ifndef PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE
      using namespace Eigen;
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "reachableWorkspace", &reachableWorkspace_,
        bp::args("model", "q0", "time_horizon", "frame_id", "n_samples", "facet_dims"),
        "Computes the reachable workspace on a fixed time horizon. For more information, "
        "please see https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\ttime_horizon: time horizon for which the polytope will be computed\n"
        "\tframe_id: frame for which the polytope should be computed\n\n"
        "Returns:\n \tvertex");
      bp::def(
        "reachableWorkspaceHull", &reachableWorkspaceHull_,
        bp::args("model", "q0", "time_horizon", "frame_id", "n_samples", "facet_dims"),
        "Computes the convex hull of the reachable workspace on a fixed time horizon. For "
        "more information, please see "
        "https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\ttime_horizon: time horizon for which the polytope will be computed\n"
        "\tframe_id: frame for which the polytope should be computed\n\n"
        "Returns:\n \t(vertex, faces)");

  #ifdef PINOCCHIO_WITH_HPP_FCL
      bp::def(
        "reachableWorkspaceWithCollisions", &reachableWorkspaceWithCollisions_,
        bp::args(
          "model", "geom_model", "q0", "time_horizon", "frame_id", "n_samples", "facet_dims"),
        "Computes the reachable workspace taking geometry model into account on a fixed time "
        "horizon. For more information, please see "
        "https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tgeom_model: model of the environment to check for collisions\n"
        "\ttime_horizon: time horizon for which the polytope will be computed\n"
        "\tframe_id: frame for which the polytope should be computed\n\n"
        "Returns:\n \tvertex");

      bp::def(
        "reachableWorkspaceWithCollisionsHull", &reachableWorkspaceWithCollisionsHull_,
        bp::args(
          "model", "geom_model", "q0", "time_horizon", "frame_id", "n_samples", "facet_dims"),
        "Computes the convex hull of the reachable workspace taking geometry model into "
        "account on a fixed time horizon. For more information, please see "
        "https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tgeom_model: model of the environment to check for collisions\n"
        "\ttime_horizon: time horizon for which the polytope will be computed\n"
        "\tframe_id: frame for which the polytope should be computed\n\n"
        "Returns:\n \t(vertex, faces)");
  #endif // PINOCCHIO_WITH_HPP_FCL
#endif   // PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE
    }

  } // namespace python
} // namespace pinocchio
