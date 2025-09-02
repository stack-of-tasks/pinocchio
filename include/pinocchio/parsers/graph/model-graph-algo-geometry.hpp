//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_graph_algo_geometry_hpp__
#define __pinocchio_parsers_graph_model_graph_algo_geometry_hpp__

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/multibody/geometry-object.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/meshloader-fwd.hpp"

namespace pinocchio
{
  namespace graph
  {

    /// @brief  Build a GeometryModel based on the graph that was built previously,
    /// that allows to have a root_joint.
    ///
    /// @param root_body First body to add to the model
    /// @param root_position position of said body wrt to the universe
    /// @param root_joint joint that will append to the root_body. by default, it will be fixed
    /// @param root_joint_name name of the first joint in the model
    ///
    /// @return A pinocchio model
    PINOCCHIO_PARSERS_DLLAPI GeometryModel buildGeometryModel(
      const ModelGraph & g,
      const Model & model,
      const GeometryType type,
      ::hpp::fcl::MeshLoaderPtr mesh_loader = ::hpp::fcl::MeshLoaderPtr());
  } // namespace graph
} // namespace pinocchio
#endif // ifndef __pinocchio_parsers_graph_model_graph_algo_geometry_hpp__
