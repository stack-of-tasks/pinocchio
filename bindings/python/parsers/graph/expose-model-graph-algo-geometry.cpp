//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/parsers/graph/model-graph-algo-geometry.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeAlgoGeometry()
    {
      using namespace pinocchio::graph;

      // Expose the global functions
      bp::def(
        "buildGeometryModel", &buildGeometryModel,
        (bp::arg("g"), bp::arg("model"), bp::arg("type"),
         bp::arg("mesh_loader") = ::hpp::fcl::MeshLoaderPtr()),
        "Build a pinocchio model based on the graph.");
    }
  } // namespace python
} // namespace pinocchio
