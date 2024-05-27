//
// Copyright (c) 2016-2024 CNRS INRIA
//

#ifndef __pinocchio_parsers_mjcf_geometry_hxx__
#define __pinocchio_parsers_mjcf_geometry_hxx__

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/mjcf/mjcf-graph.hpp"

namespace pinocchio
{
  namespace mjcf
  {
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geomModel,
      ::hpp::fcl::MeshLoaderPtr meshLoader)
    {
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor(model);

      typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

      MjcfGraph graph(visitor, filename);

      graph.parseGraphFromXML(filename);

      // Use the Mjcf graph to create the geometry model
      graph.parseGeomTree(type, geomModel, meshLoader);

      return geomModel;
    }
  } // namespace mjcf
} // namespace pinocchio

#endif
