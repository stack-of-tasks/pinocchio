//
// Copyright (c) 2016-2024 CNRS INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/parsers/mjcf.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/parsers/mjcf.hpp"
#endif // PINOCCHIO_LSP

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
      ::coal::MeshLoaderPtr meshLoader)
    {
      typedef ::pinocchio::parsers::Model Model;
      Model urdf_model = model;
      ::pinocchio::mjcf::details::MjcfVisitor visitor(urdf_model);

      typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

      MjcfGraph graph(visitor, filename);

      graph.parseGraphFromXML(filename);

      // Use the Mjcf graph to create the geometry model
      graph.parseGeomTree(type, geomModel, meshLoader);

      return geomModel;
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeomFromXMLContent(
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & xmlStream,
      const GeometryType type,
      GeometryModel & geomModel,
      ::coal::MeshLoaderPtr meshLoader)
    {
      typedef ::pinocchio::parsers::Model Model;
      Model mjcf_model = model;
      ::pinocchio::mjcf::details::MjcfVisitor visitor(mjcf_model);

      typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

      // No file path is available: relative mesh and texture paths are kept as is
      MjcfGraph graph(visitor, "");

      graph.parseGraphFromXMLContent(xmlStream);

      // Use the Mjcf graph to create the geometry model
      graph.parseGeomTree(type, geomModel, meshLoader);

      return geomModel;
    }
  } // namespace mjcf
} // namespace pinocchio
