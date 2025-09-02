//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph-algo-geometry.hpp"
#include "pinocchio/parsers/graph/geometries.hpp"

#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/mesh_loader/assimp.h>

namespace pinocchio
{
  namespace graph
  {
    namespace
    {
      struct AddGeometryToModel : public boost::static_visitor<>
      {
        const pinocchio::Frame & body_frame;
        const pinocchio::FrameIndex & f_id;
        const Geometry & geom;
        ::hpp::fcl::MeshLoaderPtr mesh_loader;
        GeometryModel & model;

        AddGeometryToModel(
          const pinocchio::Frame & b_f,
          const pinocchio::FrameIndex & f_id,
          const Geometry & g,
          ::hpp::fcl::MeshLoaderPtr mesh_loader,
          GeometryModel & model_)
        : body_frame(b_f)
        , f_id(f_id)
        , geom(g)
        , mesh_loader(mesh_loader)
        , model(model_)
        {
        }

        // For primitive only
        void addGeometry(std::shared_ptr<fcl::CollisionGeometry> g)
        {
          GeometryObject geometry_object(
            geom.name, body_frame.parentJoint, f_id, body_frame.placement * geom.placement, g, "",
            geom.scale, true, geom.color);

          model.addGeometryObject(geometry_object);
        }

        void operator()(const Sphere & s)
        {
          addGeometry(std::make_shared<fcl::Sphere>(s.radius));
        }

        void operator()(const Capsule & c)
        {
          addGeometry(std::make_shared<fcl::Capsule>(c.size[0], c.size[1]));
        }

        void operator()(const Cylinder & c)
        {
          addGeometry(std::make_shared<fcl::Cylinder>(c.size[0], c.size[1]));
        }

        void operator()(const Box & b)
        {
          addGeometry(std::make_shared<fcl::Box>(b.size[0], b.size[1], b.size[2]));
        }

        void operator()(const Mesh & m)
        {
          hpp::fcl::BVHModelPtr_t bvh = mesh_loader->load(m.path, geom.scale);

          GeometryObject geometry_object(
            geom.name, body_frame.parentJoint, f_id, body_frame.placement * geom.placement, bvh,
            m.path, geom.scale, true, geom.color);

          model.addGeometryObject(geometry_object);
        }
      };
    } // namespace

    GeometryModel buildGeometryModel(
      const ModelGraph & g,
      const Model & model,
      const GeometryType type,
      ::hpp::fcl::MeshLoaderPtr mesh_loader)
    {
      GeometryModel geomModel;
      if (!mesh_loader)
        mesh_loader = std::make_shared<fcl::MeshLoader>(fcl::MeshLoader());

      for (const auto & f : model.frames)
      {
        // Only add geometries to bodies
        if (f.type == BODY)
        {
          // check it's a model linked to the graph
          auto vertex = g.name_to_vertex.find(f.name);
          if (vertex == g.name_to_vertex.end())
          {
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Graph - Body in model is not in the graph. Please check arguments.");
          }

          // if it is go through vector of geometries and add them to the model
          const ModelGraphVertex & vert = g.graph[vertex->second];
          for (const Geometry & geom : vert.geometries)
          {
            AddGeometryToModel add_geom(
              f, model.getFrameId(f.name, BODY), geom, mesh_loader, geomModel);
            if (type == VISUAL && (geom.type == GeomType::VISUAL || geom.type == GeomType::BOTH))
            {
              boost::apply_visitor(add_geom, geom.geometry);
            }
            else if (
              type == COLLISION
              && (geom.type == GeomType::COLLISION || geom.type == GeomType::BOTH))
            {
              boost::apply_visitor(add_geom, geom.geometry);
            }
          }
        }
      }
      return geomModel;
    }
  } // namespace graph
} // namespace pinocchio
