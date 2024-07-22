//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_multibody_parsers_sdf_geometry_hxx__
#define __pinocchio_multibody_parsers_sdf_geometry_hxx__

#include "pinocchio/parsers/config.hpp"
#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/parsers/utils.hpp"
#include "pinocchio/parsers/meshloader-fwd.hpp"

#include <string>
#include <vector>

namespace pinocchio
{
  namespace sdf
  {
    namespace details
    {
      /**
       * @brief      Recursive procedure for reading the URDF tree, looking for geometries
       *             This function fill the geometric model whith geometry objects retrieved from
       * the URDF tree
       *
       * @param[in]  tree           The URDF kinematic tree
       * @param[in]  meshLoader     The FCL mesh loader to avoid duplications of already loaded
       * geometries
       * @param[in]  link           The current URDF link
       * @param      model          The model to which is the GeometryModel associated
       * @param      geomModel      The GeometryModel where the Collision Objects must be added
       * @param[in]  package_dirs   A vector containing the different directories where to search
       * for packages
       * @param[in]  type           The type of objects that must be loaded ( can be VISUAL or
       * COLLISION)
       *
       */
      PINOCCHIO_PARSERS_DLLAPI void addLinkGeometryToGeomModel(
        const SdfGraph & graph,
        ::hpp::fcl::MeshLoaderPtr & meshLoader,
        const ::sdf::ElementPtr link,
        GeometryModel & geomModel,
        const std::vector<std::string> & package_dirs,
        const GeometryType type);

      PINOCCHIO_PARSERS_DLLAPI void parseTreeForGeom(
        const Model & model,
        const SdfGraph & graph,
        GeometryModel & geomModel,
        const std::string & rootLinkName,
        const GeometryType type,
        const std::vector<std::string> & package_dirs,
        ::hpp::fcl::MeshLoaderPtr meshLoader);
    } // namespace details

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & const_model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geomModel,
      const std::string & rootLinkName,
      const std::vector<std::string> & package_dirs,
      ::hpp::fcl::MeshLoaderPtr meshLoader)
    {
      Model & model =
        const_cast<Model &>(const_model); // TODO: buildGeom should not need to parse model again.
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor(model);
      ::pinocchio::sdf::details::SdfGraph graph(visitor);

      // Create maps from the SDF Graph
      graph.parseGraphFromFile(filename);

      if (rootLinkName == "")
      {
        const_cast<std::string &>(rootLinkName) = details::findRootLink(graph);
      }

      details::parseTreeForGeom(
        model, graph, geomModel, rootLinkName, type, package_dirs, meshLoader);
      return geomModel;
    }

  } // namespace sdf
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_parsers_sdf_geometry_hxx__
