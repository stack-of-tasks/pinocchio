//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/parsers/utils.hpp"

#include <sstream>
#include <iomanip>
#include <boost/shared_ptr.hpp>

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include <hpp/fcl/mesh_loader/loader.h>
  #include <hpp/fcl/mesh_loader/assimp.h>
#endif // PINOCCHIO_WITH_HPP_FCL

namespace pinocchio
{
  namespace sdf
  {
    namespace details
    {
      /**
       * @brief Get the first geometry attached to a link
       *
       * @param[in] link   The URDF link
       *
       * @return Either the first collision or visual
       */
      template<GeometryType type>
      static bool hasLinkElement(const ::sdf::ElementPtr link);

      template<>
      bool hasLinkElement<::pinocchio::COLLISION>(const ::sdf::ElementPtr link)
      {
        return link->HasElement("collision");
      }

      template<>
      bool hasLinkElement<::pinocchio::VISUAL>(const ::sdf::ElementPtr link)
      {
        return link->HasElement("visual");
      }

      /**
       * @brief Get the array of geometries attached to a link
       *
       * @param[in] link   The URDF link
       *
       * @return the array of either collisions or visuals
       */
      template<GeometryType type>
      static const std::vector<::sdf::ElementPtr>
      getLinkGeometryArray(const ::sdf::ElementPtr link);

      template<>
      const std::vector<::sdf::ElementPtr>
      getLinkGeometryArray<::pinocchio::COLLISION>(const ::sdf::ElementPtr link)
      {
        std::vector<::sdf::ElementPtr> geometry_array;
        ::sdf::ElementPtr geomElement = link->GetElement("collision");
        while (geomElement)
        {
          // Inserting data in std::map
          if (geomElement->Get<std::string>("name") != "__default__")
          {
            geometry_array.push_back(geomElement);
          }
          geomElement = geomElement->GetNextElement("collision");
        }
        return geometry_array;
      }

      template<>
      const std::vector<::sdf::ElementPtr>
      getLinkGeometryArray<::pinocchio::VISUAL>(const ::sdf::ElementPtr link)
      {
        std::vector<::sdf::ElementPtr> geometry_array;
        ::sdf::ElementPtr geomElement = link->GetElement("visual");
        while (geomElement)
        {
          // Inserting data in std::map
          if (geomElement->Get<std::string>("name") != "__default__")
          {
            geometry_array.push_back(geomElement);
          }
          geomElement = geomElement->GetNextElement("visual");
        }
        return geometry_array;
      }

      static Eigen::Vector3d retrieveMeshScale(const ::sdf::ElementPtr sdf_mesh)
      {
        const ignition::math::Vector3d ign_scale = sdf_mesh->Get<ignition::math::Vector3d>("scale");
        return Eigen::Vector3d(ign_scale.X(), ign_scale.Y(), ign_scale.Z());
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      /**
       * @brief      Get a fcl::CollisionObject from an urdf geometry, searching
       *             for it in specified package directories
       *
       * @param[in]  urdf_geometry  A shared pointer on the input urdf Geometry
       * @param[in]  package_dirs   A vector containing the different directories where to search
       * for packages
       * @param[out] meshPath      The Absolute path of the mesh currently read
       * @param[out] meshScale     Scale of transformation currently applied to the mesh
       *
       * @return     A shared pointer on the geometry converted as a fcl::CollisionGeometry
       */
      std::shared_ptr<fcl::CollisionGeometry> static retrieveCollisionGeometry(
        ::hpp::fcl::MeshLoaderPtr & meshLoader,
        const ::sdf::ElementPtr sdf_geometry,
        const std::vector<std::string> & package_dirs,
        std::string & meshPath,
        Eigen::Vector3d & meshScale)
      {
        std::shared_ptr<fcl::CollisionGeometry> geometry;

        // Handle the case where collision geometry is a mesh
        if (sdf_geometry->HasElement("mesh"))
        {
          const ::sdf::ElementPtr sdf_mesh = sdf_geometry->GetElement("mesh");
          std::string collisionFilename = sdf_mesh->Get<std::string>("uri");

          meshPath = retrieveResourcePath(collisionFilename, package_dirs);
          if (meshPath == "")
          {
            std::stringstream ss;
            ss << "Mesh " << collisionFilename << " could not be found.";
            throw std::invalid_argument(ss.str());
          }

          Eigen::Vector3d scale(retrieveMeshScale(sdf_mesh));

          // Create FCL mesh by parsing Collada file.
          hpp::fcl::BVHModelPtr_t bvh = meshLoader->load(meshPath, scale);
          geometry = bvh;
        }

        // Handle the case where collision geometry is a cylinder
        // Use FCL capsules for cylinders
        else if (sdf_geometry->HasElement("cylinder"))
        {
          meshScale << 1, 1, 1;
          const ::sdf::ElementPtr collisionGeometry = sdf_geometry->GetElement("cylinder");

          double radius = collisionGeometry->Get<double>("radius");
          double length = collisionGeometry->Get<double>("length");

          // Create fcl capsule geometry.
          meshPath = "CYLINDER";
          geometry = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Cylinder(radius, length));
        }
        // Handle the case where collision geometry is a box.
        else if (sdf_geometry->HasElement("box"))
        {
          meshPath = "BOX";
          meshScale << 1, 1, 1;

          const ::sdf::ElementPtr collisionGeometry = sdf_geometry->GetElement("box");
          double x = collisionGeometry->Get<double>("x");
          double y = collisionGeometry->Get<double>("y");
          double z = collisionGeometry->Get<double>("z");
          geometry = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(x, y, z));
        }
        // Handle the case where collision geometry is a sphere.
        else if (sdf_geometry->HasElement("sphere"))
        {
          meshPath = "SPHERE";
          meshScale << 1, 1, 1;
          const ::sdf::ElementPtr collisionGeometry = sdf_geometry->GetElement("sphere");

          double radius = collisionGeometry->Get<double>("radius");
          geometry = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(radius));
        }
        else
          throw std::invalid_argument("Unknown geometry type :");

        if (!geometry)
        {
          throw std::invalid_argument("The polyhedron retrived is empty");
        }
        return geometry;
      }
#endif

      template<GeometryType type>
      static void addLinkGeometryToGeomModel(
        const SdfGraph & graph,
        ::hpp::fcl::MeshLoaderPtr & meshLoader,
        ::sdf::ElementPtr link,
        GeometryModel & geomModel,
        const std::vector<std::string> & package_dirs)
      {

        typedef std::vector<::sdf::ElementPtr> GeometryArray;
        typedef GeometryModel::SE3 SE3;

        if (hasLinkElement<type>(link))
        {
          std::string meshPath = "";

          Eigen::Vector3d meshScale(Eigen::Vector3d::Ones());
          const std::string link_name = link->Get<std::string>("name");

          const GeometryArray geometries_array = getLinkGeometryArray<type>(link);

          FrameIndex frame_id = graph.urdfVisitor.getBodyId(link_name);
          Frame frame = graph.urdfVisitor.getBodyFrame(link_name);

          SE3 body_placement = frame.placement;

          std::size_t objectId = 0;
          for (typename GeometryArray::const_iterator i = geometries_array.begin();
               i != geometries_array.end(); ++i)
          {
            meshPath.clear();
            const ::sdf::ElementPtr sdf_geometry = (*i)->GetElement("geometry");
#ifdef PINOCCHIO_WITH_HPP_FCL
            const GeometryObject::CollisionGeometryPtr geometry = retrieveCollisionGeometry(
              meshLoader, sdf_geometry, package_dirs, meshPath, meshScale);
#else
            const ::sdf::ElementPtr sdf_mesh = sdf_geometry->GetElement("mesh");

            if (sdf_mesh)
            {
              std::string collisionFilename = sdf_mesh->Get<std::string>("url");
              meshPath = retrieveResourcePath(collisionFilename, package_dirs);

              meshScale = retrieveMeshScale(sdf_mesh);
            }

            const auto geometry = std::make_shared<fcl::CollisionGeometry>();
#endif // PINOCCHIO_WITH_HPP_FCL

            const ignition::math::Pose3d & pose =
              (*i)->template Get<ignition::math::Pose3d>("pose");

            Eigen::Vector4d meshColor(Eigen::Vector4d::Zero());
            std::string meshTexturePath;
            bool overrideMaterial = false;
            const ::sdf::ElementPtr sdf_material = (*i)->GetElement("material");
            if (sdf_material)
            {
              const ignition::math::Color ign_meshColor =
                sdf_material->Get<ignition::math::Color>("ambient");

              meshColor << ign_meshColor.R(), ign_meshColor.G(), ign_meshColor.B(),
                ign_meshColor.A();
              overrideMaterial = true;
            }

            const SE3 lMg = convertFromPose3d(pose);
            const SE3 geomPlacement = body_placement * lMg;
            std::ostringstream geometry_object_suffix;
            geometry_object_suffix << "_" << objectId;
            const std::string & geometry_object_name =
              std::string(link_name + geometry_object_suffix.str());
            GeometryObject geometry_object(
              geometry_object_name, frame.parentJoint, frame_id, geomPlacement, geometry, meshPath,
              meshScale, overrideMaterial, meshColor, meshTexturePath);
            geomModel.addGeometryObject(geometry_object);
            ++objectId;
          }
        }
      }

      void addLinkGeometryToGeomModel(
        const SdfGraph & graph,
        ::hpp::fcl::MeshLoaderPtr & meshLoader,
        const ::sdf::ElementPtr link,
        GeometryModel & geomModel,
        const std::vector<std::string> & package_dirs,
        const GeometryType type)
      {
        switch (type)
        {
        case COLLISION:
          addLinkGeometryToGeomModel<::pinocchio::COLLISION>(
            graph, meshLoader, link, geomModel, package_dirs);
          break;
        case VISUAL:
          addLinkGeometryToGeomModel<::pinocchio::VISUAL>(
            graph, meshLoader, link, geomModel, package_dirs);
          break;
        default:
          break;
        }
      }

      void parseTreeForGeom(
        const Model & model,
        const SdfGraph & graph,
        GeometryModel & geomModel,
        const std::string & rootLinkName,
        const GeometryType type,
        const std::vector<std::string> & package_dirs,
        ::hpp::fcl::MeshLoaderPtr meshLoader)
      {
        std::vector<std::string> hint_directories(package_dirs);
        std::vector<std::string> ros_pkg_paths = rosPaths();
        hint_directories.insert(hint_directories.end(), ros_pkg_paths.begin(), ros_pkg_paths.end());

#ifdef PINOCCHIO_WITH_HPP_FCL
        if (!meshLoader)
          meshLoader = fcl::MeshLoaderPtr(new fcl::MeshLoader);
#endif // ifdef PINOCCHIO_WITH_HPP_FCL

        const ::sdf::ElementPtr rootElement = graph.mapOfLinks.find(rootLinkName)->second;

        addLinkGeometryToGeomModel(
          graph, meshLoader, rootElement, geomModel, hint_directories, type);

        for (pinocchio::Model::FrameVector::const_iterator fm = std::begin(model.frames);
             fm != std::end(model.frames); ++fm)
        {
          if (
            (fm->type != FIXED_JOINT && fm->type != JOINT)
            || (graph.mapOfJoints.find(fm->name) == graph.mapOfJoints.end()))
          {
            continue;
          }
          const ::sdf::ElementPtr childJointElement = graph.mapOfJoints.find(fm->name)->second;
          const std::string childLinkName =
            childJointElement->GetElement("child")->template Get<std::string>();
          const ::sdf::ElementPtr childLinkElement = graph.mapOfLinks.find(childLinkName)->second;

          addLinkGeometryToGeomModel(
            graph, meshLoader, childLinkElement, geomModel, hint_directories, type);
        }
      }
    } // namespace details
  }   // namespace sdf
} // namespace pinocchio
