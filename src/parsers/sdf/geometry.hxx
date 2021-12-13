//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_multibody_parsers_sdf_geometry_hxx__
#define __pinocchio_multibody_parsers_sdf_geometry_hxx__

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
      template <GeometryType type>
      inline bool hasLinkElement(const ::sdf::ElementPtr link);

      template <>
      inline bool hasLinkElement<::pinocchio::COLLISION>(const ::sdf::ElementPtr link)
      {
        return link->HasElement("collision");
      }

      template <>
      inline bool hasLinkElement<::pinocchio::VISUAL>(const ::sdf::ElementPtr link)
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
      template <GeometryType type>
      inline const std::vector< ::sdf::ElementPtr>
      getLinkGeometryArray(const ::sdf::ElementPtr link);

      template <>
      inline const std::vector< ::sdf::ElementPtr>
      getLinkGeometryArray<::pinocchio::COLLISION>(const ::sdf::ElementPtr link)
      {
        std::vector< ::sdf::ElementPtr> geometry_array;
        ::sdf::ElementPtr geomElement = link->GetElement("collision");
        while (geomElement)
        { 
	  //Inserting data in std::map
	  if (geomElement->Get<std::string>("name") != "__default__") {
	    geometry_array.push_back(geomElement);
	  }
	    geomElement = link->GetNextElement("collision");
        }
        return geometry_array;
      }

      template <>
      inline const std::vector< ::sdf::ElementPtr>
      getLinkGeometryArray<::pinocchio::VISUAL>(const ::sdf::ElementPtr link)
      {
        std::vector< ::sdf::ElementPtr> geometry_array;
        ::sdf::ElementPtr geomElement = link->GetElement("visual");
        while (geomElement)
        {
          //Inserting data in std::map
	  if (geomElement->Get<std::string>("name") != "__default__") {
	    geometry_array.push_back(geomElement);
	  }
          geomElement = link->GetNextElement("visual");
        }
        return geometry_array;
      }

      
      template<typename Vector3>
      static void retrieveMeshScale(const fcl::Vec3f & meshScale,
                                    const Eigen::MatrixBase<Vector3> & scale)
      {
        Vector3 & scale_ = PINOCCHIO_EIGEN_CONST_CAST(Vector3,scale);
        scale_ <<
          meshScale[0],
          meshScale[1],
          meshScale[2];
      }


      /**
       * @brief      Get a fcl::CollisionObject from an urdf geometry, searching
       *             for it in specified package directories
       *
       * @param[in]  urdf_geometry  A shared pointer on the input urdf Geometry
       * @param[in]  package_dirs   A vector containing the different directories where to search for packages
       * @param[out] meshPath      The Absolute path of the mesh currently read
       * @param[out] meshScale     Scale of transformation currently applied to the mesh
       *
       * @return     A shared pointer on the geometry converted as a fcl::CollisionGeometry
       */
      boost::shared_ptr<fcl::CollisionGeometry>
      inline retrieveCollisionGeometry(const SdfGraph& graph,
                                       fcl::MeshLoaderPtr& meshLoader,
                                       const std::string& linkName,
                                       const std::string& geomName,
                                       const ::sdf::ElementPtr sdf_geometry,
                                       const std::vector<std::string> & package_dirs,
                                       std::string & meshPath,
                                       Eigen::Vector3d & meshScale)
      {
        boost::shared_ptr<fcl::CollisionGeometry> geometry;

        // Handle the case where collision geometry is a mesh
        if (sdf_geometry->HasElement("mesh"))
        { 
          const ::sdf::ElementPtr sdf_mesh = sdf_geometry->GetElement("mesh");
          std::string collisionFilename = sdf_mesh->Get<std::string>("uri");
          
          meshPath = retrieveResourcePath(collisionFilename, package_dirs);
          if (meshPath == "") {
            std::stringstream ss;
            ss << "Mesh " << collisionFilename << " could not be found.";
            throw std::invalid_argument (ss.str());
          }
          
          const ignition::math::Vector3d ign_scale =
            sdf_mesh->Get<ignition::math::Vector3d>("scale");
          
          fcl::Vec3f scale = fcl::Vec3f(ign_scale.X(),
                                        ign_scale.Y(),
                                        ign_scale.Z()
                                        );
          
          retrieveMeshScale(scale, meshScale);
          
          // Create FCL mesh by parsing Collada file.
          hpp::fcl::BVHModelPtr_t bvh = meshLoader->load (meshPath, scale);
          geometry = bvh;
        }

        // Handle the case where collision geometry is a cylinder
        // Use FCL capsules for cylinders
        else if (sdf_geometry->HasElement("cylinder"))
        {
          meshScale << 1,1,1;
          const ::sdf::ElementPtr collisionGeometry =
            sdf_geometry->GetElement("cylinder");
          
          double radius = collisionGeometry->Get<double>("radius");
          double length = collisionGeometry->Get<double>("length");

          // Create fcl capsule geometry.
          meshPath = "CYLINDER";
          geometry = boost::shared_ptr < fcl::CollisionGeometry >(
                                    new fcl::Cylinder (radius, length));
        }
        // Handle the case where collision geometry is a box.
        else if (sdf_geometry->HasElement("box"))
        {
          meshPath = "BOX";
          meshScale << 1,1,1;

          const ::sdf::ElementPtr collisionGeometry =
            sdf_geometry->GetElement("box");
          double x = collisionGeometry->Get<double>("x");
          double y = collisionGeometry->Get<double>("y");
          double z = collisionGeometry->Get<double>("z");
          geometry = boost::shared_ptr < fcl::CollisionGeometry > (new fcl::Box (x, y, z));
        }
        // Handle the case where collision geometry is a sphere.
        else if (sdf_geometry->HasElement("sphere"))
        {
          meshPath = "SPHERE";
          meshScale << 1,1,1;
          const ::sdf::ElementPtr collisionGeometry =
            sdf_geometry->GetElement("sphere");
          
          double radius = collisionGeometry->Get<double>("radius");
          geometry =
            boost::shared_ptr < fcl::CollisionGeometry > (new fcl::Sphere (radius));
        }
        else throw std::invalid_argument("Unknown geometry type :");

        if (!geometry)
        {
          throw std::invalid_argument("The polyhedron retrived is empty");
        }
        return geometry;
      }
      
      template<GeometryType type>
      static void addLinkGeometryToGeomModel(const SdfGraph & graph,
                                             ::hpp::fcl::MeshLoaderPtr & meshLoader,
                                             ::sdf::ElementPtr link,
                                             GeometryModel & geomModel,
                                             const std::vector<std::string> & package_dirs)
      {

        typedef std::vector< ::sdf::ElementPtr> GeometryArray;
        typedef GeometryModel::SE3 SE3;

        bool has_element = false;
        if(hasLinkElement<type>)
        {
          std::string meshPath = "";

          Eigen::Vector3d meshScale(Eigen::Vector3d::Ones());
          const std::string link_name = link->Get<std::string>("name");

          const GeometryArray geometries_array =
            getLinkGeometryArray<type>(link);

          FrameIndex frame_id = graph.urdfVisitor.getBodyId(link_name);
          Frame frame = graph.urdfVisitor.getBodyFrame (link_name);
          
          SE3 body_placement = frame.placement;

          std::size_t objectId = 0;
          for (typename GeometryArray::const_iterator i = geometries_array.begin();
               i != geometries_array.end(); ++i)
          {
            meshPath.clear();
#ifdef PINOCCHIO_WITH_HPP_FCL
            const std::string geom_name = (*i)->template Get<std::string>("name");

            const ::sdf::ElementPtr sdf_geometry = (*i)->GetElement("geometry");
            
            const GeometryObject::CollisionGeometryPtr geometry =
              retrieveCollisionGeometry(graph, meshLoader, link_name, geom_name,
                                        sdf_geometry, package_dirs, meshPath, meshScale);
#else
            const ::sdf::ElementPtr sdf_mesh = sdf_geometry->GetElement("mesh");

            if (sdf_mesh)
            {
              std::string collisionFilename = sdf_mesh->Get<std::string>("url");
              meshPath = retrieveResourcePath(collisionFilename, package_dirs);

              const ignition::math::Vector3d ign_scale =
                sdf_mesh->Get<ignition::math::Vector3d>("scale");
              
              const fcl::Vec3f scale = fcl::Vec3f(ign_scale.X(), ign_scale.Y(),
                                                  ign_scale.Z());
              retrieveMeshScale(scale, meshScale);
            }

            const boost::shared_ptr<fcl::CollisionGeometry>
              geometry(new fcl::CollisionGeometry());
#endif // PINOCCHIO_WITH_HPP_FCL
           
            const ignition::math::Pose3d& pose =
              (*i)->template Get<ignition::math::Pose3d>("pose");

            Eigen::Vector4d meshColor(Eigen::Vector4d::Zero());
            std::string meshTexturePath;
            bool overrideMaterial = false;
            const ::sdf::ElementPtr sdf_material = (*i)->GetElement("material");
            if (sdf_material)
            {
              const ignition::math::Vector4d ign_meshColor =
                sdf_material->Get<ignition::math::Vector4d>("ambient");
              
              meshColor << ign_meshColor.X(),
                ign_meshColor.Y(),
                ign_meshColor.Z(),
                ign_meshColor.W();
              overrideMaterial = true;
            }

            const SE3 lMg = convertFromPose3d(pose);
            const SE3 geomPlacement = body_placement * lMg;
            std::ostringstream geometry_object_suffix;
            geometry_object_suffix << "_" << objectId;
            const std::string & geometry_object_name = std::string(link_name + geometry_object_suffix.str());
            GeometryObject geometry_object(geometry_object_name,
                                           frame.parentJoint, frame_id,
                                           geomPlacement, geometry,
                                           meshPath, meshScale,
                                           overrideMaterial, meshColor, meshTexturePath);
            geomModel.addGeometryObject(geometry_object);
            ++objectId;
          }
        }
      }

      
      /**
       * @brief      Recursive procedure for reading the URDF tree, looking for geometries
       *             This function fill the geometric model whith geometry objects retrieved from the URDF tree
       *
       * @param[in]  tree           The URDF kinematic tree
       * @param[in]  meshLoader     The FCL mesh loader to avoid duplications of already loaded geometries
       * @param[in]  link           The current URDF link
       * @param      model          The model to which is the GeometryModel associated
       * @param      geomModel      The GeometryModel where the Collision Objects must be added
       * @param[in]  package_dirs   A vector containing the different directories where to search for packages
       * @param[in]  type           The type of objects that must be loaded ( can be VISUAL or COLLISION)
       *
       */
      PINOCCHIO_DLLAPI void recursiveParseGraphForGeom(const SdfGraph& graph,
                                         ::hpp::fcl::MeshLoaderPtr& meshLoader,
                                         const ::sdf::ElementPtr link,
                                         GeometryModel & geomModel,
                                         const std::vector<std::string> & package_dirs,
                                         const GeometryType type);

      PINOCCHIO_DLLAPI void parseTreeForGeom(const SdfGraph& graph,
                                             GeometryModel & geomModel,
                                             const GeometryType type,
                                             const std::vector<std::string> & package_dirs,
                                             ::hpp::fcl::MeshLoaderPtr meshLoader);
    } // namespace details

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel& buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & const_model,
                             PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                             const std::string & filename,
                             const GeometryType type,
                             GeometryModel & geomModel,
                             const std::vector<std::string> & package_dirs,
                             ::hpp::fcl::MeshLoaderPtr meshLoader)
      
    {
      Model& model = const_cast<Model &>(const_model); //TODO: buildGeom should not need to parse model again.
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      ::pinocchio::sdf::details::SdfGraph graph (visitor, contact_models);
      //if (verbose) visitor.log = &std::cout;

      //Create maps from the SDF Graph
      graph.parseGraph(filename);
      
      details::parseTreeForGeom (graph, geomModel, type,
                                 package_dirs, meshLoader);
      return geomModel;
    }

  } // namespace sdf
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_parsers_sdf_geometry_hxx__
