//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/types.hpp"
#include "pinocchio/parsers/urdf/utils.hpp"
#include "pinocchio/parsers/utils.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace pinocchio
{
  namespace urdf
  {
    /// \internal
    namespace details
    {
      struct UrdfTree
      {
        typedef boost::property_tree::ptree ptree;
        typedef std::map<std::string, const ptree&> LinkMap_t;
        
        void parse (const std::string & xmlStr)
        {
          urdf_ = ::urdf::parseURDF(xmlStr);
          if (!urdf_) {
            throw std::invalid_argument("Unable to parse URDF");
          }
          
          std::istringstream iss(xmlStr);
          using namespace boost::property_tree;
          read_xml(iss, tree_, xml_parser::no_comments);
          
          BOOST_FOREACH(const ptree::value_type & link, tree_.get_child("robot")) {
            if (link.first == "link") {
              std::string name = link.second.get<std::string>("<xmlattr>.name");
              links_.insert(std::pair<std::string,const ptree&>(name, link.second));
            }
          } // BOOST_FOREACH
        }
        
        bool isCapsule(const std::string & linkName,
                       const std::string & geomName) const
        {
          LinkMap_t::const_iterator _link = links_.find(linkName);
          assert (_link != links_.end());
          const ptree& link = _link->second;
          if (link.count ("collision_checking") == 0)
            return false;
          BOOST_FOREACH(const ptree::value_type & cc, link.get_child("collision_checking")) {
            if (cc.first == "capsule") {
#ifdef PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
              std::cerr << "Warning: support for tag link/collision_checking/capsule"
                " is not available for URDFDOM < 0.3.0" << std::endl;
#else
              std::string name = cc.second.get<std::string>("<xmlattr>.name");
              if (geomName == name) return true;
#endif
            }
          } // BOOST_FOREACH
          
          return false;
        }

        bool isMeshConvex(const std::string & linkName,
                          const std::string & geomName) const
        {
          LinkMap_t::const_iterator _link = links_.find(linkName);
          assert (_link != links_.end());
          const ptree& link = _link->second;
          if (link.count ("collision_checking") == 0)
            return false;
          BOOST_FOREACH(const ptree::value_type & cc, link.get_child("collision_checking")) {
            if (cc.first == "convex") {
#ifdef PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
              std::cerr << "Warning: support for tag link/collision_checking/convex"
                " is not available for URDFDOM < 0.3.0" << std::endl;
#else
              std::string name = cc.second.get<std::string>("<xmlattr>.name");
              if (geomName == name) return true;
#endif
            }
          } // BOOST_FOREACH
          
          return false;
        }
        
        // For standard URDF tags
        ::urdf::ModelInterfaceSharedPtr urdf_;
        // For other tags
        ptree tree_;
        // A mapping from link name to corresponding child of tree_
        LinkMap_t links_;
      };

      template<typename Vector3>
      static void retrieveMeshScale(const ::urdf::MeshSharedPtr & mesh,
                                    const Eigen::MatrixBase<Vector3> & scale)
      {
        Vector3 & scale_ = PINOCCHIO_EIGEN_CONST_CAST(Vector3,scale);
        scale_ <<
        mesh->scale.x,
        mesh->scale.y,
        mesh->scale.z;
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
# if ( HPP_FCL_MAJOR_VERSION>1 || ( HPP_FCL_MAJOR_VERSION==1 && \
      ( HPP_FCL_MINOR_VERSION>1 || ( HPP_FCL_MINOR_VERSION==1 && \
                                     HPP_FCL_PATCH_VERSION>3))))
#  define PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3
# endif

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
      inline retrieveCollisionGeometry(const UrdfTree& tree,
                                       fcl::MeshLoaderPtr& meshLoader,
                                       const std::string& linkName,
                                       const std::string& geomName,
                                       const ::urdf::GeometrySharedPtr urdf_geometry,
                                       const std::vector<std::string> & package_dirs,
                                       std::string & meshPath,
                                       Eigen::Vector3d & meshScale)
      {
        boost::shared_ptr<fcl::CollisionGeometry> geometry;

        // Handle the case where collision geometry is a mesh
        if (urdf_geometry->type == ::urdf::Geometry::MESH)
        {
          const ::urdf::MeshSharedPtr urdf_mesh = ::urdf::dynamic_pointer_cast< ::urdf::Mesh> (urdf_geometry);
          std::string collisionFilename = urdf_mesh->filename;
          
          meshPath = retrieveResourcePath(collisionFilename, package_dirs);
          if (meshPath == "") {
            std::stringstream ss;
            ss << "Mesh " << collisionFilename << " could not be found.";
            throw std::invalid_argument (ss.str());
          }
          
          fcl::Vec3f scale = fcl::Vec3f(urdf_mesh->scale.x,
                                        urdf_mesh->scale.y,
                                        urdf_mesh->scale.z
                                        );
          
          retrieveMeshScale(urdf_mesh, meshScale);
          
          // Create FCL mesh by parsing Collada file.
#ifdef PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3
          hpp::fcl::BVHModelPtr_t bvh = meshLoader->load (meshPath, scale);
          bool convex = tree.isMeshConvex (linkName, geomName);
          if (convex) {
            bvh->buildConvexRepresentation (false);
            geometry = bvh->convex;
          } else
            geometry = bvh;
#else
          geometry = meshLoader->load (meshPath, scale);
#endif
        }

        // Handle the case where collision geometry is a cylinder
        // Use FCL capsules for cylinders
        else if (urdf_geometry->type == ::urdf::Geometry::CYLINDER)
        {
          const bool is_capsule = tree.isCapsule(linkName, geomName);
          meshScale << 1,1,1;
          const ::urdf::CylinderSharedPtr collisionGeometry = ::urdf::dynamic_pointer_cast< ::urdf::Cylinder> (urdf_geometry);
          
          double radius = collisionGeometry->radius;
          double length = collisionGeometry->length;
          
          // Create fcl capsule geometry.
          if (is_capsule) {
            meshPath = "CAPSULE";
            geometry = boost::shared_ptr < fcl::CollisionGeometry >(new fcl::Capsule (radius, length));
          } else {
            meshPath = "CYLINDER";
            geometry = boost::shared_ptr < fcl::CollisionGeometry >(new fcl::Cylinder (radius, length));
          }
        }
        // Handle the case where collision geometry is a box.
        else if (urdf_geometry->type == ::urdf::Geometry::BOX) 
        {
          meshPath = "BOX";
          meshScale << 1,1,1;
          const ::urdf::BoxSharedPtr collisionGeometry = ::urdf::dynamic_pointer_cast< ::urdf::Box> (urdf_geometry);
          
          double x = collisionGeometry->dim.x;
          double y = collisionGeometry->dim.y;
          double z = collisionGeometry->dim.z;
          
          geometry = boost::shared_ptr < fcl::CollisionGeometry > (new fcl::Box (x, y, z));
        }
        // Handle the case where collision geometry is a sphere.
        else if (urdf_geometry->type == ::urdf::Geometry::SPHERE)
        {
          meshPath = "SPHERE";
          meshScale << 1,1,1;
          const ::urdf::SphereSharedPtr collisionGeometry = ::urdf::dynamic_pointer_cast< ::urdf::Sphere> (urdf_geometry);
          
          double radius = collisionGeometry->radius;
          
          geometry = boost::shared_ptr < fcl::CollisionGeometry > (new fcl::Sphere (radius));
        }
        else throw std::invalid_argument("Unknown geometry type :");
        
        if (!geometry)
        {
          throw std::invalid_argument("The polyhedron retrived is empty");
        }

        return geometry;
      }
#endif // PINOCCHIO_WITH_HPP_FCL

     /**
      * @brief Get the first geometry attached to a link
      *
      * @param[in] link   The URDF link
      *
      * @return Either the first collision or visual
      */
      template<typename T>
      inline PINOCCHIO_URDF_SHARED_PTR(const T)
      getLinkGeometry(const ::urdf::LinkConstSharedPtr link);
      
      template<>
      inline ::urdf::CollisionConstSharedPtr
      getLinkGeometry< ::urdf::Collision>(const ::urdf::LinkConstSharedPtr link)
      {
        return link->collision;
      }
      
      template<>
      inline ::urdf::VisualConstSharedPtr
      getLinkGeometry< ::urdf::Visual>(const ::urdf::LinkConstSharedPtr link)
      {
        return link->visual;
      }


     /**
      * @brief Get the material values from the link visual object
      *
      * @param[in]  Visual/Collision The Visual or the Collision object.
      * @param[out] meshTexturePath  The absolute file path containing the texture description.
      * @param[out] meshColor        The mesh RGBA vector.
      * @param[in]  package_dirs     A vector containing the different directories where to search for packages
      *
      */
      template<typename urdfObject>
      inline bool getVisualMaterial(const PINOCCHIO_URDF_SHARED_PTR(urdfObject) urdf_object,std::string & meshTexturePath,
                                    Eigen::Vector4d & meshColor, const std::vector<std::string> & package_dirs);
      
      template<>
      inline bool getVisualMaterial< ::urdf::Collision>
      (const ::urdf::CollisionSharedPtr, std::string& meshTexturePath,
       Eigen::Vector4d & meshColor, const std::vector<std::string> &)
      {
        meshColor << 0.9, 0.9, 0.9, 1.;
        meshTexturePath = "";
        return false;
      }
      
      template<>
      inline bool getVisualMaterial< ::urdf::Visual>
      (const ::urdf::VisualSharedPtr urdf_visual, std::string& meshTexturePath,
       Eigen::Vector4d & meshColor, const std::vector<std::string> & package_dirs)
      {
        meshColor << 0.9, 0.9, 0.9, 1.;
        meshTexturePath = "";
        bool overrideMaterial = false;
        if(urdf_visual->material) {
          overrideMaterial = true;
          meshColor << urdf_visual->material->color.r, urdf_visual->material->color.g,
          urdf_visual->material->color.b, urdf_visual->material->color.a;
          if(urdf_visual->material->texture_filename!="")
            meshTexturePath = retrieveResourcePath((urdf_visual)->material->texture_filename, package_dirs);
        }
        return overrideMaterial;
      }
      
     /**
      * @brief Get the array of geometries attached to a link
      *
      * @param[in] link   The URDF link
      *
      * @return the array of either collisions or visuals
      */
      template<typename T>
      inline const std::vector< PINOCCHIO_URDF_SHARED_PTR(T) > &
      getLinkGeometryArray(const ::urdf::LinkConstSharedPtr link);
      
      template<>
      inline const std::vector< ::urdf::CollisionSharedPtr> &
      getLinkGeometryArray< ::urdf::Collision>(const ::urdf::LinkConstSharedPtr link)
      {
        return link->collision_array;
      }
      
      template<>
      inline const std::vector< ::urdf::VisualSharedPtr> &
      getLinkGeometryArray< ::urdf::Visual>(const ::urdf::LinkConstSharedPtr link)
      {
        return link->visual_array;
      }

      /**
       * @brief      Add the geometries attached to an URDF link to a GeometryModel, looking
       *             either for collisions or visuals
       *
       * @param[in]  tree           The URDF kinematic tree
       * @param[in]  meshLoader     The FCL mesh loader to avoid duplications of already loaded geometries
       * @param[in]  link            The current URDF link
       * @param      model           The model to which is the GeometryModel associated
       * @param      geomModel      The GeometryModel where the Collision Objects must be added
       * @param[in]  package_dirs    A vector containing the different directories where to search for packages
       *
       */
      template<typename GeometryType>
      static void addLinkGeometryToGeomModel(const UrdfTree & tree,
                                             ::hpp::fcl::MeshLoaderPtr & meshLoader,
                                             ::urdf::LinkConstSharedPtr link,
                                             UrdfGeomVisitorBase& visitor,
                                             GeometryModel & geomModel,
                                             const std::vector<std::string> & package_dirs)
      {
#ifndef PINOCCHIO_WITH_HPP_FCL
        PINOCCHIO_UNUSED_VARIABLE(tree);
        PINOCCHIO_UNUSED_VARIABLE(meshLoader);
#endif // PINOCCHIO_WITH_HPP_FCL
        
        typedef std::vector< PINOCCHIO_URDF_SHARED_PTR(GeometryType) > VectorSharedT;
        typedef GeometryModel::SE3 SE3;

        if(getLinkGeometry<GeometryType>(link))
        {
          std::string meshPath = "";

          Eigen::Vector3d meshScale(Eigen::Vector3d::Ones());

          const std::string & link_name = link->name;

          VectorSharedT geometries_array = getLinkGeometryArray<GeometryType>(link);

          FrameIndex frame_id;
          UrdfGeomVisitorBase::Frame frame = visitor.getBodyFrame (link_name, frame_id);
          SE3 body_placement = frame.placement;

          std::size_t objectId = 0;
          for (typename VectorSharedT::const_iterator i = geometries_array.begin();i != geometries_array.end(); ++i)
          {
            meshPath.clear();
#ifdef PINOCCHIO_WITH_HPP_FCL
            
#ifdef PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
            const std::string & geom_name = (*i)->group_name;
#else
            const std::string & geom_name = (*i)->name;
#endif // PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
            const GeometryObject::CollisionGeometryPtr geometry =
            retrieveCollisionGeometry(tree, meshLoader, link_name, geom_name,
                                      (*i)->geometry, package_dirs, meshPath, meshScale);
#else
            ::urdf::MeshSharedPtr urdf_mesh = ::urdf::dynamic_pointer_cast< ::urdf::Mesh> ((*i)->geometry);
            if (urdf_mesh)
            {
              meshPath = retrieveResourcePath(urdf_mesh->filename, package_dirs);
              retrieveMeshScale(urdf_mesh, meshScale);
            }

            const boost::shared_ptr<fcl::CollisionGeometry> geometry(new fcl::CollisionGeometry());
#endif // PINOCCHIO_WITH_HPP_FCL

            Eigen::Vector4d meshColor;
            std::string meshTexturePath;
            bool overrideMaterial = getVisualMaterial<GeometryType>((*i), meshTexturePath, meshColor, package_dirs);

            SE3 geomPlacement = body_placement * convertFromUrdf((*i)->origin);
            std::ostringstream geometry_object_suffix;
            geometry_object_suffix << "_" << objectId;
            const std::string & geometry_object_name = std::string(link_name + geometry_object_suffix.str());
            GeometryObject geometry_object(geometry_object_name,
                                           frame_id, frame.parent,
                                           geometry,
                                           geomPlacement, meshPath, meshScale,
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
      void recursiveParseTreeForGeom(const UrdfTree& tree,
                                     ::hpp::fcl::MeshLoaderPtr& meshLoader,
                                     ::urdf::LinkConstSharedPtr link,
                                     UrdfGeomVisitorBase& visitor,
                                     GeometryModel & geomModel,
                                     const std::vector<std::string> & package_dirs,
                                     const GeometryType type)
      {
        
        switch(type)
        {
          case COLLISION:
            addLinkGeometryToGeomModel< ::urdf::Collision >(tree, meshLoader, link, visitor, geomModel, package_dirs);
            break;
          case VISUAL:
            addLinkGeometryToGeomModel< ::urdf::Visual >(tree, meshLoader, link, visitor, geomModel, package_dirs);
            break;
          default:
            break;
        }
        
        BOOST_FOREACH(::urdf::LinkConstSharedPtr child,link->child_links)
        {
          recursiveParseTreeForGeom(tree, meshLoader, child, visitor, geomModel, package_dirs,type);
        }
        
      }

      void parseTreeForGeom(UrdfGeomVisitorBase& visitor,
                            const std::istream& xmlStream,
                            const GeometryType type,
                            GeometryModel & geomModel,
                            const std::vector<std::string> & package_dirs,
                            ::hpp::fcl::MeshLoaderPtr meshLoader)
      {
        std::string xmlStr;
        {
          std::ostringstream os;
          os << xmlStream.rdbuf();
          xmlStr = os.str();
        }
        
        details::UrdfTree tree;
        tree.parse (xmlStr);
        
        std::vector<std::string> hint_directories(package_dirs);
        
        // Append the ROS_PACKAGE_PATH
        std::vector<std::string> ros_pkg_paths = rosPaths();
        hint_directories.insert(hint_directories.end(), ros_pkg_paths.begin(), ros_pkg_paths.end());

#ifdef PINOCCHIO_WITH_HPP_FCL
        if (!meshLoader) meshLoader = fcl::MeshLoaderPtr(new fcl::MeshLoader);
#endif // ifdef PINOCCHIO_WITH_HPP_FCL
        
        recursiveParseTreeForGeom(tree, meshLoader, tree.urdf_->getRoot(),
            visitor, geomModel, hint_directories,type);
      }
    } // namespace details
    /// \endinternal
  } // namespace urdf
} // namespace pinocchio
