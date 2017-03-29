//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.


#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/utils.hpp"
#include "pinocchio/parsers/utils.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#ifdef WITH_HPP_FCL
#include <hpp/fcl/mesh_loader/assimp.h>
#endif // WITH_HPP_FCL

namespace se3
{
  namespace urdf
  {
    namespace details
    {
#ifdef WITH_HPP_FCL      
      typedef fcl::BVHModel< fcl::OBBRSS > PolyhedronType;
      typedef boost::shared_ptr <PolyhedronType> PolyhedronPtrType;

      /**
     * @brief      Get a fcl::CollisionObject from an urdf geometry, searching
     *             for it in specified package directories
     *
     * @param[in]  urdf_geometry  A shared pointer on the input urdf Geometry
     * @param[in]  package_dirs   A vector containing the different directories where to search for packages
     * @param[out] meshPath      The Absolute path of the mesh currently read
     * @param[out] meshScale     Scale of transformation currently applied to the mesh
     *
     * @return     A shared pointer on the he geometry converted as a fcl::CollisionGeometry
     */
     boost::shared_ptr<fcl::CollisionGeometry> retrieveCollisionGeometry(const ::urdf::GeometrySharedPtr urdf_geometry,
                                                                         const std::vector<std::string> & package_dirs,
                                                                         std::string & meshPath,
                                                                         Eigen::Vector3d & meshScale)
      {
        boost::shared_ptr<fcl::CollisionGeometry> geometry;

        // Handle the case where collision geometry is a mesh
        if (urdf_geometry->type == ::urdf::Geometry::MESH)
        {
          const ::urdf::MeshSharedPtr collisionGeometry = ::urdf::dynamic_pointer_cast< ::urdf::Mesh> (urdf_geometry);
          std::string collisionFilename = collisionGeometry->filename;

          meshPath = retrieveResourcePath(collisionFilename, package_dirs);
          if (meshPath == "") {
            std::stringstream ss;
            ss << "Mesh " << collisionFilename << " could not be found.";
            throw std::invalid_argument (ss.str());
          }

          fcl::Vec3f scale = fcl::Vec3f(collisionGeometry->scale.x,
					collisionGeometry->scale.y,
					collisionGeometry->scale.z
					);

	  meshScale << collisionGeometry->scale.x,
	    collisionGeometry->scale.y,
	    collisionGeometry->scale.z;

          // Create FCL mesh by parsing Collada file.
          PolyhedronPtrType polyhedron (new PolyhedronType);

          fcl::loadPolyhedronFromResource (meshPath, scale, polyhedron);
          geometry = polyhedron;
        }

        // Handle the case where collision geometry is a cylinder
        // Use FCL capsules for cylinders
        else if (urdf_geometry->type == ::urdf::Geometry::CYLINDER)
        {
          meshPath = "CYLINDER";
          meshScale << 1,1,1;
          const ::urdf::CylinderSharedPtr collisionGeometry = ::urdf::dynamic_pointer_cast< ::urdf::Cylinder> (urdf_geometry);
    
          double radius = collisionGeometry->radius;
          double length = collisionGeometry->length;
    
          // Create fcl capsule geometry.
          geometry = boost::shared_ptr < fcl::CollisionGeometry >(new fcl::Capsule (radius, length));
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
        else throw std::runtime_error (std::string ("Unknown geometry type :"));

        if (!geometry)
        {
          throw std::runtime_error(std::string("The polyhedron retrived is empty"));
        }

        return geometry;
      }
#endif // WITH_HPP_FCL

     /**
      * @brief Get the first geometry attached to a link
      *
      * @param[in] link   The URDF link
      *
      * @return Either the first collision or visual
      */
      template<typename T>
      inline const URDF_SHARED_PTR(T)
      getLinkGeometry(const ::urdf::LinkConstSharedPtr link);

      template<>
      inline const ::urdf::CollisionSharedPtr
      getLinkGeometry< ::urdf::Collision>(const ::urdf::LinkConstSharedPtr link)
      {
        return link->collision;
      }

      template<>
      inline const ::urdf::VisualSharedPtr
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
      inline bool getVisualMaterial(const URDF_SHARED_PTR(urdfObject) urdf_object,std::string & meshTexturePath,
				    Eigen::Vector4d & meshColor, const std::vector<std::string> & package_dirs);

      template<>
      inline bool getVisualMaterial< ::urdf::Collision>(const ::urdf::CollisionSharedPtr, std::string& meshTexturePath,
							Eigen::Vector4d & meshColor, const std::vector<std::string> &)
      {
        meshColor.setZero();
        meshTexturePath = "";
        return false;
      }
      
      template<>
      inline bool getVisualMaterial< ::urdf::Visual>(const ::urdf::VisualSharedPtr urdf_visual, std::string& meshTexturePath,
						     Eigen::Vector4d & meshColor, const std::vector<std::string> & package_dirs)
      {
        meshColor.setZero();
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
      inline const std::vector< URDF_SHARED_PTR(T) > &
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
     * @brief      Add the geometries attached to an urdf link to  a GeometryModel, looking
     *             either for collisions or visuals
     * 
     * @param[in]  link            The current URDF link
     * @param      model           The model to which is the GeometryModel associated
     * @param      model_geom      The GeometryModel where the Collision Objects must be added
     * @param[in]  package_dirs    A vector containing the different directories where to search for packages
     * @param[in]  type            The type of objects that must be loaded ( can be VISUAL or COLLISION)
     */
      template<typename T>
      inline void addLinkGeometryToGeomModel(::urdf::LinkConstSharedPtr link,
                                             const Model & model,
                                             GeometryModel & geomModel,
                                             const std::vector<std::string> & package_dirs) throw (std::invalid_argument)
      {
        typedef std::vector< URDF_SHARED_PTR(T) > VectorSharedT;
        if(getLinkGeometry<T>(link))
        {
          std::string meshPath = "";

          Eigen::Vector3d meshScale;
        
          const std::string & link_name = link->name;

          VectorSharedT geometries_array = getLinkGeometryArray<T>(link);

          if (!model.existFrame(link_name, BODY))
            throw std::invalid_argument("No link " + link_name + " in model");
          FrameIndex frame_id = model.getFrameId(link_name, BODY);
          SE3 body_placement = model.frames[frame_id].placement;
          assert(model.frames[frame_id].type == BODY);

          std::size_t objectId = 0;
          for (typename VectorSharedT::const_iterator i = geometries_array.begin();i != geometries_array.end(); ++i)
          {
            meshPath.clear();
#ifdef WITH_HPP_FCL
            const boost::shared_ptr<fcl::CollisionGeometry> geometry = retrieveCollisionGeometry((*i)->geometry, package_dirs, meshPath, meshScale);
#else
            ::urdf::MeshSharedPtr urdf_mesh = ::urdf::dynamic_pointer_cast< ::urdf::Mesh> ((*i)->geometry);
            if (urdf_mesh) meshPath = retrieveResourcePath(urdf_mesh->filename, package_dirs);
            
            const boost::shared_ptr<fcl::CollisionGeometry> geometry(new fcl::CollisionGeometry());
#endif // WITH_HPP_FCL

            Eigen::Vector4d meshColor;
            std::string meshTexturePath;
            bool overrideMaterial = getVisualMaterial<T>((*i), meshTexturePath, meshColor, package_dirs);

            SE3 geomPlacement = body_placement * convertFromUrdf((*i)->origin);
            std::ostringstream geometry_object_suffix;
            geometry_object_suffix << "_" << objectId;
            const std::string & geometry_object_name = std::string(link_name + geometry_object_suffix.str());
            geomModel.addGeometryObject(GeometryObject(geometry_object_name,
                                                       frame_id, model.frames[frame_id].parent, 
                                                       geometry,
                                                       geomPlacement, meshPath, meshScale,
                                                       overrideMaterial, meshColor, meshTexturePath),
                                        model);
            ++objectId;
          }
        }
      }

    /**
     * @brief      Recursive procedure for reading the URDF tree, looking for geometries
     *             This function fill the geometric model whith geometry objects retrieved from the URDF tree
     * 
     * @param[in]  link            The current URDF link
     * @param      model           The model to which is the GeometryModel associated
     * @param      geomModel      The GeometryModel where the Collision Objects must be added
     * @param[in]  package_dirs    A vector containing the different directories where to search for packages
     * @param[in]  type            The type of objects that must be loaded ( can be VISUAL or COLLISION)
     */
     void parseTreeForGeom(::urdf::LinkConstSharedPtr link,
                           const Model & model,
                           GeometryModel & geomModel,
                           const std::vector<std::string> & package_dirs,
                           const GeometryType type) throw (std::invalid_argument)
      {

        switch(type)
        {
          case COLLISION:
            addLinkGeometryToGeomModel< ::urdf::Collision >(link, model, geomModel, package_dirs);
          break;
          case VISUAL:
            addLinkGeometryToGeomModel< ::urdf::Visual >(link, model, geomModel, package_dirs);
          break;
          default:
          break;
        }
        
        BOOST_FOREACH(::urdf::LinkConstSharedPtr child,link->child_links)
        {
          parseTreeForGeom(child, model, geomModel, package_dirs,type);
        }

      }

    } // namespace details



    GeometryModel& buildGeom(const Model & model,
                             const std::string & filename,
                             const GeometryType type,
                             GeometryModel & geomModel,
                             const std::vector<std::string> & package_dirs)
      throw(std::invalid_argument)
    {
      std::vector<std::string> hint_directories(package_dirs);

      // Append the ROS_PACKAGE_PATH
      std::vector<std::string> ros_pkg_paths = rosPaths();
      hint_directories.insert(hint_directories.end(), ros_pkg_paths.begin(), ros_pkg_paths.end());

      if(hint_directories.empty())
      {
        throw std::runtime_error("You did not specify any package directory and ROS_PACKAGE_PATH is empty. Geometric parsing will crash");
      }

      ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
      details::parseTreeForGeom(urdfTree->getRoot(), model, geomModel, hint_directories,type);
      return geomModel;
    }

  } // namespace urdf
} // namespace se3
