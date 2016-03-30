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

#ifndef __se3_urdf_geom_hxx__
#define __se3_urdf_geom_hxx__

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <boost/foreach.hpp>
#include "pinocchio/multibody/model.hpp"

#include "pinocchio/multibody/parser/from-collada-to-fcl.hpp"

namespace se3
{
  namespace urdf
  {
    
    inline fcl::CollisionObject retrieveCollisionGeometry (const ::urdf::LinkConstPtr & link, const std::vector < std::string > & package_dirs)
    {
      boost::shared_ptr < ::urdf::Collision> collision = link->collision;
      boost::shared_ptr < fcl::CollisionGeometry > geometry;

      // Handle the case where collision geometry is a mesh
      if (collision->geometry->type == ::urdf::Geometry::MESH)
      {
        boost::shared_ptr < ::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Mesh> (collision->geometry);
        std::string collisionFilename = collisionGeometry->filename;

        std::string full_path = convertURDFMeshPathToAbsolutePath(collisionFilename, package_dirs);

        ::urdf::Vector3 scale = collisionGeometry->scale;

        // Create FCL mesh by parsing Collada file.
        Polyhedron_ptr polyhedron (new PolyhedronType);

        loadPolyhedronFromResource (full_path, scale, polyhedron);
        geometry = polyhedron;
      }

      // Handle the case where collision geometry is a cylinder
      // Use FCL capsules for cylinders
      else if (collision->geometry->type == ::urdf::Geometry::CYLINDER)
      {
        boost::shared_ptr < ::urdf::Cylinder> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Cylinder> (collision->geometry);
  
        double radius = collisionGeometry->radius;
        double length = collisionGeometry->length;
  
        // Create fcl capsule geometry.
        geometry = boost::shared_ptr < fcl::CollisionGeometry >(new fcl::Capsule (radius, length));
      }
      // Handle the case where collision geometry is a box.
      else if (collision->geometry->type == ::urdf::Geometry::BOX) 
      {
        boost::shared_ptr < ::urdf::Box> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Box> (collision->geometry);
  
        double x = collisionGeometry->dim.x;
        double y = collisionGeometry->dim.y;
        double z = collisionGeometry->dim.z;
  
        geometry = boost::shared_ptr < fcl::CollisionGeometry > (new fcl::Box (x, y, z));
      }
      // Handle the case where collision geometry is a sphere.
      else if (collision->geometry->type == ::urdf::Geometry::SPHERE)
      {
        boost::shared_ptr < ::urdf::Sphere> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Sphere> (collision->geometry);

        double radius = collisionGeometry->radius;

        geometry = boost::shared_ptr < fcl::CollisionGeometry > (new fcl::Sphere (radius));
      }
      else throw std::runtime_error (std::string ("Unknown geometry type :"));

      if (!geometry)
      {
        throw std::runtime_error(std::string("The polyhedron retrived is empty"));
      }
      fcl::CollisionObject collisionObject (geometry, fcl::Transform3f());

      return collisionObject;
    }


    inline void parseTreeForGeom(::urdf::LinkConstPtr link,
                                 const Model & model,
                                 GeometryModel & geom_model,
                                 const std::vector<std::string> & package_dirs) throw (std::invalid_argument)
    {

      // start with first link that is not empty
      if(link->collision)
      {


        assert(link->getParent()!=NULL);

        fcl::CollisionObject collision_object = retrieveCollisionGeometry(link, package_dirs);
        SE3 geomPlacement = convertFromUrdf(link->collision->origin);
        std::string collision_object_name = link->name ;
        geom_model.addGeomObject(model.parents[model.getBodyId(collision_object_name)], collision_object, geomPlacement, collision_object_name);     

        if (link->getParent() == NULL)
        {
          const std::string exception_message (link->name + " - joint information missing.");
          throw std::invalid_argument(exception_message);
        }

      } // if(link->collision)
      
      BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
      {
        parseTreeForGeom(child, model, geom_model, package_dirs);
      }

    }


    inline GeometryModel buildGeom(const Model & model,
                                   const std::string & filename,
                                   const std::vector<std::string> & package_dirs) throw(std::invalid_argument)
    {
      GeometryModel model_geom(model);

      std::vector<std::string> hint_directories(package_dirs);

      // Append the ROS_PACKAGE_PATH
      std::vector<std::string> ros_pkg_paths = extractPathFromEnvVar("ROS_PACKAGE_PATH");
      hint_directories.insert(hint_directories.end(), ros_pkg_paths.begin(), ros_pkg_paths.end());

      if(hint_directories.empty())
      {
        throw std::runtime_error("You did not specify any package directory and ROS_PACKAGE_PATH is empty. Geometric parsing will crash");
      }

      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      parseTreeForGeom(urdfTree->getRoot(), model, model_geom, hint_directories);
      return model_geom;
    }


  } // namespace urdf
} // namespace se3

#endif // ifndef __se3_urdf_geom_hxx__

