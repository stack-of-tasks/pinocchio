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

#include <iostream>
#include <boost/foreach.hpp>
#include "pinocchio/multibody/model.hpp"

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include "pinocchio/multibody/parser/from-collada-to-fcl.hpp"

#include <exception>

namespace se3
{
  namespace urdf
  {
    
    inline fcl::CollisionObject retrieveCollisionGeometry (const ::urdf::LinkConstPtr & link,
                                                           const std::string & meshRootDir)
    {
      boost::shared_ptr < ::urdf::Collision> collision = link->collision;
      boost::shared_ptr < fcl::CollisionGeometry > geometry;

      // Handle the case where collision geometry is a mesh
      if (collision->geometry->type == ::urdf::Geometry::MESH)
      {
        boost::shared_ptr < ::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Mesh> (collision->geometry);
        std::string collisionFilename = collisionGeometry->filename;

        std::string full_path = fromURDFMeshPathToAbsolutePath(collisionFilename, meshRootDir);

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
                                 GeometryModel & model_geom,
                                 const std::string & meshRootDir,
                                 const bool rootJointAdded) throw (std::invalid_argument)
    {

      // start with first link that is not empty
      if(link->inertial)
      {
        ::urdf::JointConstPtr joint = link->parent_joint;

        if (joint == NULL && rootJointAdded )
        {
          
          if (link->collision)
          {
            fcl::CollisionObject collision_object = retrieveCollisionGeometry(link, meshRootDir);
            const SE3 geomPlacement = convertFromUrdf(link->collision->origin);
            const std::string & collision_object_name = link->name ;
            model_geom.addGeomObject(model.getJointId("root_joint"), collision_object, geomPlacement, collision_object_name);
          }
        }

        if(joint!=NULL)
        {
          assert(link->getParent()!=NULL);

          if (link->collision)
          {
            fcl::CollisionObject collision_object = retrieveCollisionGeometry(link, meshRootDir);
            const SE3 geomPlacement = convertFromUrdf(link->collision->origin);
            const std::string & collision_object_name = link->name ;
            model_geom.addGeomObject(model.getJointId(joint->name), collision_object, geomPlacement, collision_object_name);
          }      
        }
        else if (link->getParent() != NULL)
        {
          const std::string exception_message (link->name + " - joint information missing.");
          throw std::invalid_argument(exception_message);
        }

      }
      
      BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
      {
        parseTreeForGeom(child, model, model_geom, meshRootDir, rootJointAdded);
      }
    }



    template <typename D>
    std::pair<Model, GeometryModel>
    buildModelAndGeom(const std::string & filename,
                      const std::string & meshRootDir,
                      const JointModelBase<D> & root_joint)
    {
      // Read model
      Model model;
      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      parseTree(urdfTree->getRoot(), model, SE3::Identity(), root_joint);
      
      // Read geometries
      GeometryModel model_geom(model);
      parseTreeForGeom(urdfTree->getRoot(), model, model_geom, meshRootDir, true);
      
      // Return a pair containing the kinematic tree and the geometries
      return std::make_pair(model, model_geom);
    }

    inline std::pair<Model, GeometryModel> buildModelAndGeom(const std::string & filename,
                                                             const std::string & meshRootDir)
    {
      // Read model
      Model model;
      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      parseTree(urdfTree->getRoot(), model, SE3::Identity());
      
      // Read geometries
      GeometryModel model_geom (model);
      parseTreeForGeom(urdfTree->getRoot(), model, model_geom, meshRootDir, false);
      
      // Return a pair containing the kinematic tree and the geometries
      return std::make_pair(model, model_geom);
    }

  } // namespace urdf
} // namespace se3

#endif // ifndef __se3_urdf_geom_hxx__

