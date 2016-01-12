//
// Copyright (c) 2015 CNRS
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


    inline fcl::CollisionObject retrieveCollisionGeometry (const ::urdf::LinkConstPtr & link, const std::string & meshRootDir)
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
        PolyhedronPtrType  polyhedron (new PolyhedronType);

        loadPolyhedronFromResource (full_path, scale, polyhedron);
        geometry = polyhedron;
      }

      if (!geometry)
      {
        throw std::runtime_error(std::string("The polyhedron retrived is empty"));
      }
      fcl::CollisionObject collisionObject (geometry, fcl::Transform3f());

      return collisionObject; // TO CHECK: what happens if geometry is empty ?
    }


    inline void parseTreeForGeom( ::urdf::LinkConstPtr link, Model & model,GeometryModel & model_geom, const std::string & meshRootDir) throw (std::invalid_argument)
    {

      ::urdf::JointConstPtr joint = link->parent_joint;

      if(joint!=NULL)
      {
        assert(link->getParent()!=NULL);

        if (link->collision)
        {
          fcl::CollisionObject collision_object = retrieveCollisionGeometry(link, meshRootDir);
          SE3 geomPlacement = convertFromUrdf(link->collision->origin);
          std::string collision_object_name = link->name ;
          model_geom.addGeomObject(model.getJointId(joint->name), collision_object, geomPlacement, collision_object_name);
        }      
      }
      else if (link->getParent() != NULL)
      {
        const std::string exception_message (link->name + " - joint information missing.");
        throw std::invalid_argument(exception_message);
      }

      BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
      {
        parseTreeForGeom(child, model, model_geom, meshRootDir);
      }
    }



    template <typename D>
    std::pair<Model, GeometryModel > buildModelAndGeom( const std::string & filename, const std::string & meshRootDir, const JointModelBase<D> &  root_joint )
    {
      Model model; GeometryModel model_geom;

      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      parseTree(urdfTree->getRoot(), model, SE3::Identity(), root_joint);
      parseTreeForGeom(urdfTree->getRoot(), model, model_geom, meshRootDir);
      return std::make_pair(model, model_geom);
    }

    inline std::pair<Model, GeometryModel > buildModelAndGeom( const std::string & filename, const std::string & meshRootDir)
    {
      Model model; GeometryModel model_geom;

      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      parseTree(urdfTree->getRoot(), model, SE3::Identity());
      parseTreeForGeom(urdfTree->getRoot(), model, model_geom, meshRootDir);
      return std::make_pair(model, model_geom);
    }

  } // namespace urdf
} // namespace se3

#endif // ifndef __se3_urdf_geom_hxx__

