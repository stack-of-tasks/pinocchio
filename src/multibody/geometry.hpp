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

#ifndef __se3_geom_hpp__
#define __se3_geom_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include <iostream>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <map>
#include <list>


namespace se3
{
  class GeometryModel
  {
  public:
    typedef Model::Index Index;

    Index ngeom;
    std::vector<fcl::CollisionObject> collision_objects;
    std::vector<std::string> geom_names;
    std::vector<Index> geom_parents;           // Joint parent of body <i>, denoted <li> (li==parents[i])
    std::vector<SE3> geometryPlacement;                      // Position of geometry object in parent joint's frame
    
    std::map < Index, std::list<Index> >  innerObjects;        // Associate a list of CollisionObjects to a given joint Id 
    std::map < Index, std::list<Index> >  outerObjects;        // Associate a list of CollisionObjects to a given joint Id 

    GeometryModel()
      : ngeom(0)
      , collision_objects()
      , geom_names(0)
      , geom_parents(0)
      , geometryPlacement(0)
      , innerObjects()
      , outerObjects()
    {
    }

    ~GeometryModel() {};

    Index addGeomObject(  Index parent,const fcl::CollisionObject & co, const SE3 & placement, const std::string & geoName = "");
    Index getGeomId( const std::string & name ) const;
    bool existGeomName( const std::string & name ) const;
    const std::string& getGeomName( Index index ) const;

    void addInnerObject(const Index& joint, const Index& inner_object);
    void addOutterObject(const Index& joint, const Index& outer_object);

    friend std::ostream& operator<<(std::ostream& os, const GeometryModel& model_geom);

  private:
    
  };

  class GeometryData
  {
  public:
    typedef Model::Index Index;

    Data& data_ref;
    GeometryModel& model_geom;

    std::vector<se3::SE3> oMg;
    std::vector<fcl::Transform3f> oMg_fcl;

    GeometryData(Data& data, GeometryModel& model_geom)
        : data_ref(data)
        , model_geom(model_geom)
        , oMg(model_geom.ngeom)
        , oMg_fcl(model_geom.ngeom)
    {
    }

    ~GeometryData() {};

    bool collide(const Index& co1, const Index& co2);

    fcl::DistanceResult computeDistance(const Index& co1, const Index& co2);

    friend std::ostream& operator<<(std::ostream& os, const GeometryData& data_geom);
  private:
    
  };

  inline GeometryModel::Index GeometryModel::addGeomObject(  Index parent,const fcl::CollisionObject & co, const SE3 & placement, const std::string & geoName )
  {

    Index idx = (Index) (ngeom ++);


    collision_objects    .push_back(co);
    geom_parents         .push_back(parent);
    geometryPlacement                  .push_back(placement);
    geom_names           .push_back( (geoName!="")?geoName:random(8));

    return idx;
  }
  inline GeometryModel::Index GeometryModel::getGeomId( const std::string & name ) const
  {
    std::vector<std::string>::iterator::difference_type
      res = std::find(geom_names.begin(),geom_names.end(),name) - geom_names.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
    assert( (res>=0)&&(res<(long)collision_objects.size())&&"The joint name you asked do not exist" );
    return Index(res);
  }
  inline bool GeometryModel::existGeomName( const std::string & name ) const
  {
    return (geom_names.end() != std::find(geom_names.begin(),geom_names.end(),name));
  }
  inline const std::string& GeometryModel::getGeomName( Index index ) const
  {
    assert( index < (Index)collision_objects.size() );
    return geom_names[index];
  }

  inline void GeometryModel::addInnerObject(const Index& joint, const Index& inner_object)
  {
    innerObjects[joint].push_back(inner_object);
  }

  inline void GeometryModel::addOutterObject(const Index& joint, const Index& outer_object)
  {
    outerObjects[joint].push_back(outer_object);
  }

  inline std::ostream& operator<<(std::ostream& os, const GeometryModel& model_geom)
  {
    os << "Nb collision objects = " << model_geom.ngeom << std::endl;
    
    for(GeometryModel::Index i=0;i<(GeometryModel::Index)(model_geom.ngeom);++i)
    {
      os  << "Object n " << i << " : " << model_geom.geom_names[i] << ": attached to joint = " << model_geom.geom_parents[i]
          << "\nwith offset \n" << model_geom.geometryPlacement[i] <<std::endl;
    }

    return os;
  }

  inline std::ostream& operator<<(std::ostream& os, const GeometryData& data_geom)
  {

    for(GeometryData::Index i=0;i<(GeometryData::Index)(data_geom.model_geom.ngeom);++i)
    {
      os << "collision object oMi " << data_geom.oMg[i] << std::endl;
    }

    return os;
  }


  inline bool GeometryData::collide(const Index& co1, const Index& co2) 
  {
    fcl::CollisionRequest collisionRequest (1, false, false, 1, false, true, fcl::GST_INDEP);
    fcl::CollisionResult collisionResult;

    if (fcl::collide (model_geom.collision_objects[co1].collisionGeometry().get(), oMg_fcl[co1],
                      model_geom.collision_objects[co2].collisionGeometry().get(), oMg_fcl[co2],
                      collisionRequest, collisionResult) != 0)
    {
      std::cout << "Collision between " << model_geom.getGeomName(co1) << " and " << model_geom.getGeomName(co2) << std::endl;;
      return true;
    }
    return false;
  }

  inline fcl::DistanceResult GeometryData::computeDistance(const Index& co1, const Index& co2)
  {
    fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
    fcl::DistanceResult result;
    fcl::distance ( model_geom.collision_objects[co1].collisionGeometry().get(), oMg_fcl[co1],
                    model_geom.collision_objects[co2].collisionGeometry().get(), oMg_fcl[co2],
                    distanceRequest, result);
    return result;
  }

} // namespace se3

#endif // ifndef __se3_geom_hpp__
