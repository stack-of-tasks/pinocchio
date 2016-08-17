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

#ifndef __se3_geom_hpp__
#define __se3_geom_hpp__


#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <map>
#include <list>
#include <utility>
#include <assert.h>


namespace se3
{
  
  struct GeometryModel
  {
    
    typedef std::vector<GeomIndex> GeomIndexList;

    /// \brief The number of GeometryObjects
    Index ngeoms;

    /// \brief Vector of GeometryObjects used for collision computations
    std::vector<GeometryObject> geometryObjects;
    ///
    /// \brief Vector of collision pairs.
    ///
    CollisionPairsVector_t collisionPairs;
  
    /// \brief A list of associated collision GeometryObjects to a given joint Id.
    ///        Inner objects can be seen as geometry objects that directly move when the associated joint moves
    std::map < JointIndex, GeomIndexList >  innerObjects;

    /// \brief A list of associated collision GeometryObjects to a given joint Id
    ///        Outer objects can be seen as geometry objects that may often be obstacles to the Inner objects of given joint
    std::map < JointIndex, GeomIndexList >  outerObjects;

    GeometryModel()
      : ngeoms(0)
      , geometryObjects()
      , collisionPairs()
      , innerObjects()
      , outerObjects()
    { 
      const std::size_t num_max_collision_pairs = (ngeoms * (ngeoms-1))/2;
      collisionPairs.reserve(num_max_collision_pairs);
    }

    ~GeometryModel() {};

    /**
     * @brief      Add a geometry object to a GeometryModel
     *
     * @param[in]  parent     Index of the parent frame
     * @param[in]  co         The actual fcl CollisionGeometry
     * @param[in]  placement  The relative placement regarding to the parent frame
     * @param[in]  geom_name  The name of the Geometry Object
     * @param[in]  mesh_path  The absolute path to the mesh
     *
     * @return     The index of the new added GeometryObject in geometryObjects
     */
    inline GeomIndex addGeometryObject(const Model& model,
                                       const FrameIndex parent, const boost::shared_ptr<fcl::CollisionGeometry> & co,
                                       const SE3 & placement, const std::string & geom_name = "",
                                       const std::string & mesh_path = "") throw(std::invalid_argument);



    /**
     * @brief      Return the index of a GeometryObject given by its name.
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @return     Index of the corresponding GeometryObject
     */
    GeomIndex getGeometryId(const std::string & name) const;

    
    /**
     * @brief      Check if a GeometryObject  given by its name exists.
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @return     True if the GeometryObject exists in the geometryObjects.
     */
    bool existGeometryName(const std::string & name) const;


    /**
     * @brief      Get the name of a GeometryObject given by its index.
     *
     * @param[in]  index  Index of the GeometryObject
     *
     * @return     Name of the GeometryObject
     */
    const std::string & getGeometryName(const GeomIndex index) const;

#ifdef WITH_HPP_FCL
    ///
    /// \brief Add a collision pair into the vector of collision_pairs.
    ///        The method check before if the given CollisionPair is already included.
    ///
    /// \param[in] pair The CollisionPair to add.
    ///
    void addCollisionPair (const CollisionPair & pair);
    
    ///
    /// \brief Add all possible collision pairs.
    ///
    /// \note Collision pairs between geometries of having the same parent joint
    ///       are not added.
    ///
    void addAllCollisionPairs();
   
    ///
    /// \brief Remove if exists the CollisionPair from the vector collision_pairs.
    ///
    /// \param[in] pair The CollisionPair to remove.
    ///
    void removeCollisionPair (const CollisionPair& pair);
    
    ///
    /// \brief Remove all collision pairs from collisionPairs. Same as collisionPairs.clear().
    void removeAllCollisionPairs ();
   
    ///
    /// \brief Check if a collision pair exists in collisionPairs.
    ///        See also findCollisitionPair(const CollisionPair & pair).
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair (const CollisionPair & pair) const;
    
    ///
    /// \brief Return the index of a given collision pair in collisionPairs.
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return The index of the CollisionPair in collisionPairs.
    ///
    Index findCollisionPair (const CollisionPair & pair) const;
    
    /// \brief Display on std::cout the list of pairs (is it really useful?).
    void displayCollisionPairs() const;
#endif // WITH_HPP_FCL

    /**
     * @brief      Associate a GeometryObject of type COLLISION to a joint's inner objects list
     *
     * @param[in]  joint         Index of the joint
     * @param[in]  inner_object  Index of the GeometryObject that will be an inner object
     */
    void addInnerObject(const JointIndex joint, const GeomIndex inner_object);
    
    /**
     * @brief      Associate a GeometryObject of type COLLISION to a joint's outer objects list
     *
     * @param[in]  joint         Index of the joint
     * @param[in]  inner_object  Index of the GeometryObject that will be an outer object
     */
    void addOutterObject(const JointIndex joint, const GeomIndex outer_object);
    friend std::ostream& operator<<(std::ostream & os, const GeometryModel & model_geom);
  }; // struct GeometryModel

  struct GeometryData
  {

    ///
    /// \brief A const reference to the model storing all the geometries.
    ///        See class GeometryModel.
    ///
    const GeometryModel & model_geom;

    ///
    /// \brief Vector gathering the SE3 placements of the geometry objects relative to the world.
    ///        See updateGeometryPlacements to update the placements.
    ///
    /// oMg is used for pinocchio (kinematics) computation but is translated to fcl type
    /// for fcl (collision) computation. The copy is done in collisionObjects[i]->setTransform(.)
    ///
    std::vector<se3::SE3> oMg;
#ifdef WITH_HPP_FCL
    ///
    /// \brief Collision objects (ie a fcl placed geometry).
    ///
    /// The object contains a pointer on the collision geometries contained in geomModel.geometryObjects.
    /// \sa GeometryModel::geometryObjects and GeometryObjects
    ///
    std::vector<fcl::CollisionObject> collisionObjects;

    ///
    /// \brief Vector of collision pairs.
    ///
    std::vector<bool> activeCollisionPairs;

    ///
    /// \brief Vector gathering the result of the distance computation for all the collision pairs.
    ///
    std::vector <DistanceResult> distance_results;
    
    ///
    /// \brief Vector gathering the result of the collision computation for all the collision pairs.
    ///
    std::vector <CollisionResult> collision_results;

    ///
    /// \brief Radius of the bodies, i.e. distance of the further point of the geometry model
    /// attached to the body from the joint center.
    ///
    std::vector<double> radius;
    GeometryData(const GeometryModel & modelGeom)
        : model_geom(modelGeom)
        , oMg(model_geom.ngeoms)
        , activeCollisionPairs(modelGeom.collisionPairs.size(), true)
        , distance_results(modelGeom.collisionPairs.size())
        , collision_results(modelGeom.collisionPairs.size())
        , radius()
         
    {
      collisionObjects.reserve(modelGeom.geometryObjects.size());
      BOOST_FOREACH( const GeometryObject & geom, modelGeom.geometryObjects)
        { collisionObjects.push_back
            (fcl::CollisionObject(geom.collision_geometry)); }
    }
#else
    GeometryData(const GeometryModel & modelGeom)
    : model_geom(modelGeom)
    , oMg(model_geom.ngeoms)
    {}
#endif // WITH_HPP_FCL   


    ~GeometryData() {};
#ifdef WITH_HPP_FCL
    void activateCollisionPair(const Index pairId,const bool flag=true);
    void deactivateCollisionPair(const Index pairId);

    ///
    /// \brief Compute the collision status between two collision objects of a given collision pair.
    ///
    /// \param[in] pair The collsion pair.
    ///
    /// \return Return true is the collision objects are colliding.
    ///
    CollisionResult computeCollision(const CollisionPair & pair) const;
    
    ///
    /// \brief Compute the collision result of all the collision pairs according to
    ///        the current placements of the geometires stored in GeometryData::oMg.
    ///        The results are stored in the vector GeometryData::collision_results.
    ///
    void computeAllCollisions();
    
    ///
    /// \brief Check if at least one of the collision pairs has its two collision objects in collision.
    ///
    bool isColliding() const;

    ///
    /// \brief Compute the minimal distance between collision objects of a collison pair
    ///
    /// \param[in] pair The collsion pair.
    ///
    /// \return An fcl struct containing the distance result.
    ///
    DistanceResult computeDistance(const CollisionPair & pair) const;
    
    ///
    /// \brief Compute the distance result for all collision pairs according to
    ///        the current placements of the geometries stored in GeometryData::oMg.
    ///        The results are stored in the vector GeometryData::distance_results.
    ///
    void computeAllDistances();
    
    void resetDistances();
#endif //WITH_HPP_FCL
    friend std::ostream & operator<<(std::ostream & os, const GeometryData & data_geom);
    
  }; // struct GeometryData

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/geometry.hxx"

#endif // ifndef __se3_geom_hpp__
