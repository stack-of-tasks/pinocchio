//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_collision_collision_hxx__
#define __pinocchio_collision_collision_hxx__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/collision/distance.hpp"

namespace pinocchio
{

  inline bool computeCollision(
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const PairIndex pair_id,
    fcl::CollisionRequest & collision_request)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      geom_model.collisionPairs.size() == geom_data.collisionResults.size());
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair_id < geom_model.collisionPairs.size());

    const CollisionPair & pair = geom_model.collisionPairs[pair_id];

    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair.first < geom_model.ngeoms);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair.second < geom_model.ngeoms);

    collision_request.distance_upper_bound =
      collision_request.security_margin + 1e-6; // TODO: change the margin

    fcl::CollisionResult & collision_result = geom_data.collisionResults[pair_id];
    collision_result.clear();

    fcl::Transform3f oM1(toFclTransform3f(geom_data.oMg[pair.first])),
      oM2(toFclTransform3f(geom_data.oMg[pair.second]));

    try
    {
      GeometryData::ComputeCollision & do_computations = geom_data.collision_functors[pair_id];
      do_computations(oM1, oM2, collision_request, collision_result);
    }
    catch (std::invalid_argument & e)
    {
      PINOCCHIO_THROW_PRETTY(
        std::invalid_argument, "Problem when trying to check the collision of collision pair #"
                                 << pair_id << " (" << pair.first << "," << pair.second << ")"
                                 << std::endl
                                 << "hpp-fcl original error:\n"
                                 << e.what() << std::endl);
    }
    catch (std::logic_error & e)
    {
      PINOCCHIO_THROW_PRETTY(
        std::logic_error, "Problem when trying to check the collision of collision pair #"
                            << pair_id << " (" << pair.first << "," << pair.second << ")"
                            << std::endl
                            << "hpp-fcl original error:\n"
                            << e.what() << std::endl);
    }

    return collision_result.isCollision();
  }

  inline bool computeCollision(
    const GeometryModel & geom_model, GeometryData & geom_data, const PairIndex pair_id)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair_id < geom_model.collisionPairs.size());
    fcl::CollisionRequest & collision_request = geom_data.collisionRequests[pair_id];

    return computeCollision(geom_model, geom_data, pair_id, collision_request);
  }

  inline bool computeCollisions(
    const GeometryModel & geom_model, GeometryData & geom_data, const bool stopAtFirstCollision)
  {
    bool isColliding = false;

    for (std::size_t cp_index = 0; cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
      const CollisionPair & cp = geom_model.collisionPairs[cp_index];

      if (
        geom_data.activeCollisionPairs[cp_index]
        && !(
          geom_model.geometryObjects[cp.first].disableCollision
          || geom_model.geometryObjects[cp.second].disableCollision))
      {
        bool res = computeCollision(geom_model, geom_data, cp_index);
        if (!isColliding && res)
        {
          isColliding = true;
          geom_data.collisionPairIndex = cp_index; // first pair to be in collision
          if (stopAtFirstCollision)
            return true;
        }
      }
    }

    return isColliding;
  }

  /* --- COLLISIONS ----------------------------------------------------------------- */
  /* --- COLLISIONS ----------------------------------------------------------------- */
  /* --- COLLISIONS ----------------------------------------------------------------- */

  // WARNING, if stopAtFirstcollision = true, then the collisions vector will not be fulfilled.
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  inline bool computeCollisions(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const bool stopAtFirstCollision)
  {
    updateGeometryPlacements(model, data, geom_model, geom_data, q);
    return computeCollisions(geom_model, geom_data, stopAtFirstCollision);
  }

  /* --- RADIUS -------------------------------------------------------------------- */
  /* --- RADIUS -------------------------------------------------------------------- */
  /* --- RADIUS -------------------------------------------------------------------- */

  /// Given p1..3 being either "min" or "max", return one of the corners of the
  /// AABB cub of the FCL object.
#define PINOCCHIO_GEOM_AABB(FCL, p1, p2, p3)                                                       \
  SE3::Vector3(FCL->aabb_local.p1##_[0], FCL->aabb_local.p2##_[1], FCL->aabb_local.p3##_[2])

  /// For all bodies of the model, compute the point of the geometry model
  /// that is the further from the center of the joint. This quantity is used
  /// in some continuous collision test.
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void computeBodyRadius(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const GeometryModel & geom_model,
    GeometryData & geom_data)
  {
    geom_data.radius.resize(model.joints.size(), 0);
    BOOST_FOREACH (const GeometryObject & geom_object, geom_model.geometryObjects)
    {
      const GeometryObject::CollisionGeometryPtr & geometry = geom_object.geometry;

      // Force computation of the Local AABB
      // TODO: change for a more elegant solution
      const_cast<hpp::fcl::CollisionGeometry &>(*geometry).computeLocalAABB();

      const GeometryModel::SE3 & jMb = geom_object.placement; // placement in joint.
      const Model::JointIndex i = geom_object.parentJoint;
      assert(i < geom_data.radius.size());

      Scalar radius = geom_data.radius[i] * geom_data.radius[i];

      // The radius is simply the one of the 8 corners of the AABB cube, expressed
      // in the joint frame, whose norm is the highest.
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, min, min, min)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, min, min, max)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, min, max, min)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, min, max, max)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, max, min, min)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, max, min, max)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, max, max, min)).squaredNorm(), radius);
      radius =
        std::max(jMb.act(PINOCCHIO_GEOM_AABB(geometry, max, max, max)).squaredNorm(), radius);

      // Don't forget to sqroot the squared norm before storing it.
      geom_data.radius[i] = sqrt(radius);
    }
  }

#undef PINOCCHIO_GEOM_AABB

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_collision_hxx__
