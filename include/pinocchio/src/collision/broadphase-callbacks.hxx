//
// Copyright (c) 2015-2024 CNRS INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/collision/broadphase-callbacks.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/collision/broadphase-callbacks.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  /// @brief Interface for Pinocchio collision callback functors
  struct CollisionCallBackBase : coal::CollisionCallBackBase
  {
    CollisionCallBackBase(const GeometryModel & geometry_model, GeometryData & geometry_data)
    : geometry_model_ptr(&geometry_model)
    , geometry_data_ptr(&geometry_data)
    , collision(false)
    , accumulate(false)
    {
    }

    const GeometryModel & getGeometryModel() const
    {
      return *geometry_model_ptr;
    }
    const GeometryData & getGeometryData() const
    {
      return *geometry_data_ptr;
    }
    GeometryData & getGeometryData()
    {
      return *geometry_data_ptr;
    }

    /// \brief If true, the stopping criteria related to the collision callback has been met and one
    /// can stop.
    virtual bool stop() const = 0;

    /// \brief Callback method called after the termination of a collisition detection algorithm.
    ///        The default implementation does nothing.
    virtual void done() {};

    virtual ~CollisionCallBackBase() = default;

  protected:
    /// @brief Geometry model associated to the callback
    const GeometryModel * geometry_model_ptr;

    /// @brief Geometry data associated to the callback
    GeometryData * geometry_data_ptr;

  public:
    /// @brief Whether there is a collision or not
    bool collision;

    /// @brief Whether the callback is used in an accumulate mode where several collide methods are
    /// called successively.
    bool accumulate;
  };

  struct CollisionCallBackDefault : CollisionCallBackBase
  {
    CollisionCallBackDefault(
      const GeometryModel & geometry_model,
      GeometryData & geometry_data,
      bool stopAtFirstCollision = false)
    : CollisionCallBackBase(geometry_model, geometry_data)
    , stopAtFirstCollision(stopAtFirstCollision)
    , count(0)
    //  , visited(Eigen::MatrixXd::Zero(geometry_model.ngeoms,geometry_model.ngeoms))
    {
    }

    void init()
    {
      if (accumulate) // skip reseting of the parameters
        return;

      count = 0;
      collision = false;
      collisionPairIndex = std::numeric_limits<PairIndex>::max();
      //    visited.setZero();
    }

    bool collide(coal::CollisionObject * o1, coal::CollisionObject * o2)
    {

      assert(!stop() && "must never happened");

      CollisionObject & co1 = reinterpret_cast<CollisionObject &>(*o1);
      CollisionObject & co2 = reinterpret_cast<CollisionObject &>(*o2);

      const Eigen::Index go1_index = (Eigen::Index)co1.geometryObjectIndex;
      const Eigen::Index go2_index = (Eigen::Index)co2.geometryObjectIndex;

      const GeometryModel & geometry_model = *geometry_model_ptr;

      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        go1_index < (Eigen::Index)geometry_model.ngeoms && go1_index >= 0);
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        go2_index < (Eigen::Index)geometry_model.ngeoms && go2_index >= 0);

      const int pair_index = geometry_model.collisionPairMapping(go1_index, go2_index);
      if (pair_index == -1)
        return false;

      const GeometryData & geometry_data = *geometry_data_ptr;
      const CollisionPair & cp = geometry_model.collisionPairs[(PairIndex)pair_index];
      const bool do_collision_check =
        geometry_data.activeCollisionPairs[(PairIndex)pair_index]
        && !(
          geometry_model.geometryObjects[cp.first].disableCollision
          || geometry_model.geometryObjects[cp.second].disableCollision);
      if (!do_collision_check)
        return false;

      count++;

      coal::CollisionRequest collision_request(
        geometry_data_ptr->collisionRequests[size_t(pair_index)]);
      // collision_request.gjk_variant = coal::GJKVariant::PolyakAcceleration;
      //    collision_request.gjk_initial_guess = coal::GJKInitialGuess::BoundingVolumeGuess;

      if (
        co1.collisionGeometry().get()
          != geometry_model.geometryObjects[size_t(go1_index)].geometry.get()
        || co2.collisionGeometry().get()
             != geometry_model.geometryObjects[size_t(go2_index)].geometry.get())
        PINOCCHIO_THROW_PRETTY(
          std::logic_error, "go1: " << go1_index << " or go2: " << go2_index
                                    << " have not been updated and have missmatching pointers.");
      //    if(!(co1.collisionGeometry()->aabb_local.volume() < 0 ||
      //    co2.collisionGeometry()->aabb_local.volume() <0)) { // TODO(jcarpent): check potential
      //    bug
      //      collision_request.gjk_initial_guess = coal::GJKInitialGuess::BoundingVolumeGuess;
      //    }

      bool res;
      try
      {
        res = computeCollision(
          *geometry_model_ptr, *geometry_data_ptr, (PairIndex)pair_index, collision_request);
      }
      catch (std::logic_error & e)
      {
        PINOCCHIO_THROW_PRETTY(
          std::logic_error,
          "Geometries with index go1: " << go1_index << " or go2: " << go2_index
                                        << " have produced an internal error within coal.\n what:\n"
                                        << e.what());
      }

      if (res && !collision)
      {
        collision = true;
        collisionPairIndex = (PairIndex)pair_index;
      }

      if (!stopAtFirstCollision)
        return false;
      else
        return res;
    }

    bool stop() const final
    {
      if (stopAtFirstCollision && collision)
        return true;

      return false;
    }

    void done() final
    {
      if (collision)
        geometry_data_ptr->collisionPairIndex = collisionPairIndex;
    }

    /// @brief Whether to stop or not when localizing a first collision
    bool stopAtFirstCollision;

    /// @brief The collision index of the first pair in collision
    PairIndex collisionPairIndex;

    /// @brief Number of visits of the collide method
    size_t count;

    //  Eigen::MatrixXd visited;
  };

  /// @brief This callback appends collision pairs when the broad phase finds them in collision.
  /// A narrow phase pass must then be run on all appended collision pairs to complete the full
  /// collision detection check.
  struct CollisionCallBackCollect : CollisionCallBackBase
  {
    /// @brief Default constructor.
    /// This constructor allocates pair_indexes depending on geometry_model and geometry_data.
    CollisionCallBackCollect(const GeometryModel & geometry_model, GeometryData & geometry_data)
    : CollisionCallBackBase(geometry_model, geometry_data)
    {
      this->pair_indexes.reserve(geometry_model.collisionPairs.size());
    }

    /// @brief Constructor based on a given max_num_pairs.
    CollisionCallBackCollect(
      const GeometryModel & geometry_model, GeometryData & geometry_data, const int max_num_pairs)
    : CollisionCallBackBase(geometry_model, geometry_data)
    {
      this->pair_indexes.reserve(size_t(max_num_pairs));
    }

    /// @brief This method is called at the beginning of any broad phase call.
    void init()
    {
      this->pair_indexes.clear();
    }

    /// @brief This method is called when two leafs of the broad phase are found to be in collision
    /// (i.e. when two AABBs of a geometry_model's geometries are colliding).
    bool collide(coal::CollisionObject * o1, coal::CollisionObject * o2)
    {
      assert(!stop() && "must never happened");

      CollisionObject & co1 = reinterpret_cast<CollisionObject &>(*o1);
      CollisionObject & co2 = reinterpret_cast<CollisionObject &>(*o2);

      const Eigen::Index go1_index = (Eigen::Index)co1.geometryObjectIndex;
      const Eigen::Index go2_index = (Eigen::Index)co2.geometryObjectIndex;

      const GeometryModel & geometry_model = this->getGeometryModel();

      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        go1_index < (Eigen::Index)geometry_model.ngeoms && go1_index >= 0);
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        go2_index < (Eigen::Index)geometry_model.ngeoms && go2_index >= 0);

      const int pair_index_ = geometry_model.collisionPairMapping(go1_index, go2_index);
      if (pair_index_ == -1)
        return false;

      const PairIndex pair_index = PairIndex(pair_index_);
      const GeometryData & geometry_data = this->getGeometryData();
      const CollisionPair & cp = geometry_model.collisionPairs[pair_index];
      const bool do_collision_check =
        geometry_data.activeCollisionPairs[pair_index]
        && !(
          geometry_model.geometryObjects[cp.first].disableCollision
          || geometry_model.geometryObjects[cp.second].disableCollision);

      if (do_collision_check)
      {
        this->pair_indexes.push_back(pair_index);
      }

      return false;
    }

    bool stop() const final
    {
      return false;
    }

    /// @brief Vector of pairs that where found in collision by the broad phase.
    /// These pairs must then be sent to the narrow phase for a complete collision detection check.
    std::vector<PairIndex> pair_indexes;
  };

} // namespace pinocchio
