//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_collision_broadphase_manager_hxx__
#define __pinocchio_collision_broadphase_manager_hxx__

namespace pinocchio
{

  template<typename Manager>
  void BroadPhaseManagerTpl<Manager>::update(bool compute_local_aabb)
  {
    const GeometryModel & geom_model = getGeometryModel();
    assert(selected_geometry_objects.size() == size_t(collision_object_inflation.size()));

    GeometryData & geom_data = getGeometryData();

    // Pass 1: check the new active geometries and list the new deactive geometries
    std::vector<size_t> new_active, new_inactive;
    for (size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      const size_t geometry_object_id = selected_geometry_objects[k];
      const GeometryObject & geom_object = geom_model.geometryObjects[geometry_object_id];

      if (geom_object.geometry->aabb_local.volume() <= 0.) // degenerated geometry, we should not
                                                           // consider it as an active object.
      {
        if (collision_object_is_active[k])
        {
          collision_object_is_active[k] = false;
          new_inactive.push_back(k);
        }

        continue; // don't go further and checks the next objects
      }

      if (collision_object_is_active[k] != !geom_object.disableCollision) // change state
      {
        collision_object_is_active[k] = !geom_object.disableCollision;
        if (geom_object.disableCollision)
          new_inactive.push_back(k);
        else
          new_active.push_back(k);
      }
    }

    // The pass should be done over all the geometry objects composing the collision tree.
    collision_object_inflation.setZero();
    for (size_t pair_id = 0; pair_id < geom_model.collisionPairs.size(); ++pair_id)
    {
      // TODO(jcarpent): enhance the performances by collecting only the collision pairs related to
      // the selected_geometry_objects
      const CollisionPair & pair = geom_model.collisionPairs[pair_id];
      const GeomIndex geom1_id = pair.first;
      const GeomIndex geom2_id = pair.second;

      const bool geom1_is_selected =
        geometry_to_collision_index[geom1_id] != (std::numeric_limits<size_t>::max)();
      const bool geom2_is_selected =
        geometry_to_collision_index[geom2_id] != (std::numeric_limits<size_t>::max)();
      if (!(geom1_is_selected || geom2_is_selected))
        continue;

      const bool check_collision = geom_data.activeCollisionPairs[pair_id]
                                   && !(
                                     geom_model.geometryObjects[geom1_id].disableCollision
                                     || geom_model.geometryObjects[geom2_id].disableCollision);

      if (!check_collision)
        continue;

      const ::hpp::fcl::CollisionRequest & cr = geom_data.collisionRequests[pair_id];
      const double inflation = (cr.break_distance + cr.security_margin) * 0.5;

      if (geom1_is_selected)
      {
        const Eigen::DenseIndex geom1_id_local =
          static_cast<Eigen::DenseIndex>(geometry_to_collision_index[geom1_id]);
        collision_object_inflation[geom1_id_local] =
          (std::max)(inflation, collision_object_inflation[geom1_id_local]);
      }

      if (geom2_is_selected)
      {
        const Eigen::DenseIndex geom2_id_local =
          static_cast<Eigen::DenseIndex>(geometry_to_collision_index[geom2_id]);
        collision_object_inflation[geom2_id_local] =
          (std::max)(inflation, collision_object_inflation[geom2_id_local]);
      }
    }

    for (size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      if (!collision_object_is_active[k])
        continue;

      const size_t geometry_object_id = selected_geometry_objects[k];

      const GeometryObject & geom_obj = geom_model.geometryObjects[geometry_object_id];
      hpp::fcl::CollisionGeometryPtr_t new_geometry = geom_obj.geometry;

      CollisionObject & collision_obj = collision_objects[k];
      hpp::fcl::CollisionGeometryPtr_t geometry = collision_obj.collisionGeometry();

      collision_obj.setTransform(toFclTransform3f(geom_data.oMg[geometry_object_id]));

      if (new_geometry.get() != geometry.get())
        collision_obj.setCollisionGeometry(new_geometry, compute_local_aabb);

      collision_obj.computeAABB();
      collision_obj.getAABB().expand(collision_object_inflation[static_cast<Eigen::DenseIndex>(k)]);
    }

    // Remove deactivated collision objects
    for (size_t k : new_inactive)
      manager.unregisterObject(&collision_objects[k]);

    // Add deactivated collision objects
    for (size_t k : new_active)
      manager.registerObject(&collision_objects[k]);

    manager.update(); // because the position has changed.
    assert(check() && "The status of the BroadPhaseManager is not valid");
  }

  template<typename Manager>
  void BroadPhaseManagerTpl<Manager>::update(GeometryData * geom_data_ptr_new)
  {
    Base::geometry_data_ptr = geom_data_ptr_new;
    update(false);
  }

  template<typename Manager>
  BroadPhaseManagerTpl<Manager>::~BroadPhaseManagerTpl()
  {
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::check() const
  {
    std::vector<hpp::fcl::CollisionObject *> collision_objects_ptr = manager.getObjects();
    if (collision_objects_ptr.size() > collision_objects.size())
      return false;

    size_t count_active_objects = 0;
    for (auto active : collision_object_is_active)
      count_active_objects += active;

    if (count_active_objects != collision_objects_ptr.size())
      return false;

    const GeometryModel & geom_model = getGeometryModel();
    for (size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      const size_t geometry_id = selected_geometry_objects[k];

      const hpp::fcl::CollisionObject & collision_obj = collision_objects[k];
      hpp::fcl::CollisionGeometryConstPtr_t geometry = collision_obj.collisionGeometry();

      if (collision_object_is_active[k]) // The collision object is active
      {
        if (
          std::find(collision_objects_ptr.begin(), collision_objects_ptr.end(), &collision_obj)
          == collision_objects_ptr.end())
          return false;

        if (
          geometry.get()->aabb_local.volume()
          == -(std::numeric_limits<hpp::fcl::FCL_REAL>::infinity)())
          return false;

        const GeometryObject & geom_obj = geom_model.geometryObjects[geometry_id];
        hpp::fcl::CollisionGeometryConstPtr_t geometry_of_geom_obj = geom_obj.geometry;

        if (geometry.get() != geometry_of_geom_obj.get())
          return false;
      }
      else
      {
        if (
          std::find(collision_objects_ptr.begin(), collision_objects_ptr.end(), &collision_obj)
          != collision_objects_ptr.end())
          return false;
      }
    }

    return true;
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::check(CollisionCallBackBase * callback) const
  {
    return &callback->getGeometryModel() == &getGeometryModel()
           && &callback->getGeometryData() == &getGeometryData();
  }

  template<typename Manager>
  void BroadPhaseManagerTpl<Manager>::init()
  {
    const GeometryModel & geom_model = getGeometryModel();
    collision_objects.reserve(selected_geometry_objects.size());
    for (size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      const size_t geometry_id = selected_geometry_objects[k];
      GeometryObject & geom_obj =
        const_cast<GeometryObject &>(geom_model.geometryObjects[geometry_id]);
      collision_objects.push_back(CollisionObject(geom_obj.geometry, geometry_id));

      // Feed the base broadphase manager
      if (collision_object_is_active[k])
        manager.registerObject(&collision_objects.back());
    }
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::collide(
    CollisionObject & obj, CollisionCallBackBase * callback) const
  {
    manager.collide(&obj, callback);
    return callback->collision;
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::collide(CollisionCallBackBase * callback) const
  {
    manager.collide(callback);
    return callback->collision;
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::collide(
    BroadPhaseManagerTpl & other_manager, CollisionCallBackBase * callback) const
  {
    manager.collide(&other_manager.manager, callback);
    return callback->collision;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_broadphase_manager_hxx__
