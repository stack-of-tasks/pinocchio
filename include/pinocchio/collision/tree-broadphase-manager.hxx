//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_collision_tree_broadphase_manager_hxx__
#define __pinocchio_collision_tree_broadphase_manager_hxx__

namespace pinocchio
{

  template<typename Manager>
  void TreeBroadPhaseManagerTpl<Manager>::update(bool compute_local_aabb)
  {
    for (auto && manager : managers)
    {
      manager.update(compute_local_aabb);
    }
  }

  template<typename Manager>
  void TreeBroadPhaseManagerTpl<Manager>::update(GeometryData * geom_data_ptr_new)
  {
    for (auto && manager : managers)
    {
      manager.update(geom_data_ptr_new);
    }
  }

  template<typename Manager>
  bool TreeBroadPhaseManagerTpl<Manager>::check() const
  {
    for (auto && manager : managers)
    {
      if (!manager.check())
        return false;
    }

    return true;
  }

  template<typename Manager>
  bool TreeBroadPhaseManagerTpl<Manager>::check(CollisionCallBackBase * callback) const
  {
    for (auto && manager : managers)
    {
      if (!manager.check(callback))
        return false;
    }

    return true;
  }

  template<typename Manager>
  void TreeBroadPhaseManagerTpl<Manager>::init(const size_t njoints)
  {
    managers.reserve(njoints);
    for (size_t joint_id = 0; joint_id < njoints; ++joint_id)
    {
      GeometryObjectFilterSelectByJoint filter(joint_id);
      managers.push_back(
        BroadPhaseManager(&getModel(), &getGeometryModel(), &getGeometryData(), filter));
    }
  }

  template<typename Manager>
  bool TreeBroadPhaseManagerTpl<Manager>::collide(CollisionCallBackBase * callback) const
  {
    const size_t num_joints = managers.size();

    callback->init();
    const bool accumulate_save_value = callback->accumulate;
    callback->accumulate = true;

    for (size_t i = 0; i < num_joints; ++i)
    {
      const BroadPhaseManager & manager_outer = managers[i];
      bool should_stop = false;
      for (size_t j = i + 1; j < num_joints; ++j)
      {
        BroadPhaseManager & manager_inner = const_cast<BroadPhaseManager &>(managers[j]);
        manager_outer.collide(manager_inner, callback);
        should_stop = callback->stop();

        if (should_stop)
          break;
      }

      if (should_stop)
        break;
    }

    callback->accumulate = accumulate_save_value; // restore initial value

    callback->done();
    return callback->collision;
  }

  template<typename Manager>
  bool TreeBroadPhaseManagerTpl<Manager>::collide(
    CollisionObject & collision_object, CollisionCallBackBase * callback) const
  {
    const size_t num_joints = managers.size();

    callback->init();
    const bool accumulate_save_value = callback->accumulate;
    callback->accumulate = true;

    for (size_t i = 0; i < num_joints; ++i)
    {
      const BroadPhaseManager & manager = managers[i];
      manager.collide(collision_object, callback);
      if (callback->stop())
        break;
    }

    callback->accumulate = accumulate_save_value; // restore initial value

    callback->done();
    return callback->collision;
  }

  template<typename Manager>
  bool TreeBroadPhaseManagerTpl<Manager>::collide(
    TreeBroadPhaseManagerTpl & other_manager, CollisionCallBackBase * callback) const
  {
    const size_t num_joints = managers.size();

    callback->init();
    const bool accumulate_save_value = callback->accumulate;
    callback->accumulate = true;

    for (size_t i = 0; i < num_joints; ++i)
    {
      const BroadPhaseManager & manager_outer = managers[i];
      bool should_stop = false;
      for (auto && manager_inner : other_manager.managers)
      {
        manager_outer.collide(manager_inner, callback);
        should_stop = callback->stop();

        if (should_stop)
          break;
      }
      if (should_stop)
        break;
    }

    callback->accumulate = accumulate_save_value; // restore initial value

    callback->done();
    return callback->collision;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_tree_broadphase_manager_hxx__
