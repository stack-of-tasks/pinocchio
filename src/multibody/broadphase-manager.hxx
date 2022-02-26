//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_broadphase_manager_hxx__
#define __pinocchio_multibody_broadphase_manager_hxx__

#ifdef PINOCCHIO_WITH_HPP_FCL

namespace pinocchio
{

  template<typename Manager>
  void BroadPhaseManagerTpl<Manager>::update(bool compute_local_aabb)
  {
    const GeometryModel & geom_model = getGeometryModel();
    assert(geom_model.ngeoms == collision_object_inflation.size());
    
    GeometryData & geom_data = getGeometryData();
    collision_object_inflation.setZero();
    
    // The pass should be done over all the geometry objects composing the collision tree.
    for(size_t pair_id = 0; pair_id < geom_model.collisionPairs.size(); ++pair_id)
    {
      // TODO(jcarpent): enhance the performances by collecting only the collision pairs related to the selected_geometry_objects
      const CollisionPair & pair = geom_model.collisionPairs[pair_id];
      const GeomIndex geom1_id = pair.first;
      const GeomIndex geom2_id = pair.second;
      
      if(   geometry_to_collision_index[geom1_id] == std::numeric_limits<size_t>::max()
         || geometry_to_collision_index[geom2_id] == std::numeric_limits<size_t>::max())
        continue;
      
      const bool check_collision =
           geom_data.activeCollisionPairs[pair_id]
      && !(geom_model.geometryObjects[geom1_id].disableCollision || geom_model.geometryObjects[geom2_id].disableCollision);
      
      if(!check_collision) continue;
      
      const ::hpp::fcl::CollisionRequest & cr = geom_data.collisionRequests[pair_id];
      const double inflation = (cr.break_distance + cr.security_margin)*0.5;
      
      const Eigen::DenseIndex geom1_id_local = static_cast<Eigen::DenseIndex>(geometry_to_collision_index[geom1_id]);
      const Eigen::DenseIndex geom2_id_local = static_cast<Eigen::DenseIndex>(geometry_to_collision_index[geom2_id]);
      
      collision_object_inflation[geom1_id_local] = (std::max)(inflation,collision_object_inflation[geom1_id_local]);
      collision_object_inflation[geom2_id_local] = (std::max)(inflation,collision_object_inflation[geom2_id_local]);
    }
    
    for(size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      const size_t geometry_object_id = selected_geometry_objects[k];
      const GeometryObject & geom_obj = geom_model.geometryObjects[geometry_object_id];
      hpp::fcl::CollisionGeometryPtr_t new_geometry = geom_obj.geometry;
      
      CollisionObject & collision_obj = collision_objects[k];
      hpp::fcl::CollisionGeometryPtr_t geometry = collision_obj.collisionGeometry();
      
      collision_obj.setTransform(toFclTransform3f(geom_data.oMg[geometry_object_id]));
      
      if(new_geometry.get() != geometry.get())
      {
        collision_obj.setCollisionGeometry(new_geometry,compute_local_aabb);
      }
      else
      {
        collision_obj.computeAABB();
      }
      
      collision_obj.getAABB().expand(collision_object_inflation[static_cast<Eigen::DenseIndex>(k)]);
    }
    
    assert(check() && "The status of the BroadPhaseManager is not valid");
    
    manager.update(); // because the position has changed.
  }
  
  template<typename Manager>
  void BroadPhaseManagerTpl<Manager>::update(GeometryData * geom_data_ptr_new)
  {
    Base::geometry_data_ptr = geom_data_ptr_new;
    update(false);
  }
  
  template<typename Manager>
  BroadPhaseManagerTpl<Manager>::~BroadPhaseManagerTpl()
  {}
  
  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::check() const
  {
    std::vector<hpp::fcl::CollisionObject*> collision_objects_ptr = manager.getObjects();
    if(collision_objects_ptr.size() != collision_objects.size())
      return false;
    
    const GeometryModel & geom_model = getGeometryModel();
    for(size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      const size_t i = selected_geometry_objects[k];
      const hpp::fcl::CollisionObject & collision_obj = collision_objects[k];

      if(std::find(collision_objects_ptr.begin(), collision_objects_ptr.end(), &collision_obj) == collision_objects_ptr.end())
        return false;
      
      hpp::fcl::CollisionGeometryConstPtr_t geometry = collision_obj.collisionGeometry();
      const GeometryObject & geom_obj = geom_model.geometryObjects[i];
      hpp::fcl::CollisionGeometryConstPtr_t geometry_of_geom_obj = geom_obj.geometry;
      
      if(geometry.get() != geometry_of_geom_obj.get())
        return false;
    }
    
    return true;
  }
  
  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::check(CollisionCallBackBase * callback) const
  {
    return
       &callback->getGeometryModel() == &getGeometryModel()
    && &callback->getGeometryData() == &getGeometryData();
  }

  template<typename Manager>
  void BroadPhaseManagerTpl<Manager>::init()
  {
    const GeometryModel & geom_model = getGeometryModel();
    collision_objects.reserve(selected_geometry_objects.size());
    for(size_t i: selected_geometry_objects)
    {
      GeometryObject & geom_obj = const_cast<GeometryObject &>(geom_model.geometryObjects[i]);
      collision_objects.push_back(CollisionObject(geom_obj.geometry,i));

      // Feed the base broadphase manager
      manager.registerObject(&collision_objects.back());
    }
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::collide(CollisionObject & obj,
                                              CollisionCallBackBase * callback) const
  {
    manager.collide(&obj,callback);
    return callback->collision;
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::collide(CollisionCallBackBase * callback) const
  {
    manager.collide(callback);
    return callback->collision;
  }

  template<typename Manager>
  bool BroadPhaseManagerTpl<Manager>::collide(BroadPhaseManagerTpl & other_manager,
                                              CollisionCallBackBase * callback) const
  {
    manager.collide(&other_manager.manager,callback);
    return callback->collision;
  }

} // namespace pinocchio

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_multibody_broadphase_manager_hxx__
