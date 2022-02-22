//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_broadphase_manager_hxx__
#define __pinocchio_multibody_broadphase_manager_hxx__

#ifdef PINOCCHIO_WITH_HPP_FCL

namespace pinocchio
{

  template<typename BroadPhaseManagerDerived>
  void BroadPhaseManagerTpl<BroadPhaseManagerDerived>::update(bool compute_local_aabb)
  {
    assert(geometry_model_ptr->ngeoms == collision_object_inflation.size());
    
    const GeometryModel & geom_model = *geometry_model_ptr;
    GeometryData & geom_data = *geometry_data_ptr;
    collision_object_inflation.setZero();
    
    for(size_t pair_id = 0; pair_id < geom_data.activeCollisionPairs.size(); ++pair_id)
    {
      const CollisionPair & pair = geom_model.collisionPairs[pair_id];
      const GeomIndex geom1_id = pair.first;
      const GeomIndex geom2_id = pair.second;
      
      const bool check_collision =
           geom_data.activeCollisionPairs[pair_id]
      && !(geom_model.geometryObjects[geom1_id].disableCollision || geom_model.geometryObjects[geom2_id].disableCollision);
      
      if(!check_collision) continue;
      
      const ::hpp::fcl::CollisionRequest & cr = geom_data.collisionRequests[pair_id];
      const double inflation = (cr.break_distance + cr.security_margin)*0.5;
      
      collision_object_inflation[static_cast<Eigen::DenseIndex>(geom1_id)]
      = (std::max)(inflation,collision_object_inflation[static_cast<Eigen::DenseIndex>(geom1_id)]);
      collision_object_inflation[static_cast<Eigen::DenseIndex>(geom2_id)]
      = (std::max)(inflation,collision_object_inflation[static_cast<Eigen::DenseIndex>(geom2_id)]);
    }
    
    for(size_t i = 0; i < geometry_model_ptr->geometryObjects.size(); ++i)
    {
      const GeometryObject & geom_obj = geometry_model_ptr->geometryObjects[i];
      hpp::fcl::CollisionGeometryPtr_t new_geometry = geom_obj.geometry;
      
      CollisionObject & collision_obj = collision_objects[i];
      hpp::fcl::CollisionGeometryPtr_t geometry = collision_obj.collisionGeometry();
      
      collision_obj.setTransform(toFclTransform3f(geom_data.oMg[i]));
      
      if(new_geometry.get() != geometry.get())
      {
        collision_obj.setCollisionGeometry(new_geometry,compute_local_aabb);
      }
      else
      {
        collision_obj.computeAABB();
      }
      
      collision_obj.getAABB().expand(collision_object_inflation[static_cast<Eigen::DenseIndex>(i)]);
    }
    
    assert(check() && "The status of the BroadPhaseManager is not valid");
    
    Base::update(); // because the position has changed.
  }
  
  template<typename BroadPhaseManagerDerived>
  void BroadPhaseManagerTpl<BroadPhaseManagerDerived>::update()
  {
    update(false);
  }
  
  template<typename BroadPhaseManagerDerived>
  void BroadPhaseManagerTpl<BroadPhaseManagerDerived>::update(GeometryData * geom_data_ptr_new)
  {
    geometry_data_ptr = geom_data_ptr_new;
    update(false);
  }
  
  template<typename BroadPhaseManagerDerived>
  BroadPhaseManagerTpl<BroadPhaseManagerDerived>::~BroadPhaseManagerTpl()
  {}
  
  template<typename BroadPhaseManagerDerived>
  bool BroadPhaseManagerTpl<BroadPhaseManagerDerived>::check() const
  {
    std::vector<hpp::fcl::CollisionObject*> collision_objects_ptr = this->getObjects();
    if(collision_objects_ptr.size() != collision_objects.size())
      return false;
    
    for(size_t i = 0; i < collision_objects.size(); ++i)
    {
      const hpp::fcl::CollisionObject & collision_obj = collision_objects[i];

      if(std::find(collision_objects_ptr.begin(), collision_objects_ptr.end(), &collision_obj) == collision_objects_ptr.end())
        return false;
      
      hpp::fcl::CollisionGeometryConstPtr_t geometry = collision_obj.collisionGeometry();
      const GeometryObject & geom_obj = geometry_model_ptr->geometryObjects[i];
      hpp::fcl::CollisionGeometryConstPtr_t geometry_of_geom_obj = geom_obj.geometry;
      
      if(geometry.get() != geometry_of_geom_obj.get())
        return false;
    }
    
    return true;
  }
  
  template<typename BroadPhaseManagerDerived>
  bool BroadPhaseManagerTpl<BroadPhaseManagerDerived>::check(CollisionCallBackBase * callback) const
  {
    return
       &callback->getGeometryModel() == geometry_model_ptr
    && &callback->getGeometryData() == geometry_data_ptr;
  }

  template<typename BroadPhaseManagerDerived>
  void BroadPhaseManagerTpl<BroadPhaseManagerDerived>::init()
  {
    collision_objects.reserve(geometry_model_ptr->geometryObjects.size());
    for(size_t i = 0; i < geometry_model_ptr->geometryObjects.size() ; ++i)
    {
      GeometryObject & geom_obj = const_cast<GeometryObject &>(geometry_model_ptr->geometryObjects[i]);
      collision_objects.push_back(CollisionObject(geom_obj.geometry,i));

      // Feed the base broadphase manager
      Base::registerObject(&collision_objects[i]);
    }
  }

} // namespace pinocchio

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_multibody_broadphase_manager_hxx__
