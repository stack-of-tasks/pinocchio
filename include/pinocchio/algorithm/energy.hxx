//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_energy_hxx__
#define __pinocchio_algorithm_energy_hxx__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
   
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Scalar
  computeKineticEnergy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    data.kinetic_energy = Scalar(0);
    
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
      data.kinetic_energy += model.inertias[i].vtiv(data.v[i]);
    
    data.kinetic_energy *= .5;
    
    return data.kinetic_energy;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Scalar
  computePotentialEnergy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");;
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::Motion Motion;

    data.potential_energy = Scalar(0);
    const typename Motion::ConstLinearType & g = model.gravity.linear();
    
    typename Data::Vector3 com_global; // tmp variable
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      com_global.noalias() = data.oMi[i].translation() + data.oMi[i].rotation() * model.inertias[i].lever();
      data.potential_energy -= model.inertias[i].mass() * com_global.dot(g);
    }
    
    return data.potential_energy;
  }
}
#endif // __pinocchio_algorithm_energy_hxx__

