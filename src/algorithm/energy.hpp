//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_energy_hpp__
#define __se3_energy_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3 {
  
  ///
  /// \brief Computes the kinetic energy of the system.
  ///        The result is accessible through data.kinetic_energy.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The kinetic energy of the system in [J].
  ///
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline typename JointCollection::Scalar
  kineticEnergy(const ModelTpl<JointCollection> & model,
                DataTpl<JointCollection> & data,
                const Eigen::MatrixBase<ConfigVectorType> & q,
                const Eigen::MatrixBase<TangentVectorType> & v,
                const bool update_kinematics = true);
  
  ///
  /// \brief Computes the potential energy of the system, i.e. the potential energy linked to the gravity field.
  ///        The result is accessible through data.potential_energy.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The potential energy of the system expressed in [J].
  ///
  template<typename JointCollection, typename ConfigVectorType>
  inline typename JointCollection::Scalar
  potentialEnergy(const ModelTpl<JointCollection> & model,
                  DataTpl<JointCollection> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const bool update_kinematics = true);
}

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
namespace se3
{
   
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline typename JointCollection::Scalar
  kineticEnergy(const ModelTpl<JointCollection> & model,
                DataTpl<JointCollection> & data,
                const Eigen::MatrixBase<ConfigVectorType> & q,
                const Eigen::MatrixBase<TangentVectorType> & v,
                const bool update_kinematics)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename JointCollection::Scalar Scalar;
    
    data.kinetic_energy = Scalar(0);
    
    if (update_kinematics)
      forwardKinematics(model,data,q.derived(),v.derived());
    
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
      data.kinetic_energy += model.inertias[i].vtiv(data.v[i]);
    
    data.kinetic_energy *= .5;
    
    return data.kinetic_energy;
  }
  
  template<typename JointCollection, typename ConfigVectorType>
  inline typename JointCollection::Scalar
  potentialEnergy(const ModelTpl<JointCollection> & model,
                  DataTpl<JointCollection> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const bool update_kinematics)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::Motion Motion;
    typedef typename JointCollection::Scalar Scalar;
    
    data.potential_energy = Scalar(0);
    const typename Motion::ConstLinearType & g = model.gravity.linear();
    
    if (update_kinematics)
      forwardKinematics(model,data,q);
    
    typename Data::Vector3 com_global; // tmp variable
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      com_global.noalias() = data.oMi[i].translation() + data.oMi[i].rotation() * model.inertias[i].lever();
      data.potential_energy -= model.inertias[i].mass() * com_global.dot(g);
    }
    
    return data.potential_energy;
  }
}
#endif // __se3_energy_hpp__
