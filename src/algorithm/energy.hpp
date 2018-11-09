//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_energy_hpp__
#define __pinocchio_energy_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio {
  
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
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline Scalar
  kineticEnergy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                DataTpl<Scalar,Options,JointCollectionTpl> & data,
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
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline Scalar
  potentialEnergy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const bool update_kinematics = true);
}

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
namespace pinocchio
{
   
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline Scalar
  kineticEnergy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                const Eigen::MatrixBase<ConfigVectorType> & q,
                const Eigen::MatrixBase<TangentVectorType> & v,
                const bool update_kinematics)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    data.kinetic_energy = Scalar(0);
    
    if (update_kinematics)
      forwardKinematics(model,data,q.derived(),v.derived());
    
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
      data.kinetic_energy += model.inertias[i].vtiv(data.v[i]);
    
    data.kinetic_energy *= .5;
    
    return data.kinetic_energy;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline Scalar
  potentialEnergy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const bool update_kinematics)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::Motion Motion;

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
#endif // __pinocchio_energy_hpp__
