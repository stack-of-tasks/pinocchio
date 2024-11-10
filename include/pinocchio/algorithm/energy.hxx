//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_energy_hxx__
#define __pinocchio_algorithm_energy_hxx__

#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct KineticEnergyAlgoForwardStep
  : public fusion::JointUnaryVisitorBase<
      KineticEnergyAlgoForwardStep<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {
      const JointIndex & i = jmodel.id();
      data.kinetic_energy += model.inertias[i].vtiv(data.v[i]);
      data.kinetic_energy +=
        (jmodel.jointVelocitySelector(model.armature).array() * jdata.joint_v().array().square())
          .sum();
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  Scalar computeKineticEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    data.kinetic_energy = Scalar(0);
    typedef KineticEnergyAlgoForwardStep<Scalar, Options, JointCollectionTpl> Pass;
    for (JointIndex i = 1; i < (JointIndex)(model.njoints); ++i)
    {
      Pass::run(model.joints[i], data.joints[i], typename Pass::ArgsType(model, data));
    }
    data.kinetic_energy *= .5;

    return data.kinetic_energy;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  Scalar computePotentialEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    ;

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::Motion Motion;

    data.potential_energy = Scalar(0);
    const typename Motion::ConstLinearType & g = model.gravity.linear();

    typename Data::Vector3 com_global; // tmp variable
    for (JointIndex i = 1; i < (JointIndex)(model.njoints); ++i)
    {
      com_global.noalias() =
        data.oMi[i].translation() + data.oMi[i].rotation() * model.inertias[i].lever();
      data.potential_energy -= model.inertias[i].mass() * com_global.dot(g);
    }

    return data.potential_energy;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  Scalar computeMechanicalEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::Motion Motion;
    typedef typename Model::JointIndex JointIndex;

    data.kinetic_energy = Scalar(0);
    data.potential_energy = Scalar(0);

    typename Data::Vector3 com_global; // tmp variable
    const typename Motion::ConstLinearType & g = model.gravity.linear();
    typedef KineticEnergyAlgoForwardStep<Scalar, Options, JointCollectionTpl> Pass;
    for (JointIndex i = 1; i < (JointIndex)(model.njoints); ++i)
    {
      Pass::run(model.joints[i], data.joints[i], typename Pass::ArgsType(model, data));
      com_global.noalias() =
        data.oMi[i].translation() + data.oMi[i].rotation() * model.inertias[i].lever();
      data.potential_energy -= model.inertias[i].mass() * com_global.dot(g);
    }
    data.kinetic_energy *= .5;

    data.mechanical_energy = data.kinetic_energy + data.potential_energy;

    return data.mechanical_energy;
  }

  namespace impl
  {
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    Scalar computeKineticEnergy(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      pinocchio::impl::forwardKinematics(model, data, q.derived(), v.derived());
      return pinocchio::computeKineticEnergy(model, data);
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    Scalar computePotentialEnergy(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      pinocchio::impl::forwardKinematics(model, data, q);
      return pinocchio::computePotentialEnergy(model, data);
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    Scalar computeMechanicalEnergy(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      pinocchio::impl::forwardKinematics(model, data, q, v);
      return pinocchio::computeMechanicalEnergy(model, data);
    }
  } // namespace impl

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  Scalar computeKineticEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return impl::computeKineticEnergy(model, data, make_const_ref(q), make_const_ref(v));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  Scalar computePotentialEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    return impl::computePotentialEnergy(model, data, make_const_ref(q));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  Scalar computeMechanicalEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return impl::computeMechanicalEnergy(model, data, make_const_ref(q), make_const_ref(v));
  }

} // namespace pinocchio
#endif // __pinocchio_algorithm_energy_hxx__
