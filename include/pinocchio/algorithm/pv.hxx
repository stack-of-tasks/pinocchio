//
// Copyright (c) 2023-2024 INRIA
// Copyright (c) 2023 KU Leuven
//

#ifndef __pinocchio_algorithm_pv_hxx__
#define __pinocchio_algorithm_pv_hxx__

#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/utils/check.hpp"
#include "pinocchio/algorithm/aba.hpp"

/// @cond DEV

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class Allocator>
  inline void initPvSolver(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, Allocator> & contact_models)
  {

    // Allocate memory for the backward propagation of LA, KA and lA
    typedef typename Model::JointIndex JointIndex;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    std::fill(data.constraints_supported_dim.begin(), data.constraints_supported_dim.end(), 0);
    // Getting the constrained links
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      switch (contact_model.reference_frame)
      {
      case LOCAL:
        if (contact_model.type == CONTACT_6D)
          data.constraints_supported_dim[joint_id] += 6;
        else if (contact_model.type == CONTACT_3D)
          data.constraints_supported_dim[joint_id] += 3;
        break;
      case WORLD:
        assert(false && "WORLD not implemented");
        break;
      case LOCAL_WORLD_ALIGNED:
        assert(false && "LOCAL_WORLD_ALIGNED not implemented");
        break;
      default:
        assert(false && "Must never happen");
        break;
      }
    }
    // Running backprop to get the count of constraints
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      const JointIndex & parent = model.parents[i];
      data.par_cons_ind[i] = data.constraints_supported_dim[parent];
      data.constraints_supported_dim[parent] += data.constraints_supported_dim[i];
    }

    // Allocating memory for LA, KA and lA
    for (JointIndex i = 0; i < (JointIndex)model.njoints; ++i)
    {
      data.lA[i] = Data::VectorXs::Zero(data.constraints_supported_dim[i]);
      data.lambdaA[i] = Data::VectorXs::Zero(data.constraints_supported_dim[i]);

      data.LA[i] =
        Data::MatrixXs::Zero(data.constraints_supported_dim[i], data.constraints_supported_dim[i]);
      data.KA[i] = Data::MatrixXs::Zero(6, data.constraints_supported_dim[i]);
      data.KAS[i] = Data::MatrixXs::Zero(model.joints[i].nv(), data.constraints_supported_dim[i]);
    }

    // For Local, append the constraint matrices in K
    std::vector<int> condim_counter(static_cast<size_t>(model.njoints), 0);
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      const typename RigidConstraintData::SE3 & oMc = contact_model.joint1_placement;
      if (contact_model.type == CONTACT_6D)
      {
        data.KA[joint_id].middleCols(condim_counter[joint_id], 6) =
          oMc.toActionMatrixInverse().transpose();
        condim_counter[joint_id] += 6;
      }
      else if (contact_model.type == CONTACT_3D)
      {
        data.KA[joint_id].middleCols(condim_counter[joint_id], 3) =
          oMc.toActionMatrixInverse().transpose().leftCols(3);
        condim_counter[joint_id] += 3;
      }
    }

    data.lambda_c_prox.resize(data.constraints_supported_dim[0]);
    data.lambda_c.setZero();
    data.lambda_c_prox.setZero();
    data.osim_llt = Eigen::LLT<Data::MatrixXs>(data.constraints_supported_dim[0]);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  struct PvForwardStep1
  : public fusion::JointUnaryVisitorBase<
      PvForwardStep1<Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::
      vector<const Model &, Data &, const ConfigVectorType &, const TangentVectorType &>
        ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      jmodel.calc(jdata.derived(), q.derived(), v.derived());

      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();

      data.v[i] = jdata.v();
      if (parent > 0)
      {
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      data.a_gf[i].linear().noalias() =
        data.liMi[i].rotation().transpose() * data.a_gf[parent].linear();

      data.a_bias[i] = jdata.c() + (data.v[i] ^ jdata.v());

      data.Yaba[i] = model.inertias[i].matrix();
      data.h[i] = model.inertias[i] * data.v[i];
      data.f[i] = data.v[i].cross(data.h[i]);
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct cAbaBackwardStep
  : public fusion::JointUnaryVisitorBase<cAbaBackwardStep<Scalar, Options, JointCollectionTpl>>
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

      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;

      Force bias_and_force = Force::Zero();

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.Yaba[i];

      bias_and_force.toVector() -= data.Yaba[i] * data.a_bias[i].toVector();

      jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose() * data.f[i];
      jmodel.calc_aba(
        jdata.derived(), jmodel.jointVelocitySelector(model.armature), Ia, parent > 0);

      Force & pa = data.f[i];

      if (parent > 0)
      {
        pa.toVector().noalias() +=
          Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.Yaba[parent] += impl::internal::SE3actOn<Scalar>::run(data.liMi[i], Ia);
        data.f[parent] += data.liMi[i].act(pa);
      }
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct PvRegBackwardStep
  : public fusion::JointUnaryVisitorBase<PvRegBackwardStep<Scalar, Options, JointCollectionTpl>>
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

      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;

      Force bias_and_force = Force::Zero();

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.Yaba[i];

      bias_and_force.toVector() -= data.Yaba[i] * data.a_bias[i].toVector();

      jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose() * data.f[i];
      jmodel.calc_aba(
        jdata.derived(), jmodel.jointVelocitySelector(model.armature), Ia, parent > 0);

      Force & pa = data.f[i];

      if (parent > 0)
      {
        pa.toVector() +=
          Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.Yaba[parent] += impl::internal::SE3actOn<Scalar>::run(data.liMi[i], Ia);
        data.f[parent] += data.liMi[i].act(pa);
      }

      if (data.constraints_supported_dim[i] > 0)
      {
        data.KAS[i].noalias() = jdata.S().transpose() * data.KA[i];
      }
      for (int ind = 0; ind < data.constraints_supported_dim[i]; ind++)
      {
        Force za = Force(data.KA[i].col(ind));
        za.toVector().noalias() -= (jdata.UDinv() * (data.KAS[i].col(ind)));
        data.KA[parent].col(data.par_cons_ind[i] + ind).noalias() = data.liMi[i].act(za).toVector();
      }

      // Propagate LA backwards, we only care about tril because symmetric
      for (int ind = 0; ind < data.constraints_supported_dim[i]; ind++)
      {
        // Abusing previously unused data.tau for a role unrelated to its name below
        jmodel.jointVelocitySelector(data.tau).noalias() = jdata.Dinv() * data.KAS[i].col(ind);
        for (int ind2 = ind; ind2 < data.constraints_supported_dim[i]; ind2++)
        {

          data.LA[parent](data.par_cons_ind[i] + ind2, data.par_cons_ind[i] + ind) =
            data.LA[i](ind2, ind)
            + (data.KAS[i].col(ind2).dot(jmodel.jointVelocitySelector(data.tau)));
        }
      }

      // Propagate lA backwards
      if (data.constraints_supported_dim[i] > 0)
      {
        // Abusing previously unused data.tau variable for a role unrelated to its name below
        jmodel.jointVelocitySelector(data.tau).noalias() =
          (jdata.Dinv()
           * (jdata.S().transpose() * bias_and_force + jmodel.jointVelocitySelector(data.u)))
            .eval();
        const Motion a_bf = jdata.S() * jmodel.jointVelocitySelector(data.tau);
        const Motion a_bf_motion = a_bf + data.a_bias[i];
        for (int ind = 0; ind < data.constraints_supported_dim[i]; ind++)
        {
          data.lA[parent](data.par_cons_ind[i] + ind) =
            data.lA[i](ind) + (data.KA[i].col(ind).dot(a_bf_motion.toVector()));
        }
      }
    }
  };

  // A reduced backward sweep that only propagates the affine terms
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct PvBackwardStepReduced
  : public fusion::JointUnaryVisitorBase<PvBackwardStepReduced<Scalar, Options, JointCollectionTpl>>
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

      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      // Abusing otherwise unused data.tau below.
      jmodel.jointVelocitySelector(data.tau).noalias() = jdata.S().transpose() * data.f[i];
      jmodel.jointVelocitySelector(data.u) -= jmodel.jointVelocitySelector(data.tau);
      data.f[i].toVector().noalias() -= jdata.UDinv() * jmodel.jointVelocitySelector(data.tau);

      if (parent > 0)
      {
        data.f[parent] += data.liMi[i].act(data.f[i]);
      }
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct cAbaForwardStep2
  : public fusion::JointUnaryVisitorBase<cAbaForwardStep2<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      data.a[i] = data.liMi[i].actInv(data.a[parent]) + data.a_bias[i];
      jmodel.jointVelocitySelector(data.ddq).noalias() =
        jdata.Dinv() * (jmodel.jointVelocitySelector(data.u))
        - jdata.UDinv().transpose() * data.a[i].toVector();

      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct PvRegForwardStep2
  : public fusion::JointUnaryVisitorBase<PvRegForwardStep2<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      data.a[i] = data.liMi[i].actInv(data.a[parent]) + data.a_bias[i];
      jmodel.jointVelocitySelector(data.ddq).noalias() =
        jdata.Dinv() * (jmodel.jointVelocitySelector(data.u))
        - jdata.UDinv().transpose() * data.a[i].toVector();

      data.lambdaA[i].noalias() =
        data.lambdaA[parent].segment(data.par_cons_ind[i], data.lambdaA[i].size());
      for (int j = 0; j < data.constraints_supported_dim[i]; j++)
      {
        jmodel.jointVelocitySelector(data.ddq).noalias() -=
          data.lambdaA[i][j] * jdata.Dinv() * (data.KAS[i].col(j));
      }
      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ContactModelAllocator,
    class ContactDataAllocator>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & pv(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ContactModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ContactDataAllocator> & contact_datas,
    ProximalSettingsTpl<Scalar> & settings)
  {

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

    bool baumgarte_position = false;
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      if (!check_expression_if_real<Scalar, false>(
            contact_models[i].corrector.Kp.isZero(Scalar(0))))
        baumgarte_position = true;
    }

    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    data.a[0] = data.a_gf[0];
    data.f[0].setZero();
    data.u = tau;

    // Set the lA and LA at the contact points to zero.
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      data.lA[joint_id].setZero();
      data.LA[joint_id].setZero();
    }

    typedef PvForwardStep1<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i],
        typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
      if (baumgarte_position)
      {
        const JointIndex & parent = model.parents[i];
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];
      }
    }

    std::vector<int> condim_counter(static_cast<size_t>(model.njoints), 0);
    // Update lAs
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      typename RigidConstraintData::Motion & vc1 = contact_datas[i].contact1_velocity;
      typename RigidConstraintData::Motion & vc2 = contact_datas[i].contact2_velocity;
      const JointIndex & joint_id = contact_model.joint1_id;
      int con_dim = contact_model.size();

      const typename RigidConstraintModel::BaumgarteCorrectorParameters & corrector =
        contact_model.corrector;
      typename RigidConstraintData::Motion & contact_acc_err =
        contact_datas[i].contact_acceleration_error;
      typename RigidConstraintData::Motion & contact_vel_err =
        contact_datas[i].contact_velocity_error;

      const JointIndex & joint2_id = contact_model.joint2_id;
      if (joint2_id > 0)
        assert(false), "Internal loops are not yet permitted in PV";
      else
        vc2.setZero();

      contact_acc_err.setZero();
      if (!check_expression_if_real<Scalar, false>(corrector.Kd.isZero(Scalar(0))))
      {

        // TODO: modify for closed loops by subtracting vc2_in_frame1
        if (contact_model.type == CONTACT_6D)
        {
          contact_vel_err = vc1;
          contact_acc_err.toVector().noalias() -=
            corrector.Kd.asDiagonal() * contact_vel_err.toVector();
        }
        else
        {
          contact_vel_err = vc1;
          contact_vel_err.angular().setZero();
          contact_acc_err.linear().noalias() -=
            corrector.Kd.asDiagonal() * contact_vel_err.linear();
        }
      }

      if (!check_expression_if_real<Scalar, false>(corrector.Kp.isZero(Scalar(0))))
      {
        RigidConstraintData & contact_data = contact_datas[i];
        const typename RigidConstraintData::SE3 & c1Mc2 = contact_data.c1Mc2;

        if (contact_model.type == CONTACT_6D)
        {
          contact_data.contact_placement_error = -log6(c1Mc2);
          contact_acc_err.toVector().noalias() -=
            corrector.Kp.asDiagonal() * contact_data.contact_placement_error.toVector();
        }
        else if (contact_model.type == CONTACT_3D)
        {
          contact_data.contact_placement_error.linear() = -c1Mc2.translation();
          contact_data.contact_acceleration_error.angular().setZero();
          contact_acc_err.linear().noalias() -=
            corrector.Kp.asDiagonal() * contact_data.contact_placement_error.linear();
        }
      }

      for (int j = condim_counter[joint_id]; j < condim_counter[joint_id] + con_dim; j++)
      {
        data.lA[joint_id][j] -=
          (data.KA[joint_id].col(j).template head<3>().transpose()
           * data.a_gf[joint_id].linear_impl());
      }
      if (contact_model.type == CONTACT_3D)
      {
        vc1 = contact_model.joint1_placement.actInv(data.v[joint_id]);
        data.lA[joint_id].segment(condim_counter[joint_id], 3).noalias() +=
          vc1.angular().cross(vc1.linear()) - contact_acc_err.linear();
      }
      else
      {
        data.lA[joint_id].segment(condim_counter[joint_id], 6).noalias() -=
          contact_acc_err.toVector();
      }
      condim_counter[joint_id] += con_dim;
    }

    typedef PvRegBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
    for (JointIndex i = (JointIndex)model.njoints - 1; i >= 1; --i)
    {
      Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));
    }

    if (data.lA[0].size() > 0)
    {
      data.lA[0].noalias() += data.KA[0].transpose() * data.a_gf[0].toVector();
      data.lambdaA[0].noalias() = data.lA[0];
      data.LA[0].noalias() +=
        settings.mu
        * Data::MatrixXs::Identity(
          data.constraints_supported_dim[0], data.constraints_supported_dim[0]);
      data.osim_llt.compute(data.LA[0]);
      data.lambda_c_prox.setZero();
      int i = 0;
      for (i = 0; i < settings.max_iter; i++)
      {
        data.lambdaA[0].noalias() = settings.mu * data.lambda_c_prox + data.lA[0];
        data.osim_llt.solveInPlace(data.lambdaA[0]);
        settings.absolute_residual =
          (data.lambda_c_prox - data.lambdaA[0]).template lpNorm<Eigen::Infinity>();
        if (check_expression_if_real<Scalar, false>(
              settings.absolute_residual
              <= settings.absolute_accuracy)) // In the case where Scalar is not double, this will
                                              // iterate for max_it.
          break;
        data.lambda_c_prox.noalias() = data.lambdaA[0];
      }
    }

    typedef PvRegForwardStep2<Scalar, Options, JointCollectionTpl> Pass3;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i], data.joints[i], typename Pass3::ArgsType(model, data));
    }

    return data.ddq;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ContactModelAllocator,
    class ContactDataAllocator>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
  constrainedABA(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ContactModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ContactDataAllocator> & contact_datas,
    ProximalSettingsTpl<Scalar> & settings)
  {

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

    bool baumgarte_position = false;
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      if (!check_expression_if_real<Scalar, false>(
            contact_models[i].corrector.Kp.isZero(Scalar(0))))
      {
        baumgarte_position = true;
        break;
      }
    }

    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    data.a[0] = data.a_gf[0];
    data.f[0].setZero();
    data.u = tau;

    // Set the lA and LA at the contact points to zero.
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      data.lA[joint_id].setZero();
    }

    typedef PvForwardStep1<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i],
        typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
      if (baumgarte_position)
      {
        const JointIndex & parent = model.parents[i];
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];
      }
    }

    std::vector<int> condim_counter(static_cast<size_t>(model.njoints), 0);
    // Update lAs
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      typename RigidConstraintData::Motion & vc1 = contact_datas[i].contact1_velocity;
      typename RigidConstraintData::Motion & vc2 = contact_datas[i].contact2_velocity;
      const JointIndex & joint_id = contact_model.joint1_id;
      int con_dim = contact_model.size();
      const typename RigidConstraintModel::BaumgarteCorrectorParameters & corrector =
        contact_model.corrector;
      typename RigidConstraintData::Motion & contact_acc_err =
        contact_datas[i].contact_acceleration_error;
      typename RigidConstraintData::Motion & contact_vel_err =
        contact_datas[i].contact_velocity_error;

      const JointIndex & joint2_id = contact_model.joint2_id;
      if (joint2_id > 0)
        assert(false), "Internal loops are not yet permitted in constrainedABA";
      else
        vc2.setZero();

      contact_acc_err.setZero();
      if (!check_expression_if_real<Scalar, false>(corrector.Kd.isZero(Scalar(0))))
      {

        // TODO: modify for closed loops by subtracting vc2_in_frame1
        if (contact_model.type == CONTACT_6D)
        {
          contact_vel_err = vc1;
          contact_acc_err.toVector().noalias() -=
            corrector.Kd.asDiagonal() * contact_vel_err.toVector();
        }
        else
        {
          contact_vel_err = vc1;
          contact_vel_err.angular().setZero();
          contact_acc_err.linear().noalias() -=
            corrector.Kd.asDiagonal() * contact_vel_err.linear();
        }
      }

      if (!check_expression_if_real<Scalar, false>(corrector.Kp.isZero(Scalar(0))))
      {
        RigidConstraintData & contact_data = contact_datas[i];
        const typename RigidConstraintData::SE3 & c1Mc2 = contact_data.c1Mc2;

        if (contact_model.type == CONTACT_6D)
        {
          contact_data.contact_placement_error = -log6(c1Mc2);
          contact_acc_err.toVector().noalias() -=
            corrector.Kp.asDiagonal() * contact_data.contact_placement_error.toVector();
        }
        else if (contact_model.type == CONTACT_3D)
        {
          contact_data.contact_placement_error.linear() = -c1Mc2.translation();
          contact_data.contact_acceleration_error.angular().setZero();
          contact_acc_err.linear().noalias() -=
            corrector.Kp.asDiagonal() * contact_data.contact_placement_error.linear();
        }
      }

      for (int j = condim_counter[joint_id]; j < condim_counter[joint_id] + con_dim; j++)
      {
        data.lA[joint_id][j] -=
          (data.KA[joint_id].col(j).template head<3>().transpose()
           * data.a_gf[joint_id].linear_impl());
      }
      if (contact_model.type == CONTACT_3D)
      {
        vc1 = contact_model.joint1_placement.actInv(data.v[joint_id]);
        data.lA[joint_id].segment(condim_counter[joint_id], 3).noalias() +=
          vc1.angular().cross(vc1.linear()) - contact_acc_err.linear();
      }
      else
      {
        data.lA[joint_id].segment(condim_counter[joint_id], 6).noalias() -=
          contact_acc_err.toVector();
      }
      condim_counter[joint_id] += con_dim;
    }

    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      data.Yaba[joint_id].matrix().noalias() +=
        data.KA[joint_id] * (1 / settings.mu) * data.KA[joint_id].transpose();
      data.f[joint_id].toVector().noalias() +=
        data.KA[joint_id] * (1 / settings.mu) * data.lA[joint_id];
    }

    typedef cAbaBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));
    }

    std::fill(condim_counter.begin(), condim_counter.end(), 0);
    assert(settings.mu > 0 && "constrainedABA requires mu > 0.");
    typedef PvBackwardStepReduced<Scalar, Options, JointCollectionTpl> Pass4;
    int lambda_ind = 0;
    for (std::size_t j = 0; j < contact_models.size(); ++j)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[j];
      const JointIndex & joint_id = contact_model.joint1_id;
      data.lambda_c_prox.segment(lambda_ind, contact_model.size()).noalias() =
        (data.lA[joint_id].segment(condim_counter[joint_id], contact_model.size())
         + data.KA[joint_id].middleCols(condim_counter[joint_id], contact_model.size()).transpose()
             * data.a[joint_id].toVector());
      lambda_ind += contact_model.size();
      condim_counter[joint_id] += contact_model.size();
    }

    typedef cAbaForwardStep2<Scalar, Options, JointCollectionTpl> Pass3;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      if (data.constraints_supported_dim[i] > 0)
        Pass3::run(model.joints[i], data.joints[i], typename Pass3::ArgsType(model, data));
    }

    for (int i = 1; i < settings.max_iter; i++)
    {

      data.lambdaA[0].noalias() += (1 / settings.mu) * data.lambda_c_prox;
      for (JointIndex j = 1; j < (JointIndex)model.njoints; ++j)
      {
        if (data.constraints_supported_dim[j] > 0)
          data.f[j].toVector().setZero();
      }
      // Compute lambda_prox and update the data.f
      lambda_ind = 0;
      std::fill(condim_counter.begin(), condim_counter.end(), 0);
      for (std::size_t j = 0; j < contact_models.size(); ++j)
      {
        const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[j];
        const JointIndex & joint_id = contact_model.joint1_id;
        data.f[joint_id].toVector().noalias() +=
          data.KA[joint_id].middleCols(condim_counter[joint_id], contact_model.size())
          * (1 / settings.mu) * (data.lambda_c_prox.segment(lambda_ind, contact_model.size()));
        lambda_ind += contact_model.size();
        condim_counter[joint_id] += contact_model.size();
      }
      // reduced backward sweep
      for (JointIndex j = (JointIndex)model.njoints - 1; j > 0; --j)
      {
        if (data.constraints_supported_dim[j] > 0)
          Pass4::run(model.joints[j], data.joints[j], typename Pass4::ArgsType(model, data));
      }
      // outward sweep
      for (JointIndex j = 1; j < (JointIndex)model.njoints; ++j)
      {
        if (data.constraints_supported_dim[j] > 0)
          Pass3::run(model.joints[j], data.joints[j], typename Pass3::ArgsType(model, data));
      }
      lambda_ind = 0;
      std::fill(condim_counter.begin(), condim_counter.end(), 0);
      for (std::size_t j = 0; j < contact_models.size(); ++j)
      {
        const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[j];
        const JointIndex & joint_id = contact_model.joint1_id;
        data.lambda_c_prox.segment(lambda_ind, contact_model.size()).noalias() =
          (data.lA[joint_id].segment(condim_counter[joint_id], contact_model.size())
           + data.KA[joint_id]
                 .middleCols(condim_counter[joint_id], contact_model.size())
                 .transpose()
               * data.a[joint_id].toVector());
        lambda_ind += contact_model.size();
        condim_counter[joint_id] += contact_model.size();
      }
      settings.absolute_residual = (data.lambda_c_prox).template lpNorm<Eigen::Infinity>();
      if (check_expression_if_real<Scalar, false>(
            settings.absolute_residual
            <= settings.absolute_accuracy)) // In the case where Scalar is not double, this will
                                            // iterate for max_it.
      {
        break;
      }
    }

    // outward sweep after convergence/timeout for joints not supporting a constraint
    for (JointIndex j = 1; j < (JointIndex)model.njoints; ++j)
    {
      if (data.constraints_supported_dim[j] == 0)
        Pass3::run(model.joints[j], data.joints[j], typename Pass3::ArgsType(model, data));
    }

    return data.ddq;
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_algorithm_pv_hxx__
