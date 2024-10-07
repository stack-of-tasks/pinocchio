//
// Copyright (c) 2019-2022 INRIA CNRS
//

#ifndef __pinocchio_algorithm_constraint_dynamics_hxx__
#define __pinocchio_algorithm_constraint_dynamics_hxx__

#include "pinocchio/spatial/classic-acceleration.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include <limits>

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class Allocator>
  inline void initConstraintDynamics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, Allocator> & contact_models)
  {
    data.contact_chol.allocate(model, contact_models);
    data.primal_dual_contact_solution.resize(data.contact_chol.size());
    data.primal_rhs_contact.resize(data.contact_chol.constraintDim());

    data.lambda_c.resize(data.contact_chol.constraintDim());
    data.lambda_c_prox.resize(data.contact_chol.constraintDim());
    data.impulse_c.resize(data.contact_chol.constraintDim());

    // TODO: should be moved elsewhere
    data.dlambda_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dlambda_dx_prox.resize(data.contact_chol.constraintDim(), model.nv);
    data.drhs_prox.resize(data.contact_chol.constraintDim(), model.nv);
    data.dlambda_dv.resize(data.contact_chol.constraintDim(), model.nv);
    data.dlambda_dtau.resize(data.contact_chol.constraintDim(), model.nv);
    data.dvc_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_dv.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_da.resize(data.contact_chol.constraintDim(), model.nv);
    data.osim.resize(data.contact_chol.constraintDim(), data.contact_chol.constraintDim());

    data.lambda_c.setZero();
    data.lambda_c_prox.setZero();
    data.dlambda_dq.setZero();
    data.dlambda_dv.setZero();
    data.dlambda_dtau.setZero();
    data.dvc_dq.setZero();
    data.dac_dq.setZero();
    data.dac_dv.setZero();
    data.dac_da.setZero();
    data.osim.setZero();
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType,
    bool ContactMode>
  struct ContactAndImpulseDynamicsForwardStep
  : public fusion::JointUnaryVisitorBase<ContactAndImpulseDynamicsForwardStep<
      Scalar,
      Options,
      JointCollectionTpl,
      ConfigVectorType,
      TangentVectorType,
      ContactMode>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::
      vector<const Model &, Data &, const ConfigVectorType &, const TangentVectorType &>
        ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;
      typedef typename Data::Force Force;
      typedef typename Data::Inertia Inertia;

      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];

      Motion & ov = data.ov[i];
      Inertia & oinertias = data.oinertias[i];

      jmodel.calc(jdata.derived(), q.derived(), v.derived());

      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      if (parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];

      ov = data.oMi[i].act(jdata.v());
      if (parent > 0)
        ov += data.ov[parent];

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      oinertias = data.oMi[i].act(model.inertias[i]);
      data.oYcrb[i] = data.oinertias[i];
      if (ContactMode)
      {
        Force & oh = data.oh[i];
        Force & of = data.of[i];
        Motion & oa = data.oa[i];
        Motion & oa_gf = data.oa_gf[i];
        oh = oinertias * ov;
        oa = data.oMi[i].act(jdata.c());
        if (parent > 0)
        {
          oa += (data.ov[parent] ^ ov);
          oa += data.oa[parent];
        }
        oa_gf = oa - model.gravity; // add gravity contribution
        of = oinertias * oa_gf + ov.cross(oh);
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    bool ContactMode>
  struct ContactAndImpulseDynamicsBackwardStep
  : public fusion::JointUnaryVisitorBase<
      ContactAndImpulseDynamicsBackwardStep<Scalar, Options, JointCollectionTpl, ContactMode>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel, const Model & model, Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      ColsBlock dFda_cols = jmodel.jointCols(data.dFda);
      const ColsBlock J_cols = jmodel.jointCols(data.J);
      motionSet::inertiaAction(data.oYcrb[i], J_cols, dFda_cols);

      data.M.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
        J_cols.transpose() * data.dFda.middleCols(jmodel.idx_v(), data.nvSubtree[i]);
      data.oYcrb[parent] += data.oYcrb[i];

      if (ContactMode)
      {
        jmodel.jointVelocitySelector(data.nle).noalias() =
          J_cols.transpose() * data.of[i].toVector();
        data.of[parent] += data.of[i];
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
  constraintDynamics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_datas,
    ProximalSettingsTpl<Scalar> & settings)
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Data::Motion Motion;

    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef std::vector<RigidConstraintModel, ConstraintModelAllocator> VectorRigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(settings.mu >= Scalar(0)), "mu has to be positive");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      contact_models.size(), contact_datas.size(),
      "The contact models and data do not have the same vector size.");

    // Check that all the frames are related to LOCAL or LOCAL_WORLD_ALIGNED reference frames
    for (typename VectorRigidConstraintModel::const_iterator cm_it = contact_models.begin();
         cm_it != contact_models.end(); ++cm_it)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        cm_it->reference_frame != WORLD, "Contact model with name " + cm_it->name
                                           + " has reference_frame equals to WORLD. "
                                             "constraintDynamics is only operating from LOCAL or "
                                             "LOCAL_WORLD_ALIGNED reference frames.")
    }

    typename Data::TangentVectorType & a = data.ddq;
    typename Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
    typename Data::VectorXs & primal_dual_contact_solution = data.primal_dual_contact_solution;
    typename Data::VectorXs & primal_rhs_contact = data.primal_rhs_contact;

    data.oYcrb[0].setZero();
    data.of[0].setZero();
    data.oa[0].setZero();
    data.ov[0].setZero();
    typedef ContactAndImpulseDynamicsForwardStep<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1, true>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i],
        typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
    }

    typedef ContactAndImpulseDynamicsBackwardStep<Scalar, Options, JointCollectionTpl, true> Pass2;
    for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
    {
      Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
    }

    // Add the armature contribution
    data.M.diagonal() += model.armature;

    // Retrieve the Centroidal Momemtum map
    typedef typename Data::Force Force;
    typedef Eigen::Block<typename Data::Matrix6x, 3, -1> Block3x;

    data.com[0] = data.oYcrb[0].lever();

    data.Ag = data.dFda;
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for (long i = 0; i < model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);

    // Computes the Cholesky decomposition
    const Scalar mu = settings.mu;
    contact_chol.compute(model, data, contact_models, contact_datas, mu);

    primal_dual_contact_solution.tail(model.nv) = tau - data.nle;

    // Temporary variables
    //    Motion coriolis_centrifugal_acc1, coriolis_centrifugal_acc2; // Coriolis/centrifugal
    //    acceleration of the contact frame.
    typename Motion::Vector3 coriolis_centrifugal_acc1_local;
    typename Motion::Vector3 coriolis_centrifugal_acc2_local;

    Eigen::DenseIndex current_row_id = 0;
    for (size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidConstraintModel & contact_model = contact_models[contact_id];
      RigidConstraintData & contact_data = contact_datas[contact_id];
      const int contact_dim = contact_model.size();

      const typename RigidConstraintModel::BaumgarteCorrectorParameters & corrector =
        contact_model.corrector;
      const typename RigidConstraintData::Motion & contact_acceleration_desired =
        contact_data.contact_acceleration_desired;
      typename RigidConstraintData::Motion & contact_acceleration_error =
        contact_data.contact_acceleration_error;

      const typename Model::JointIndex joint1_id = contact_model.joint1_id;
      typename RigidConstraintData::SE3 & oMc1 = contact_data.oMc1;
      typename RigidConstraintData::Motion & vc1 = contact_data.contact1_velocity;
      typename RigidConstraintData::Motion & coriolis_centrifugal_acc1 =
        contact_data.contact1_acceleration_drift;

      const typename Model::JointIndex joint2_id = contact_model.joint2_id;
      typename RigidConstraintData::SE3 & oMc2 = contact_data.oMc2;
      typename RigidConstraintData::Motion & vc2 = contact_data.contact2_velocity;
      typename RigidConstraintData::Motion & coriolis_centrifugal_acc2 =
        contact_data.contact2_acceleration_drift;

      const typename RigidConstraintData::SE3 & c1Mc2 = contact_data.c1Mc2;

      // Compute contact placement and velocities
      if (joint1_id > 0)
        vc1 = oMc1.actInv(data.ov[joint1_id]);
      else
        vc1.setZero();
      if (joint2_id > 0)
        vc2 = oMc2.actInv(data.ov[joint2_id]);
      else
        vc2.setZero();

      const Motion vc2_in_frame1 = c1Mc2.act(vc2);
      // Compute placement and velocity errors
      if (contact_model.type == CONTACT_6D)
      {
        contact_data.contact_placement_error = -log6(c1Mc2);
        contact_data.contact_velocity_error.toVector() = (vc1 - vc2_in_frame1).toVector();
      }
      else
      {
        contact_data.contact_placement_error.linear() = -c1Mc2.translation();
        contact_data.contact_placement_error.angular().setZero();

        contact_data.contact_velocity_error.linear() =
          vc1.linear() - c1Mc2.rotation() * vc2.linear();
        contact_data.contact_velocity_error.angular().setZero();
      }

      if (check_expression_if_real<Scalar, false>(
            isZero(corrector.Kp, static_cast<Scalar>(0.))
            && isZero(corrector.Kd, static_cast<Scalar>(0.))))
      {
        contact_acceleration_error.setZero();
      }
      else
      {
        if (contact_model.type == CONTACT_6D)
          contact_acceleration_error.toVector().noalias() =
            -(corrector.Kp.asDiagonal() /* * Jexp6(contact_data.contact_placement_error) */
              * contact_data.contact_placement_error.toVector())
            - (corrector.Kd.asDiagonal() * contact_data.contact_velocity_error.toVector());
        else
        {
          contact_acceleration_error.linear().noalias() =
            -(corrector.Kp.asDiagonal() * contact_data.contact_placement_error.linear())
            - (corrector.Kd.asDiagonal() * contact_data.contact_velocity_error.linear());
          contact_acceleration_error.angular().setZero();
        }
      }

      switch (contact_model.reference_frame)
      {
      case LOCAL_WORLD_ALIGNED: {
        // LINEAR
        coriolis_centrifugal_acc1.linear().noalias() =
          data.oa[joint1_id].linear() + data.oa[joint1_id].angular().cross(oMc1.translation());
        if (contact_model.type == CONTACT_3D)
          coriolis_centrifugal_acc1.linear() += data.ov[joint1_id].angular().cross(
            data.ov[joint1_id].linear() + data.ov[joint1_id].angular().cross(oMc1.translation()));
        // ANGULAR
        coriolis_centrifugal_acc1.angular() = data.oa[joint1_id].angular();

        // LINEAR
        if (contact_model.type == CONTACT_3D)
        {
          coriolis_centrifugal_acc2.linear().noalias() =
            data.oa[joint2_id].linear() + data.oa[joint2_id].angular().cross(oMc2.translation())
            + data.ov[joint2_id].angular().cross(
              data.ov[joint2_id].linear() + data.ov[joint2_id].angular().cross(oMc2.translation()));
          coriolis_centrifugal_acc2.angular().setZero();
        }
        else
        {
          coriolis_centrifugal_acc2.linear() =
            data.oa[joint2_id].linear() + data.oa[joint2_id].angular().cross(oMc1.translation());
          coriolis_centrifugal_acc2.angular() = data.oa[joint2_id].angular();
        }

        contact_acceleration_error.linear() = oMc1.rotation() * contact_acceleration_error.linear();
        contact_acceleration_error.angular() =
          oMc1.rotation() * contact_acceleration_error.angular();
        break;
      }
      case LOCAL: {
        coriolis_centrifugal_acc1 = oMc1.actInv(data.oa[joint1_id]);
        if (contact_model.type == CONTACT_3D)
        {
          coriolis_centrifugal_acc1.linear() += vc1.angular().cross(vc1.linear());
          coriolis_centrifugal_acc1.angular().setZero();
        }
        else
          coriolis_centrifugal_acc1 += contact_data.contact_velocity_error.cross(vc2_in_frame1);

        if (contact_model.type == CONTACT_3D)
        {
          coriolis_centrifugal_acc2.linear().noalias() =
            oMc1.rotation().transpose()
            * (data.oa[joint2_id].linear() + data.oa[joint2_id].angular().cross(oMc2.translation())
               + data.ov[joint2_id].angular().cross(
                 data.ov[joint2_id].linear()
                 + data.ov[joint2_id].angular().cross(oMc2.translation())));
          coriolis_centrifugal_acc2.angular().setZero();
        }
        else
          coriolis_centrifugal_acc2 = oMc1.actInv(data.oa[joint2_id]);
        break;
      }
      default:
        assert(false && "must never happened");
        break;
      }

      contact_data.contact_acceleration = coriolis_centrifugal_acc2;
      switch (contact_model.type)
      {
      case CONTACT_3D:
        primal_rhs_contact.segment(current_row_id, contact_dim) =
          -coriolis_centrifugal_acc1.linear() + coriolis_centrifugal_acc2.linear()
          + contact_acceleration_error.linear() + contact_acceleration_desired.linear();
        break;
      case CONTACT_6D:
        primal_rhs_contact.segment(current_row_id, contact_dim) =
          -coriolis_centrifugal_acc1.toVector() + coriolis_centrifugal_acc2.toVector()
          + contact_acceleration_error.toVector() + contact_acceleration_desired.toVector();
        break;
      default:
        assert(false && "must never happened");
        break;
      }

      current_row_id += contact_dim;
    }

    // Solve the system
    //    Scalar primal_infeasibility = Scalar(0);
    int it = 0;
    data.lambda_c_prox.setZero();
    const Eigen::DenseIndex constraint_dim = contact_chol.constraintDim();
    for (; it < settings.max_iter;)
    {
      it++;
      primal_dual_contact_solution.head(constraint_dim) =
        primal_rhs_contact + data.lambda_c_prox * settings.mu;
      primal_dual_contact_solution.tail(model.nv) = tau - data.nle;
      contact_chol.solveInPlace(primal_dual_contact_solution);

      // Use data.lambda_c as tmp variable for computing the constraint residual
      contact_chol.getDelassusCholeskyExpression().applyOnTheRight(
        primal_dual_contact_solution.head(constraint_dim), data.lambda_c);
      data.lambda_c -= mu * primal_dual_contact_solution.head(constraint_dim)
                       + primal_rhs_contact.head(constraint_dim);

      settings.absolute_residual = data.lambda_c.template lpNorm<Eigen::Infinity>();
      settings.relative_residual =
        (primal_dual_contact_solution.head(constraint_dim) + data.lambda_c_prox)
          .template lpNorm<Eigen::Infinity>();

      data.lambda_c_prox = -primal_dual_contact_solution.head(constraint_dim);

      const bool convergence_criteria_reached =
        check_expression_if_real<Scalar, false>(
          settings.absolute_residual <= settings.absolute_accuracy)
        || check_expression_if_real<Scalar, false>(
          settings.relative_residual <= settings.relative_accuracy);
      if (convergence_criteria_reached) // In the case where Scalar is not double, this will iterate
                                        // for max_it.
        break;
    }
    settings.iter = it;
    assert(settings.iter <= settings.max_iter && "must never happened");

    // Retrieve the joint space acceleration
    a = primal_dual_contact_solution.tail(model.nv);
    data.lambda_c = -primal_dual_contact_solution.head(constraint_dim);

    // Retrieve the contact forces
    Eigen::DenseIndex current_row_sol_id = 0;
    for (size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidConstraintModel & contact_model = contact_models[contact_id];
      RigidConstraintData & contact_data = contact_datas[contact_id];
      typename RigidConstraintData::Force & fext = contact_data.contact_force;
      const int contact_dim = contact_model.size();

      switch (contact_model.type)
      {
      case CONTACT_3D: {
        fext.linear() = -primal_dual_contact_solution.template segment<3>(current_row_sol_id);
        fext.angular().setZero();
        break;
      }
      case CONTACT_6D: {
        typedef typename Data::VectorXs::template FixedSegmentReturnType<6>::Type Segment6d;
        const ForceRef<Segment6d> f_sol(
          primal_dual_contact_solution.template segment<6>(current_row_sol_id));
        fext = -f_sol;
        break;
      }
      default:
        assert(false && "must never happened");
        break;
      }

      current_row_sol_id += contact_dim;
    }

    return a;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  struct ContactABAForwardStep1
  : public fusion::JointUnaryVisitorBase<ContactABAForwardStep1<
      Scalar,
      Options,
      JointCollectionTpl,
      ConfigVectorType,
      TangentVectorType>>
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

      const JointIndex & i = jmodel.id();
      jmodel.calc(jdata.derived(), q.derived(), v.derived());

      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      if (parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());

      data.ov[i] = data.oMi[i].act(jdata.v());
      if (parent > 0)
        data.ov[i] += data.ov[parent];

      data.oa[i] = data.oMi[i].act(jdata.c());
      if (parent > 0)
      {
        data.oa[i] += (data.ov[parent] ^ data.ov[i]);
      }

      data.oa_drift[i] = data.oa[i];
      if (parent > 0)
      {
        data.oa_drift[i] += data.oa_drift[parent];
      }

      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      data.oYaba[i] = data.oYcrb[i].matrix();
      data.of[i] = data.oYcrb[i].vxiv(data.ov[i]) - data.oYcrb[i] * model.gravity; // -f_ext
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename TangentVectorType>
  struct ContactABABackwardStep1
  : public fusion::JointUnaryVisitorBase<
      ContactABABackwardStep1<Scalar, Options, JointCollectionTpl, TangentVectorType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &, const TangentVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<TangentVectorType> & tau)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;
      typedef typename Data::Matrix6x Matrix6x;

      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.oYaba[i];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);

      Force & fi_augmented = data.of_augmented[i];
      const Force & fi = data.of[i];

      fi_augmented += fi;
      jmodel.jointVelocitySelector(data.u).noalias() =
        jmodel.jointVelocitySelector(tau) - Jcols.transpose() * fi_augmented.toVector();

      jdata.U().noalias() = Ia * Jcols;
      jdata.StU().noalias() = Jcols.transpose() * jdata.U();

      // Account for the rotor inertia contribution
      jdata.StU().diagonal() += jmodel.jointVelocitySelector(model.armature);

      internal::PerformStYSInversion<Scalar>::run(jdata.StU(), jdata.Dinv());
      jdata.UDinv().noalias() = jdata.U() * jdata.Dinv();

      if (parent > 0)
      {
        Ia.noalias() -= jdata.UDinv() * jdata.U().transpose();

        fi_augmented.toVector().noalias() +=
          Ia * data.oa[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.oYaba[parent] += Ia;
        data.of_augmented[parent] += fi_augmented;
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename TangentVectorType>
  struct ContactABABackwardStepAugmented
  : public fusion::JointUnaryVisitorBase<
      ContactABABackwardStepAugmented<Scalar, Options, JointCollectionTpl, TangentVectorType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &, const TangentVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<TangentVectorType> & tau)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;
      typedef typename Data::Matrix6x Matrix6x;

      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.oYaba[i];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);

      Force & fi_augmented = data.of_augmented[i];
      const Force & fi = data.of[i];

      fi_augmented += fi;
      jmodel.jointVelocitySelector(data.u).noalias() =
        jmodel.jointVelocitySelector(tau) - Jcols.transpose() * fi_augmented.toVector();

      if (parent > 0)
      {
        fi_augmented.toVector().noalias() +=
          Ia * data.oa[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.of_augmented[parent] += fi_augmented;
      }
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct ContactABAForwardStep2
  : public fusion::JointUnaryVisitorBase<
      ContactABAForwardStep2<Scalar, Options, JointCollectionTpl>>
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
      typedef typename Data::Matrix6x Matrix6x;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);

      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];

      data.oa_augmented[i] = data.oa[i];
      if (parent > 0)
        data.oa_augmented[i] +=
          data.oa_augmented[parent]; // does not take into account the gravity field
      jmodel.jointVelocitySelector(data.ddq).noalias() =
        jdata.Dinv() * jmodel.jointVelocitySelector(data.u)
        - jdata.UDinv().transpose() * data.oa_augmented[i].toVector();
      data.oa_augmented[i].toVector() += Jcols * jmodel.jointVelocitySelector(data.ddq);
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ModelAllocator,
    class DataAllocator>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
  contactABA(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    ProximalSettingsTpl<Scalar> & settings)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(settings.mu >= Scalar(0)), "mu has to be positive");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      contact_models.size(), contact_data.size(), "contact models and data size are not the same");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef typename Data::Motion Motion;

    typedef typename Model::JointIndex JointIndex;
    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;
    typedef typename Data::Force Force;

    typedef ContactABAForwardStep1<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i],
        typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
      data.of_augmented[i].setZero();
    }

    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];

      const typename Model::JointIndex joint1_id = cmodel.joint1_id;

      // Compute relative placement between the joint and the contact frame
      SE3 & oMc = cdata.oMc1;
      oMc = data.oMi[joint1_id] * cmodel.joint1_placement; // contact placement

      typedef typename Data::Inertia Inertia;
      typedef typename Inertia::Symmetric3 Symmetric3;

      // Add contact inertia to the joint articulated inertia
      Symmetric3 S(Symmetric3::Zero());
      if (cmodel.type == CONTACT_6D)
        S.setDiagonal(Symmetric3::Vector3::Constant(settings.mu));

      const Inertia contact_inertia(settings.mu, oMc.translation(), S);
      data.oYaba[joint1_id] += contact_inertia.matrix();

      typename Data::Motion & joint_velocity = data.ov[joint1_id];
      Motion & contact1_velocity = cdata.contact1_velocity;
      contact1_velocity = oMc.actInv(joint_velocity);

      typename Data::Motion & joint_spatial_acceleration_drift = data.oa_drift[joint1_id];
      Motion & contact_acceleration_drift = cdata.contact1_acceleration_drift;
      contact_acceleration_drift =
        cmodel.desired_contact_acceleration - oMc.actInv(joint_spatial_acceleration_drift);

      // Handle the classic acceleration term
      if (cmodel.type == CONTACT_3D)
        contact_acceleration_drift.linear() -=
          contact1_velocity.angular().cross(contact1_velocity.linear());

      // Init contact force
      //      cdata.contact_force.setZero();

      // Add the contribution of the constraints to the force vector
      data.of_augmented[joint1_id] = oMc.act(cdata.contact_force);
      if (cmodel.type == CONTACT_3D)
      {
        data.of_augmented[joint1_id] -=
          settings.mu * oMc.act(Force(contact_acceleration_drift.linear(), Force::Vector3::Zero()));
      }
      else
      {
        data.of_augmented[joint1_id] -=
          oMc.act(Force(settings.mu * contact_acceleration_drift.toVector()));
      }
    }

    typedef ContactABABackwardStep1<Scalar, Options, JointCollectionTpl, TangentVectorType2> Pass2;
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      Pass2::run(
        model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data, tau.derived()));
    }

    typedef ContactABAForwardStep2<Scalar, Options, JointCollectionTpl> Pass3;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i], data.joints[i], typename Pass3::ArgsType(model, data));
      data.of_augmented[i].setZero();
    }

    settings.iter = 0;
    bool optimal_solution_found = false;
    if (contact_models.size() == 0)
    {
      return data.ddq;
    }

    Scalar primal_infeasibility = Scalar(0);
    int it = 0;
    for (int it = 0; it < settings.max_iter; ++it)
    {
      // Compute contact acceleration errors and max contact errors, aka primal_infeasibility
      primal_infeasibility = Scalar(0);
      for (size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
      {
        const RigidConstraintModel & cmodel = contact_models[contact_id];
        RigidConstraintData & cdata = contact_data[contact_id];

        const typename Model::JointIndex & joint1_id = cmodel.joint1_id;

        const SE3 & oMc = cdata.oMc1;
        const Motion & contact1_velocity = cdata.contact1_velocity;

        // Compute contact acceleration error (drift)
        const typename Data::Motion & joint_spatial_acceleration = data.oa_augmented[joint1_id];
        cdata.contact_acceleration_deviation =
          oMc.actInv(joint_spatial_acceleration) - cmodel.desired_contact_acceleration;
        if (cmodel.type == CONTACT_3D)
          cdata.contact_acceleration_deviation.linear() +=
            contact1_velocity.angular().cross(contact1_velocity.linear());

        using std::max;
        if (cmodel.type == CONTACT_3D)
        {
          primal_infeasibility = max<Scalar>(
            primal_infeasibility,
            cdata.contact_acceleration_deviation.linear().template lpNorm<Eigen::Infinity>());
        }
        else
        {
          primal_infeasibility = max<Scalar>(
            primal_infeasibility,
            cdata.contact_acceleration_deviation.toVector().template lpNorm<Eigen::Infinity>());
        }
      }

      if (check_expression_if_real<Scalar, false>(
            primal_infeasibility < settings.absolute_accuracy))
      {
        optimal_solution_found = true;
        break;
      }

      // Update contact forces
      for (size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
      {
        const RigidConstraintModel & cmodel = contact_models[contact_id];
        RigidConstraintData & cdata = contact_data[contact_id];

        const typename Model::JointIndex & joint1_id = cmodel.joint1_id;

        const SE3 & oMc = cdata.oMc1;

        // Update contact force value
        if (cmodel.type == CONTACT_3D)
          cdata.contact_force.linear().noalias() +=
            settings.mu * cdata.contact_acceleration_deviation.linear();
        else
          cdata.contact_force.toVector().noalias() +=
            settings.mu * cdata.contact_acceleration_deviation.toVector();

        // Add the contribution of the constraints to the force vector
        const Motion & contact_acceleration_drift = cdata.contact1_acceleration_drift;
        data.of_augmented[joint1_id] = oMc.act(cdata.contact_force);
        if (cmodel.type == CONTACT_3D)
        {
          data.of_augmented[joint1_id] -=
            settings.mu
            * oMc.act(Force(contact_acceleration_drift.linear(), Force::Vector3::Zero()));
        }
        else
        {
          data.of_augmented[joint1_id] -=
            oMc.act(Force(settings.mu * contact_acceleration_drift.toVector()));
        }
      }

      typedef ContactABABackwardStepAugmented<
        Scalar, Options, JointCollectionTpl, TangentVectorType2>
        Pass2Augmented;
      for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
      {
        Pass2Augmented::run(
          model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data, tau.derived()));
      }

      typedef ContactABAForwardStep2<Scalar, Options, JointCollectionTpl> Pass3;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass3::run(model.joints[i], data.joints[i], typename Pass3::ArgsType(model, data));
        data.of_augmented[i].setZero();
      }
    }

    settings.iter = it;
    settings.absolute_residual = primal_infeasibility;

    return data.ddq;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ModelAllocator,
    class DataAllocator>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & proxLTLs(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    ProximalSettingsTpl<Scalar> & settings)
  {
    // TODO: wip not yet tested.
    using namespace Eigen;

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(settings.mu >= Scalar(0)), "mu has to be positive");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      contact_models.size(), contact_data.size(), "contact models and data size are not the same");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef typename Data::Motion Motion;

    typedef typename Model::JointIndex JointIndex;
    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;
    typedef typename Data::Force Force;

    // Forward pass to compute the Jacobian and the inertia matrices
    typedef ContactABAForwardStep1<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1>
      Pass1;
    data.tau = tau;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i],
        typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
      data.of_augmented[i].setZero();
      data.of[i] += data.oYcrb[i] * data.oa_drift[i];
    }

    // Update the inertia matrix of the constrained links
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];

      const typename Model::JointIndex joint1_id = cmodel.joint1_id;

      // Compute relative placement between the joint and the contact frame
      SE3 & oMc = cdata.oMc1;
      oMc = data.oMi[joint1_id] * cmodel.joint1_placement; // contact placement

      typedef typename Data::Inertia Inertia;
      typedef typename Inertia::Symmetric3 Symmetric3;

      // Add contact inertia to the joint articulated inertia
      Symmetric3 S(Symmetric3::Zero());
      if (cmodel.type == CONTACT_6D)
        S.setDiagonal(Symmetric3::Vector3::Constant(settings.mu));

      const Inertia contact_inertia(settings.mu, oMc.translation(), S);
      data.oYcrb[joint1_id] += contact_inertia;

      typename Data::Motion & joint_velocity = data.ov[joint1_id];
      Motion & contact1_velocity = cdata.contact1_velocity;
      contact1_velocity = oMc.actInv(joint_velocity);

      typename Data::Motion & joint_spatial_acceleration_drift = data.oa_drift[joint1_id];
      Motion & contact_acceleration_drift = cdata.contact1_acceleration_drift;
      contact_acceleration_drift =
        cmodel.desired_contact_acceleration - oMc.actInv(joint_spatial_acceleration_drift);

      // Handle the classic acceleration term
      if (cmodel.type == CONTACT_3D)
        contact_acceleration_drift.linear() -=
          contact1_velocity.angular().cross(contact1_velocity.linear());

      // Init contact force
      //  cdata.contact_force.setZero();

      // Add the contribution of the constraints to the force vector
      // data.of_augmented[joint1_id] = oMc.act(cdata.contact_force);
      if (cmodel.type == CONTACT_3D)
      {
        data.of_augmented[joint1_id] -=
          settings.mu * oMc.act(Force(contact_acceleration_drift.linear(), Force::Vector3::Zero()));
      }
      else
      {
        data.of_augmented[joint1_id] -=
          oMc.act(Force(settings.mu * contact_acceleration_drift.toVector()));
      }
    }

    // Backward pass to compute the modified CRBA
    typedef impl::CrbaWorldConventionBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
    for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
    {
      Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));

      // Compute data.oF in RNEA style to get bias forces
      const JointIndex & parent = model.parents[i];
      const JointModel & jmodel = model.joints[i];
      data.of[i] += data.of_augmented[i];
      data.of[parent] += data.of[i];

      // subtract the bias forces from the torque to get Mv_dot_free
      jmodel.jointVelocitySelector(data.tau).noalias() -=
        jmodel.jointCols(data.J).transpose() * (data.of[i].toVector());
      data.of_augmented[i].toVector().setZero();
    }

    // Factorize the CRBA
    // crba(model, data, q);
    pinocchio::cholesky::decompose(model, data);
    data.ddq = pinocchio::cholesky::solve(model, data, data.tau);

    data.u = data.ddq;
    // data.tau.setZero();
    // Proximal iterations
    for (int it = 1; it < settings.max_iter; it++)
    {
      data.tau.setZero();
      // Compute accelerations and constraint residual
      data.oa_augmented[0].setZero();
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        if (data.constraints_supported_dim[i] > 0)
        {
          const JointModel & jmodel = model.joints[i];
          data.oa_augmented[i].toVector().noalias() =
            data.oa_augmented[model.parents[i]].toVector()
            + jmodel.jointCols(data.J) * jmodel.jointVelocitySelector(data.u);
          data.of_augmented[i].toVector().setZero();
        }
      }

      // Check convergence
      for (size_t k = 0; k < contact_models.size(); ++k)
      {
        const RigidConstraintModel & cmodel = contact_models[k];
        RigidConstraintData & cdata = contact_data[k];

        const typename Model::JointIndex joint1_id = cmodel.joint1_id;

        // Compute relative placement between the joint and the contact frame
        SE3 & oMc = cdata.oMc1;
        oMc = data.oMi[joint1_id] * cmodel.joint1_placement; // contact placement

        Motion & contact_acceleration_drift = cdata.contact1_acceleration_drift;
        contact_acceleration_drift -= oMc.actInv(data.oa_augmented[joint1_id]);

        // Add the contribution of the constraints to the force vector
        if (cmodel.type == CONTACT_3D)
        {
          data.of_augmented[joint1_id] -=
            settings.mu
            * oMc.act(Force(contact_acceleration_drift.linear(), Force::Vector3::Zero()));
        }
        else
        {
          data.of_augmented[joint1_id] -=
            oMc.act(Force(settings.mu * contact_acceleration_drift.toVector()));
        }
      }

      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        if (data.constraints_supported_dim[i] > 0)
        {

          const JointIndex & parent = model.parents[i];
          const JointModel & jmodel = model.joints[i];
          data.of_augmented[parent] += data.of_augmented[i];

          jmodel.jointVelocitySelector(data.tau).noalias() =
            -jmodel.jointCols(data.J).transpose() * (data.of_augmented[i].toVector());
        }
      }

      data.u = cholesky::solve(model, data, data.tau);
      data.ddq.noalias() += data.u;
    }

    return data.ddq;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraint_dynamics_hxx__
