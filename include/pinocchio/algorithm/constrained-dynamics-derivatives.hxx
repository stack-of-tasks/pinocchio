//
// Copyright (c) 2020-2022 CNRS INRIA
//

#ifndef __pinocchio_algorithm_constraint_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_constraint_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/utils/motion.hpp"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    bool ContactMode>
  struct ComputeConstraintDynamicsDerivativesForwardStep
  : public fusion::JointUnaryVisitorBase<ComputeConstraintDynamicsDerivativesForwardStep<
      Scalar,
      Options,
      JointCollectionTpl,
      ContactMode>>
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
      typedef typename Data::Motion Motion;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);

      if (ContactMode)
      {
        const Motion & ov = data.ov[i];
        ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
        ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);

        motionSet::motionAction(ov, J_cols, dJ_cols);
        // TODO: make more efficient
        data.v[i] = data.oMi[i].actInv(data.ov[i]);

        if (parent > 0)
        {
          motionSet::motionAction(data.ov[parent], J_cols, dVdq_cols);
        }
        else
          dVdq_cols.setZero();

        // computes variation of inertias
        data.doYcrb[i] = data.oinertias[i].variation(ov);
        typedef impl::ComputeRNEADerivativesForwardStep<
          Scalar, Options, JointCollectionTpl, typename Data::ConfigVectorType,
          typename Data::TangentVectorType, typename Data::TangentVectorType>
          RNEAForwardStepType;
        RNEAForwardStepType::addForceCrossMatrix(data.oh[i], data.doYcrb[i]);
        Motion & oa = data.oa[i];
        Motion & oa_gf = data.oa_gf[i];
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
        const typename Data::TangentVectorType & a = data.ddq;
        data.a[i] =
          jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
        if (parent > 0)
          data.a[i] += data.liMi[i].actInv(data.a[parent]);
        oa = data.oMi[i].act(data.a[i]);
        oa_gf = oa - model.gravity; // add gravity contribution
        data.of[i] = data.oinertias[i] * oa_gf + ov.cross(data.oh[i]);
        motionSet::motionAction(data.oa_gf[parent], J_cols, dAdq_cols);
        dAdv_cols = dJ_cols;
        if (parent > 0)
        {
          motionSet::motionAction<ADDTO>(data.ov[parent], dVdq_cols, dAdq_cols);
          dAdv_cols.noalias() += dVdq_cols;
        }
      }
      else
      {
        Motion & odv = data.oa[i];
        Motion & odvparent = data.oa[parent];
        const typename Data::TangentVectorType & dimpulse = data.ddq;
        // Temporary calculation of J(dq_after)
        odv = J_cols * jmodel.jointVelocitySelector(dimpulse);
        if (parent > 0)
          odv += odvparent;
        motionSet::motionAction(odvparent, J_cols, dAdq_cols);
        data.of[i] = data.oinertias[i] * odv;
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    bool ContactMode>
  struct ComputeContactDynamicDerivativesBackwardStep
  : public fusion::JointUnaryVisitorBase<ComputeContactDynamicDerivativesBackwardStep<
      Scalar,
      Options,
      JointCollectionTpl,
      ContactMode>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel, const Model & model, Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef Eigen::Matrix<
        Scalar, JointModel::NV, 6, Options, JointModel::NV == Eigen::Dynamic ? 6 : JointModel::NV,
        6>
        MatrixNV6;
      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      ColsBlock dFda_cols = jmodel.jointCols(data.dFda);

      typename Data::RowMatrixXs & dtau_dq = data.dtau_dq;

      // Temporary variables
      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) StdY(jmodel.nv(), 6);

      motionSet::inertiaAction(data.oYcrb[i], dAdq_cols, dFdq_cols);
      // dtau/dq
      if (parent > 0)
      {
        if (ContactMode)
        {
          dFdq_cols.noalias() += data.doYcrb[i] * dVdq_cols;
          StdY.noalias() = J_cols.transpose() * data.doYcrb[i];
          for (int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()]; j >= 0;
               j = data.parents_fromRow[(typename Model::Index)j])
          {
            dtau_dq.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() =
              dFda_cols.transpose() * data.dAdq.col(j) + StdY * data.dVdq.col(j);
          }
        }
        else
        {
          for (int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()]; j >= 0;
               j = data.parents_fromRow[(typename Model::Index)j])
          {
            dtau_dq.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() =
              dFda_cols.transpose() * data.dAdq.col(j);
          }
        }
      }

      dtau_dq.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
        J_cols.transpose() * data.dFdq.middleCols(jmodel.idx_v(), data.nvSubtree[i]);
      motionSet::act<ADDTO>(J_cols, data.of[i], dFdq_cols);

      if (ContactMode)
      {
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
        ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv);

        typename Data::RowMatrixXs & dtau_dv = data.dtau_dv;
        dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
        motionSet::inertiaAction<ADDTO>(data.oYcrb[i], dAdv_cols, dFdv_cols);

        dtau_dv.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
          J_cols.transpose() * data.dFdv.middleCols(jmodel.idx_v(), data.nvSubtree[i]);
        if (parent > 0)
        {
          for (int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()]; j >= 0;
               j = data.parents_fromRow[(typename Model::Index)j])
          {
            dtau_dv.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() =
              dFda_cols.transpose() * data.dAdv.col(j) + StdY * data.J.col(j);
          }
          data.doYcrb[parent] += data.doYcrb[i];
        }
        // Restore the status of dAdq_cols (remove gravity)
        for (Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
        {
          typedef typename ColsBlock::ColXpr ColType;
          MotionRef<ColType> min(J_cols.col(k));
          MotionRef<ColType> mout(dAdq_cols.col(k));
          mout.linear() += model.gravity.linear().cross(min.angular());
        }
      }

      if (parent > 0)
        data.of[parent] += data.of[i];
    }
  };

  namespace internal
  {

    template<typename Scalar>
    struct ContactForceContribution
    {

      template<
        int Options,
        template<typename, int> class JointCollectionTpl,
        class ConstraintModelAllocator,
        class ConstraintDataAllocator>
      static void run(
        const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
          contact_models,
        DataTpl<Scalar, Options, JointCollectionTpl> & data,
        std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> &
          contact_data)
      {
        typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
        typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;
        typedef SE3Tpl<Scalar, Options> SE3;
        typedef ForceTpl<Scalar, Options> Force;

        Force of_tmp;

        // Add the contribution of the external forces.
        for (size_t k = 0; k < contact_models.size(); ++k)
        {
          const RigidConstraintModel & cmodel = contact_models[k];
          const RigidConstraintData & cdata = contact_data[k];

          // TODO: Temporary variable
          const SE3 & oMc1 = cdata.oMc1;
          Force & of1 = data.of[cmodel.joint1_id];
          const SE3 & oMc2 = cdata.oMc2;
          Force & of2 = data.of[cmodel.joint2_id];

          switch (cmodel.reference_frame)
          {
          case LOCAL: {
            switch (cmodel.type)
            {
            case CONTACT_6D: {
              if (cmodel.joint1_id > 0)
              {
                of1 -= oMc1.act(cdata.contact_force);
              }
              if (cmodel.joint2_id > 0)
              {
                of_tmp = oMc1.act(cdata.contact_force);
                of2 += of_tmp;
              }
              break;
            }
            case CONTACT_3D: {
              of_tmp.linear().noalias() = oMc1.rotation() * cdata.contact_force.linear();

              if (cmodel.joint1_id > 0)
              {
                of1.linear().noalias() -= of_tmp.linear();
                of1.angular().noalias() -= oMc1.translation().cross(of_tmp.linear());
              }
              if (cmodel.joint2_id > 0)
              {
                of2.linear() += of_tmp.linear();
                of2.angular().noalias() += oMc2.translation().cross(of_tmp.linear());
              }
              break;
            }
            default: {
              assert(false && "must never happen");
              break;
            }
            }
            break;
          }
          case LOCAL_WORLD_ALIGNED: {
            switch (cmodel.type)
            {
            case CONTACT_6D: {
              if (cmodel.joint1_id > 0)
              {
                of1 -= cdata.contact_force;
                of1.angular().noalias() -= oMc1.translation().cross(cdata.contact_force.linear());
              }
              if (cmodel.joint2_id > 0)
              {
                of2 += cdata.contact_force;
                of2.angular().noalias() += oMc1.translation().cross(cdata.contact_force.linear());
              }
              break;
            }
            case CONTACT_3D: {
              if (cmodel.joint1_id > 0)
              {
                of1.linear() -= cdata.contact_force.linear();
                of1.angular().noalias() -= oMc1.translation().cross(cdata.contact_force.linear());
              }
              if (cmodel.joint2_id > 0)
              {
                of2.linear() += cdata.contact_force.linear();
                of2.angular().noalias() += oMc2.translation().cross(cdata.contact_force.linear());
              }
              break;
            }
            default: {
              assert(false && "must never happen");
              break;
            }
            }
            break;
          }
          default:
            assert(false && "Should never happen");
            break;
          }
        }
      }
    };
  } // namespace internal

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3,
    typename MatrixType4,
    typename MatrixType5,
    typename MatrixType6>
  inline void computeConstraintDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const ProximalSettingsTpl<Scalar> & settings,
    const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
    const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
    const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
    const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau)
  {
    const Eigen::DenseIndex & nc = data.contact_chol.constraintDim();

    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      contact_data.size() == contact_models.size(),
      "contact_data and contact_models do not have the same size");

    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dq.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dv.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dtau.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dtau.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dq.rows() == nc);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dv.rows() == nc);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dtau.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dtau.rows() == nc);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(
        model.gravity.angular()[0] == Scalar(0) && model.gravity.angular()[1] == Scalar(0)
        && model.gravity.angular()[2] == Scalar(0)),
      "The gravity must be a pure force vector, no angular part");

    assert(model.check(data) && "data is not consistent with model.");

    // TODO: User should make sure the internal quantities are reset.
    data.dtau_dq.setZero();
    data.dtau_dv.setZero();
    data.dac_dq.setZero();
    data.dac_dv.setZero();

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef typename Data::SE3 SE3;
    typedef typename Data::Motion Motion;
    typedef typename Data::Force Force;
    data.oa_gf[0] = -model.gravity;

    // TODO: Temp variable
    Motion a_tmp;

    typedef ComputeConstraintDynamicsDerivativesForwardStep<
      Scalar, Options, JointCollectionTpl, true>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data));
    }

    internal::ContactForceContribution<Scalar>::run(contact_models, data, contact_data);

    // Backward Pass
    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar, Options, JointCollectionTpl, true>
      Pass2;
    for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
    {
      Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
    }

    // Compute the contact frame partial derivatives
    typename Data::SE3::Matrix6 Jlog;
    Eigen::DenseIndex current_row_sol_id = 0;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      typedef typename RigidConstraintModel::BooleanVector BooleanVector;

      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];
      //      const BooleanVector & joint1_indexes = cmodel.colwise_joint1_sparsity;
      const BooleanVector & joint2_indexes = cmodel.colwise_joint2_sparsity;

      switch (cmodel.type)
      {
      case CONTACT_6D: {
        typedef
          typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        RowsBlock contact_dvc_dq = SizeDepType<6>::middleRows(data.dvc_dq, current_row_sol_id);
        RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq, current_row_sol_id);
        RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv, current_row_sol_id);
        RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da, current_row_sol_id);

        if (cmodel.joint1_id > 0)
        {
          cdata.dv1_dq.setZero();
          cdata.da1_dq.setZero();
          cdata.da1_dv.setZero();
          cdata.da1_da.setZero();

          getFrameAccelerationDerivatives(
            model, data, cmodel.joint1_id, cmodel.joint1_placement, cmodel.reference_frame,
            cdata.dv1_dq, cdata.da1_dq, cdata.da1_dv, cdata.da1_da);

          contact_dvc_dq = cdata.dv1_dq;
          contact_dac_dq = cdata.da1_dq;
          contact_dac_dv = cdata.da1_dv;
          contact_dac_da = cdata.da1_da;
        }

        if (cmodel.joint2_id > 0)
        {
          cdata.dv2_dq.setZero();
          cdata.da2_dq.setZero();
          cdata.da2_dv.setZero();
          cdata.da2_da.setZero();
          const SE3 joint2_M_c1 = cmodel.joint2_placement * cdata.c1Mc2.inverse();

          getFrameAccelerationDerivatives(
            model, data, cmodel.joint2_id, joint2_M_c1,
            //                                            cmodel.joint2_placement,
            cmodel.reference_frame, cdata.dv2_dq, cdata.da2_dq, cdata.da2_dv, cdata.da2_da);

          // TODO: This is only in case reference_frame is LOCAL
          // TODO(jcarpent):to do colwise
          contact_dvc_dq -= cdata.dv2_dq;
          contact_dac_dq -= cdata.da2_dq;
          contact_dac_dv -= cdata.da2_dv;
          contact_dac_da -= cdata.da2_da;

          const Motion v2_in_c1 = cdata.c1Mc2.act(cdata.contact2_velocity);
          const Motion a2_in_c1 = cdata.oMc1.actInv(data.oa[cmodel.joint2_id]);

          Eigen::DenseIndex k = Eigen::DenseIndex(cmodel.colwise_span_indexes.size()) - 1;
          Eigen::DenseIndex col_id(0);
          while (cmodel.reference_frame == LOCAL && cmodel.colwise_span_indexes.size() > 0)
          {
            if (k >= 0)
            {
              col_id = cmodel.colwise_span_indexes[size_t(k)];
              k--;
            }
            else
            {
              col_id = data.parents_fromRow[size_t(col_id)];
              if (col_id < 0)
                break;
            }

            const MotionRef<typename RowsBlock::ColXpr> dvc_dv_col(contact_dac_da.col(col_id));
            const MotionRef<typename RigidConstraintData::Matrix6x::ColXpr> da2_da_col(
              cdata.da2_da.col(col_id));
            const MotionRef<typename RigidConstraintData::Matrix6x::ColXpr> dv2_dq_col(
              cdata.dv2_dq.col(col_id));

            // dv/dq
            const Motion v2_in_c1_cross_dvc_dv_col = v2_in_c1.cross(dvc_dv_col);
            contact_dvc_dq.col(col_id) -= v2_in_c1_cross_dvc_dv_col.toVector();

            // da/dv
            contact_dac_dv.col(col_id) -= v2_in_c1_cross_dvc_dv_col.toVector();
            contact_dac_dv.col(col_id) += cdata.contact_velocity_error.cross(da2_da_col).toVector();

            // da/dq
            const MotionRef<typename RowsBlock::ColXpr> dvc_dq_col(contact_dvc_dq.col(col_id));

            contact_dac_dq.col(col_id) -= a2_in_c1.cross(dvc_dv_col).toVector();
            contact_dac_dq.col(col_id) -= v2_in_c1.cross(dvc_dq_col).toVector();
            contact_dac_dq.col(col_id) +=
              cdata.contact_velocity_error.cross(v2_in_c1_cross_dvc_dv_col + dv2_dq_col).toVector();
          }

          cdata.dvc_dq = contact_dvc_dq;
          cdata.dac_dq = contact_dac_dq;
          cdata.dac_dv = contact_dac_dv;
          cdata.dac_da = contact_dac_da;
        }

        break;
      }
      case CONTACT_3D: {
        typedef
          typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;

        RowsBlock contact_dvc_dq = SizeDepType<3>::middleRows(data.dvc_dq, current_row_sol_id);
        RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq, current_row_sol_id);
        RowsBlock contact_dac_dv = SizeDepType<3>::middleRows(data.dac_dv, current_row_sol_id);
        RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da, current_row_sol_id);

        if (cmodel.joint1_id > 0)
        {
          cdata.dv1_dq.setZero();
          cdata.da1_dq.setZero();
          cdata.da1_dv.setZero();
          cdata.da1_da.setZero();

          getPointClassicAccelerationDerivatives(
            model, data, cmodel.joint1_id, cmodel.joint1_placement, cmodel.reference_frame,
            cdata.dv1_dq.template bottomRows<3>(), cdata.da1_dq.template bottomRows<3>(),
            cdata.da1_dv.template bottomRows<3>(), cdata.da1_da.template bottomRows<3>());

          contact_dvc_dq = cdata.dv1_dq.template bottomRows<3>();
          contact_dac_dq = cdata.da1_dq.template bottomRows<3>();
          contact_dac_dv = cdata.da1_dv.template bottomRows<3>();
          contact_dac_da = cdata.da1_da.template bottomRows<3>();
        }

        if (cmodel.joint2_id > 0)
        {
          cdata.dv2_dq.setZero();
          cdata.da2_dq.setZero();
          cdata.da2_dv.setZero();
          cdata.da2_da.setZero();
          const SE3 joint2_M_c1(
            cmodel.joint2_placement.rotation() * cdata.c1Mc2.rotation().transpose(),
            cmodel.joint2_placement.translation());

          getPointClassicAccelerationDerivatives(
            model, data, cmodel.joint2_id, joint2_M_c1, cmodel.reference_frame,
            cdata.dv2_dq.template bottomRows<3>(), cdata.da2_dq.template bottomRows<3>(),
            cdata.da2_dv.template bottomRows<3>(), cdata.da2_da.template bottomRows<3>());

          // TODO: This is only in case reference_frame is LOCAL
          contact_dvc_dq -= cdata.dv2_dq.template bottomRows<3>();
          contact_dac_dq -= cdata.da2_dq.template bottomRows<3>();
          contact_dac_dv -= cdata.da2_dv.template bottomRows<3>();
          contact_dac_da -= cdata.da2_da.template bottomRows<3>();
        }
        break;
      }
      default:
        assert(false && "must never happen");
        break;
      }

      assert(
        cmodel.loop_span_indexes.size() > 0
        && "Must never happened, the sparsity pattern is empty");
      // Derivative of closed loop kinematic tree
      if (cmodel.joint2_id > 0)
      {
        switch (cmodel.type)
        {
        case CONTACT_6D: {
          // TODO: THIS IS FOR THE LOCAL FRAME ONLY
          const typename Model::JointIndex joint2_id = cmodel.joint2_id;
          const Eigen::DenseIndex colRef2 =
            nv(model.joints[joint2_id]) + idx_v(model.joints[joint2_id]) - 1;

          Force contact_force_in_WORLD;
          switch (cmodel.reference_frame)
          {
          case LOCAL: {
            contact_force_in_WORLD = cdata.oMc1.act(cdata.contact_force);
            break;
          }
          case LOCAL_WORLD_ALIGNED: {
            contact_force_in_WORLD = cdata.contact_force;
            contact_force_in_WORLD.angular().noalias() +=
              cdata.oMc1.translation().cross(cdata.contact_force.linear());
            break;
          }
          default: {
            assert(false && "must never happen");
            break;
          }
          }

          // d./dq
          for (Eigen::DenseIndex k = 0; k < Eigen::DenseIndex(cmodel.colwise_span_indexes.size());
               ++k)
          {
            const Eigen::DenseIndex col_id = cmodel.colwise_span_indexes[size_t(k)];

            const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(col_id));
            const Force J_col_cross_contact_force_in_WORLD = J_col.cross(contact_force_in_WORLD);
            for (Eigen::DenseIndex j = colRef2; j >= 0; j = data.parents_fromRow[(size_t)j])
            {
              if (joint2_indexes[col_id])
              {
                data.dtau_dq(j, col_id) -=
                  data.J.col(j).dot(J_col_cross_contact_force_in_WORLD.toVector());
              }
              else
              {
                data.dtau_dq(j, col_id) +=
                  data.J.col(j).dot(J_col_cross_contact_force_in_WORLD.toVector());
              }
            }
          }
          break;
        }
        case CONTACT_3D: {

          typedef
            typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
          RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq, current_row_sol_id);
          const typename Model::JointIndex joint2_id = cmodel.joint2_id;
          const Eigen::DenseIndex colRef2 =
            nv(model.joints[joint2_id]) + idx_v(model.joints[joint2_id]) - 1;

          Force of_tmp, of_tmp2; // temporary Force variables

          switch (cmodel.reference_frame)
          {
          case LOCAL: {
            of_tmp.linear().noalias() = cdata.oMc1.rotation() * cdata.contact_force.linear();
            const Motion & c2_acc_c2 = getFrameClassicalAcceleration(
              model, data, cmodel.joint2_id, cmodel.joint2_placement, cmodel.reference_frame);
            a_tmp.angular().noalias() = cdata.oMc2.rotation() * c2_acc_c2.linear();
            break;
          }
          case LOCAL_WORLD_ALIGNED: {
            of_tmp.linear() = cdata.contact_force.linear();
            break;
          }
          default: {
            assert(false && "must never happen");
            break;
          }
          }

          // d./dq
          for (Eigen::DenseIndex k = 0; k < Eigen::DenseIndex(cmodel.loop_span_indexes.size()); ++k)
          {
            const Eigen::DenseIndex col_id = cmodel.loop_span_indexes[size_t(k)];

            const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(col_id));

            switch (cmodel.reference_frame)
            {
            case LOCAL: {
              a_tmp.linear().noalias() = a_tmp.angular().cross(J_col.angular());
              if (joint2_indexes[col_id])
              {
                contact_dac_dq.col(col_id).noalias() +=
                  cdata.oMc1.rotation().transpose() * a_tmp.linear();
              }
              else
              {
                contact_dac_dq.col(col_id).noalias() -=
                  cdata.oMc1.rotation().transpose() * a_tmp.linear();
              }
              break;
            }
            case LOCAL_WORLD_ALIGNED: {
              // Do nothing
              break;
            }
            default: {
              assert(false && "must never happen");
              break;
            }
            }

            of_tmp2.linear().noalias() = of_tmp.linear().cross(J_col.angular());
            for (Eigen::DenseIndex j = colRef2; j >= 0; j = data.parents_fromRow[(size_t)j])
            {
              const MotionRef<typename Data::Matrix6x::ColXpr> J2_col(data.J.col(j));

              // Temporary assignment
              of_tmp2.angular().noalias() =
                J2_col.linear() - cdata.oMc2.translation().cross(J2_col.angular());
              if (joint2_indexes[col_id])
              {
                data.dtau_dq(j, col_id) += of_tmp2.angular().dot(of_tmp2.linear());
              }
              else
              {
                data.dtau_dq(j, col_id) -= of_tmp2.angular().dot(of_tmp2.linear());
              }
            }
          }
          break;
        }
        default: {
          assert(false && "must never happen");
          break;
        }
        }
      }

      // Add the contribution of the corrector
      if (
        check_expression_if_real<Scalar>(!isZero(cmodel.corrector.Kp, static_cast<Scalar>(0.)))
        || check_expression_if_real<Scalar>(!isZero(cmodel.corrector.Kd, static_cast<Scalar>(0.))))
      {
        Jlog6(cdata.c1Mc2.inverse(), Jlog);

        switch (cmodel.type)
        {
        case CONTACT_6D: {
          typedef
            typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
          const RowsBlock contact_dvc_dq =
            SizeDepType<6>::middleRows(data.dvc_dq, current_row_sol_id);
          RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq, current_row_sol_id);
          RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv, current_row_sol_id);
          const RowsBlock contact_dac_da =
            SizeDepType<6>::middleRows(data.dac_da, current_row_sol_id);
          contact_dac_dq += cmodel.corrector.Kd.asDiagonal() * contact_dvc_dq;
          contact_dac_dv += cmodel.corrector.Kd.asDiagonal() * contact_dac_da;
          // d./dq
          for (Eigen::DenseIndex k = 0; k < Eigen::DenseIndex(cmodel.colwise_span_indexes.size());
               ++k)
          {
            const Eigen::DenseIndex row_id = cmodel.colwise_span_indexes[size_t(k)];
            // contact_dac_dq.col(row_id) += cmodel.corrector.Kd * contact_dvc_dq.col(row_id);
            contact_dac_dq.col(row_id).noalias() +=
              cmodel.corrector.Kp.asDiagonal() * Jlog * contact_dac_da.col(row_id);
          }
          break;
        }
        case CONTACT_3D: {
          typedef
            typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
          const RowsBlock contact_dvc_dq =
            SizeDepType<3>::middleRows(data.dvc_dq, current_row_sol_id);
          RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq, current_row_sol_id);
          RowsBlock contact_dac_dv = SizeDepType<3>::middleRows(data.dac_dv, current_row_sol_id);
          const RowsBlock contact_dac_da =
            SizeDepType<3>::middleRows(data.dac_da, current_row_sol_id);
          if (cmodel.reference_frame == LOCAL)
          {
            a_tmp.linear() = cmodel.corrector.Kd.asDiagonal() * cdata.oMc2.rotation()
                             * cdata.contact2_velocity.linear();
            typename SE3::Matrix3 vc2_cross_in_c1, vc2_cross_in_world;
            skew(a_tmp.linear(), vc2_cross_in_world);
            vc2_cross_in_c1.noalias() = cdata.oMc1.rotation().transpose() * vc2_cross_in_world;
            for (Eigen::DenseIndex k = 0; k < Eigen::DenseIndex(cmodel.loop_span_indexes.size());
                 ++k)
            {
              const Eigen::DenseIndex row_id = cmodel.loop_span_indexes[size_t(k)];
              const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(row_id));
              if (joint2_indexes[row_id])
              {
                contact_dac_dq.col(row_id).noalias() += vc2_cross_in_c1 * J_col.angular();
              }
              else
              {
                contact_dac_dq.col(row_id).noalias() -= vc2_cross_in_c1 * J_col.angular();
              }
            }
            const int colRef =
              nv(model.joints[cmodel.joint1_id]) + idx_v(model.joints[cmodel.joint1_id]) - 1;
            for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
            {
              typedef typename Data::Matrix6x::ColXpr ColType;
              const MotionRef<ColType> J_col(data.J.col(j));
              a_tmp.angular() = cdata.oMc1.rotation().transpose() * J_col.angular();
              contact_dac_dq.col(j).noalias() +=
                cmodel.corrector.Kp.asDiagonal()
                * cdata.contact_placement_error.linear().cross(a_tmp.angular());
            }
          }
          contact_dac_dq.noalias() += cmodel.corrector.Kd.asDiagonal() * contact_dvc_dq;
          contact_dac_dq.noalias() += cmodel.corrector.Kp.asDiagonal() * contact_dac_da;
          contact_dac_dv.noalias() += cmodel.corrector.Kd.asDiagonal() * contact_dac_da;
          break;
        }
        default:
          assert(false && "must never happen");
          break;
        }
      }

      current_row_sol_id += cmodel.size();
    }

    data.contact_chol.getOperationalSpaceInertiaMatrix(data.osim);
    data.contact_chol.getInverseMassMatrix(data.Minv);

    // Temporary: dlambda_dv stores J*Minv
    typename Data::MatrixXs & JMinv = data.dlambda_dv;

    JMinv.noalias() = data.dac_da * data.Minv;
    MatrixType3 & ddq_partial_dtau_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType3, ddq_partial_dtau);
    MatrixType6 & lambda_partial_dtau_ =
      PINOCCHIO_EIGEN_CONST_CAST(MatrixType6, lambda_partial_dtau);
    typename Data::MatrixXs & dlambda_dx_prox = data.dlambda_dx_prox;
    typename Data::MatrixXs & drhs_prox = data.drhs_prox;
    {
      lambda_partial_dtau_.noalias() = -data.osim * JMinv; // OUTPUT
      for (int it = 1; it < settings.iter; ++it)
      {
        lambda_partial_dtau_.swap(dlambda_dx_prox);
        dlambda_dx_prox *= settings.mu;
        dlambda_dx_prox -= JMinv;
        lambda_partial_dtau_.noalias() = data.osim * dlambda_dx_prox;
      }

      if (settings.iter % 2 == 0 && settings.iter > 0)
      {
        lambda_partial_dtau_.swap(dlambda_dx_prox); // restore previous memory address
        lambda_partial_dtau_ = dlambda_dx_prox;
      }
    }

    ddq_partial_dtau_.noalias() = JMinv.transpose() * lambda_partial_dtau;
    ddq_partial_dtau_ += data.Minv; // OUTPUT

    MatrixType4 & lambda_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType4, lambda_partial_dq);
    drhs_prox.noalias() = -JMinv * data.dtau_dq;
    drhs_prox += data.dac_dq;
    {
      lambda_partial_dq_.noalias() = -data.osim * data.drhs_prox; // OUTPUT
      for (int it = 1; it < settings.iter; ++it)
      {
        lambda_partial_dq_.swap(dlambda_dx_prox);
        dlambda_dx_prox *= settings.mu;
        dlambda_dx_prox -= drhs_prox;
        lambda_partial_dq_.noalias() = data.osim * dlambda_dx_prox;
      }

      if (settings.iter % 2 == 0 && settings.iter > 0)
      {
        lambda_partial_dq_.swap(dlambda_dx_prox); // restore previous memory address
        lambda_partial_dq_ = dlambda_dx_prox;
      }
    }

    MatrixType5 & lambda_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType5, lambda_partial_dv);
    drhs_prox.noalias() = -JMinv * data.dtau_dv;
    drhs_prox += data.dac_dv;
    {
      lambda_partial_dv_.noalias() = -data.osim * data.drhs_prox; // OUTPUT
      for (int it = 1; it < settings.iter; ++it)
      {
        lambda_partial_dv_.swap(dlambda_dx_prox);
        dlambda_dx_prox *= settings.mu;
        dlambda_dx_prox -= drhs_prox;
        lambda_partial_dv_.noalias() = data.osim * dlambda_dx_prox;
      }

      if (settings.iter % 2 == 0 && settings.iter > 0)
      {
        lambda_partial_dv_.swap(dlambda_dx_prox); // restore previous memory address
        lambda_partial_dv_ = dlambda_dx_prox;
      }
    }

    current_row_sol_id = 0;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];

      switch (cmodel.type)
      {
      case CONTACT_6D: {

        typedef
          typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::ConstType
          ConstRowsBlock;

        // TODO: replace with contact_model::nc
        RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da, current_row_sol_id);

        ConstRowsBlock contact_dlambda_dq =
          SizeDepType<6>::middleRows(lambda_partial_dq, current_row_sol_id);
        ConstRowsBlock contact_dlambda_dv =
          SizeDepType<6>::middleRows(lambda_partial_dv, current_row_sol_id);

        // TODO: Sparsity in dac_da with loop joints?

        data.dtau_dq.noalias() -= contact_dac_da.transpose() * contact_dlambda_dq;
        data.dtau_dv.noalias() -= contact_dac_da.transpose() * contact_dlambda_dv;

        // END TODO

        /*

         for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
         {
         data.dtau_dq.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dq;
         data.dtau_dv.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dv;
         }
         */
        break;
      }
      case CONTACT_3D: {

        typedef
          typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::ConstType
          ConstRowsBlock;

        RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da, current_row_sol_id);

        ConstRowsBlock contact_dlambda_dq =
          SizeDepType<3>::middleRows(lambda_partial_dq, current_row_sol_id);
        ConstRowsBlock contact_dlambda_dv =
          SizeDepType<3>::middleRows(lambda_partial_dv, current_row_sol_id);

        // TODO: Sparsity in dac_da with loop joints?

        data.dtau_dq.noalias() -= contact_dac_da.transpose() * contact_dlambda_dq;
        data.dtau_dv.noalias() -= contact_dac_da.transpose() * contact_dlambda_dv;

        // END TODO
        /*



         for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
         {
         data.dtau_dq.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dq;
         data.dtau_dv.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dv;
         }
         */
        break;
      }

      default:
        assert(false && "must never happen");
        break;
      }
      current_row_sol_id += cmodel.size();
    }

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType1, ddq_partial_dq).noalias() =
      -data.Minv * data.dtau_dq; // OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType2, ddq_partial_dv).noalias() =
      -data.Minv * data.dtau_dv; // OUTPUT

    MatrixType4 & dfc_dq = PINOCCHIO_EIGEN_CONST_CAST(MatrixType4, lambda_partial_dq);
    typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type Rows6Block;
    typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type Rows3Block;

    current_row_sol_id = 0;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      const RigidConstraintData & cdata = contact_data[k];
      const typename Model::JointIndex joint1_id = cmodel.joint1_id;
      const int colRef = nv(model.joints[joint1_id]) + idx_v(model.joints[joint1_id]) - 1;

      switch (cmodel.reference_frame)
      {
      case LOCAL:
        break;
      case LOCAL_WORLD_ALIGNED: {
        const Force & of = cdata.contact_force;
        switch (cmodel.type)
        {
        case CONTACT_6D: {
          Rows6Block contact_dfc_dq = SizeDepType<6>::middleRows(dfc_dq, current_row_sol_id);
          for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
            typedef typename Rows6Block::ColXpr ColTypeOut;
            const MotionRef<ColType> J_col(data.J.col(j));
            ForceRef<ColTypeOut> fout(contact_dfc_dq.col(j));
            fout.linear().noalias() += J_col.angular().cross(of.linear());
            fout.angular().noalias() += J_col.angular().cross(of.angular());
          }
          break;
        }
        case CONTACT_3D: {
          Rows3Block contact_dfc_dq = SizeDepType<3>::middleRows(dfc_dq, current_row_sol_id);
          for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
            const MotionRef<ColType> J_col(data.J.col(j));
            contact_dfc_dq.col(j).noalias() += J_col.angular().cross(of.linear());
          }
          break;
        }
        default:
          assert(false && "must never happen");
          break;
        }
        break;
      }
      default:
        assert(false && "must never happen");
        break;
      }
      current_row_sol_id += cmodel.size();
    }
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraint_dynamics_derivatives_hxx__
