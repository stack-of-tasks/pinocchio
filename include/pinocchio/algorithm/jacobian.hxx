//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_algorithm_jacobian_hxx__
#define __pinocchio_algorithm_jacobian_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace pinocchio
{
  namespace impl
  {
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename Matrix6xLike>
    struct JointJacobiansForwardStep
    : public fusion::JointUnaryVisitorBase<JointJacobiansForwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ConfigVectorType,
        Matrix6xLike>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &, Matrix6xLike &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];

        Matrix6xLike & J_ = J.const_cast_derived();
        jmodel.jointExtendedModelCols(J_) = data.oMi[i].act(jdata.S());
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::Matrix6x & computeJointJacobians(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Matrix6x Matrix6x;

      typedef JointJacobiansForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, Matrix6x>
        Pass;
      typedef typename Pass::ArgsType ArgsType;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass::run(
          model.joints[i], data.joints[i],
          ArgsType(model, data, q.derived(), data.J.const_cast_derived()));
      }

      return data.J;
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct JointJacobiansForwardStep2
    : public fusion::JointUnaryVisitorBase<
        JointJacobiansForwardStep2<Scalar, Options, JointCollectionTpl>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<Data &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        Data & data)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        jmodel.jointExtendedModelCols(data.J) = data.oMi[i].act(jdata.S());
      }
    };
  } // namespace impl
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::Matrix6x & computeJointJacobians(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    typedef impl::JointJacobiansForwardStep2<Scalar, Options, JointCollectionTpl> Pass;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass::run(model.joints[i], data.joints[i], typename Pass::ArgsType(data));
    }

    return data.J;
  }

  namespace details
  {
    template<typename Scalar, int Options, typename Matrix6xLikeIn, typename Matrix6xLikeOut>
    void translateJointJacobian(
      const SE3Tpl<Scalar, Options> & placement,
      const Eigen::MatrixBase<Matrix6xLikeIn> & Jin,
      const Eigen::MatrixBase<Matrix6xLikeOut> & Jout)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jin.rows(), 6);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jin.cols(), Jout.cols());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jout.rows(), 6);

      Matrix6xLikeOut & Jout_ = Jout.const_cast_derived();

      typedef typename Matrix6xLikeIn::ConstColXpr ConstColXprIn;
      typedef const MotionRef<ConstColXprIn> MotionIn;

      typedef typename Matrix6xLikeOut::ColXpr ColXprOut;
      typedef MotionRef<ColXprOut> MotionOut;

      for (Eigen::DenseIndex j = 0; j < Jin.cols(); ++j)
      {
        MotionIn v_in(Jin.col(j));
        MotionOut v_out(Jout_.col(j));

        v_out = v_in;
        v_out.linear() -= placement.translation().cross(v_in.angular());
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLikeIn,
      typename Matrix6xLikeOut>
    void translateJointJacobian(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex joint_id,
      const ReferenceFrame rf,
      const SE3Tpl<Scalar, Options> & placement,
      const Eigen::MatrixBase<Matrix6xLikeIn> & Jin,
      const Eigen::MatrixBase<Matrix6xLikeOut> & Jout)
    {
      assert(model.check(data) && "data is not consistent with model.");

      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jin.rows(), 6);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jin.cols(), model.nvExtended);

      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jout.rows(), 6);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jout.cols(), model.nv);

      Matrix6xLikeOut & Jout_ = Jout.const_cast_derived();

      typedef typename Matrix6xLikeIn::ConstColXpr ConstColXprIn;
      typedef const MotionRef<ConstColXprIn> MotionIn;

      typedef typename Matrix6xLikeOut::ColXpr ColXprOut;
      typedef MotionRef<ColXprOut> MotionOut;

      const bool is_joint_mimic = (model.mimic_joint_supports[joint_id].back() == joint_id);
      const int joint_first_col = model.idx_vExtendeds[joint_id];
      const int joint_last_col = model.idx_vExtendeds[joint_id] + model.nvExtendeds[joint_id] - 1;

      // If the current joint is mimic, start the first pass (on non mimic joints) at the first non
      // mimic parent Else if the joint is not a mimic, include the current joint in the first pass
      const int colRef =
        is_joint_mimic ? data.non_mimic_parents_fromRow[(size_t)joint_first_col] : joint_last_col;

      // If the current joint is mimic, start the second pass (on mimic joints) at the current joint
      // Else if the joint is not a mimic, start the second pass (on mimic joints) at the first
      // mimic parent
      const int colRefMimicPass =
        is_joint_mimic ? joint_last_col : data.mimic_parents_fromRow[(size_t)joint_first_col];

      switch (rf)
      {
      case WORLD: {
        for (Eigen::DenseIndex jExtended = colRef; jExtended >= 0;
             jExtended = data.non_mimic_parents_fromRow[(size_t)jExtended])
        {
          MotionIn v_in(Jin.col(jExtended));
          MotionOut v_out(Jout_.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out = v_in;
        }
        // Add mimicking joint effect into mimicked column
        for (Eigen::DenseIndex jExtended = colRefMimicPass; jExtended >= 0;
             jExtended = data.mimic_parents_fromRow[(size_t)jExtended])
        {
          MotionIn v_in(Jin.col(jExtended));
          MotionOut v_out(Jout_.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out += v_in;
        }
        break;
      }
      case LOCAL_WORLD_ALIGNED: {
        for (Eigen::DenseIndex jExtended = colRef; jExtended >= 0;
             jExtended = data.non_mimic_parents_fromRow[(size_t)jExtended])
        {
          MotionIn v_in(Jin.col(jExtended));
          MotionOut v_out(Jout_.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out = v_in;
          v_out.linear().noalias() -= placement.translation().cross(v_in.angular());
        }
        // Add mimicking joint effect into mimicked column
        for (Eigen::DenseIndex jExtended = colRefMimicPass; jExtended >= 0;
             jExtended = data.mimic_parents_fromRow[(size_t)jExtended])
        {
          MotionIn v_in(Jin.col(jExtended));
          MotionOut v_out(Jout_.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out += v_in;
          v_out.linear().noalias() -= placement.translation().cross(v_in.angular());
        }
        break;
      }
      case LOCAL: {
        for (Eigen::DenseIndex jExtended = colRef; jExtended >= 0;
             jExtended = data.non_mimic_parents_fromRow[(size_t)jExtended])
        {
          MotionIn v_in(Jin.col(jExtended));
          MotionOut v_out(Jout_.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out = placement.actInv(v_in);
        }
        // Add mimicking joint effect into mimicked column
        for (Eigen::DenseIndex jExtended = colRefMimicPass; jExtended >= 0;
             jExtended = data.mimic_parents_fromRow[(size_t)jExtended])
        {
          MotionIn v_in(Jin.col(jExtended));
          MotionOut v_out(Jout_.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out += placement.actInv(v_in);
        }
        break;
      }
      default:
        PINOCCHIO_CHECK_INPUT_ARGUMENT(false, "must never happened");
        break;
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLikeIn,
      typename Matrix6xLikeOut>
    void translateJointJacobian(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex joint_id,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix6xLikeIn> & Jin,
      const Eigen::MatrixBase<Matrix6xLikeOut> & Jout)
    {
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      const typename Data::SE3 & oMjoint = data.oMi[joint_id];

      translateJointJacobian(model, data, joint_id, rf, oMjoint, Jin, Jout);
    }
  } // namespace details
  namespace impl
  {
    /* Return the jacobian of the output frame attached to joint <jointId> in the
     world frame or in the local frame depending on the template argument. The
     function computeJacobians should have been called first. */
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLike>
    void getJointJacobian(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const JointIndex joint_id,
      const ReferenceFrame reference_frame,
      const Eigen::MatrixBase<Matrix6xLike> & J)
    {
      assert(model.check(data) && "data is not consistent with model.");

      ::pinocchio::details::translateJointJacobian(
        model, data, joint_id, reference_frame, data.J, J.const_cast_derived());
    }

    /// Compute the minimal number of value to fill the WORLD jacobian for a particular joint.
    /// Fill liMi, oMi and J.
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename Matrix6xLike>
    struct JointJacobianWorldForwardStep
    : public fusion::JointUnaryVisitorBase<JointJacobianWorldForwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ConfigVectorType,
        Matrix6xLike>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &, Matrix6xLike &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        algo_impl(jmodel, jdata, model, data, q, J);
      }

      template<typename JointModel>
      static void algo_impl(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];

        Matrix6xLike & J_ = J.const_cast_derived();
        jmodel.jointCols(J_) = data.oMi[i].act(jdata.S());
      }

      // Mimic specialization: We don't fill the jacobian, this will be done in
      // JointJacobianWorldMimicStep
      static void algo_impl(
        const JointModelBase<JointModelMimic> & jmodel,
        JointDataBase<typename JointModelMimic::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> &)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];
      }
    };

    /// Modify the jacobian to add mimic joint effect
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLike>
    struct JointJacobianWorldMimicStep
    : public fusion::JointUnaryVisitorBase<
        JointJacobianWorldMimicStep<Scalar, Options, JointCollectionTpl, Matrix6xLike>>
    {
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<const Data &, Matrix6xLike &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Data & data,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;
        const JointIndex & i = jmodel.id();
        Matrix6xLike & J_ = J.const_cast_derived();

        jmodel.jointCols(J_) += data.oMi[i].act(jdata.S());
      }
    };

    /// Compute the minimal number of value to fill the LOCAL_WORLD_ALIGNED jacobian for a
    /// particular joint.
    /// Need oMi filled.
    /// Fill J.
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLike>
    struct JointJacobianLocalWorldAlignedForwardStep
    : public fusion::JointUnaryVisitorBase<JointJacobianLocalWorldAlignedForwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        Matrix6xLike>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef SE3Tpl<Scalar, Options> SE3;
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Data &, const SE3 &, Matrix6xLike &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Data & data,
        const SE3 & oMf,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        algo_impl(jmodel, jdata, data, oMf, J);
      }

      template<typename JointModel>
      static void algo_impl(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Data & data,
        const SE3 & oMf,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();

        Matrix6xLike & J_ = J.const_cast_derived();
        auto placement = data.oMi[i];
        placement.translation().noalias() -= oMf.translation();
        jmodel.jointCols(J_) = placement.act(jdata.S());
      }

      // Mimic specialization: We apply mimicking joint effect of mimicked column.
      static void algo_impl(
        const JointModelBase<JointModelMimic> & jmodel,
        JointDataBase<typename JointModelMimic::JointDataDerived> & jdata,
        const Data & data,
        const SE3 & oMf,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();

        Matrix6xLike & J_ = J.const_cast_derived();
        auto placement = data.oMi[i];
        placement.translation().noalias() -= oMf.translation();
        jmodel.jointCols(J_) += placement.act(jdata.S());
      }
    };

    /// Compute the minimal number of value to fill the LOCAL jacobian for a particular joint.
    /// Fill liMi, iMf and J.
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename Matrix6xLike>
    struct JointJacobianLocalBackwardStep
    : public fusion::JointUnaryVisitorBase<JointJacobianLocalBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ConfigVectorType,
        Matrix6xLike>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &, Matrix6xLike &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        algo_impl(jmodel, jdata, model, data, q, J);
      }

      template<typename JointModel>
      static void algo_impl(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;
        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];
        Matrix6xLike & J_ = J.const_cast_derived();

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        data.iMf[parent] = data.liMi[i] * data.iMf[i];
        jmodel.jointCols(J_) = data.iMf[i].actInv(jdata.S());
      }

      // Mimic specialization: We don't fill the jacobian, this will be done in
      // JointJacobianLocalMimicStep
      static void algo_impl(
        const JointModelBase<JointModelMimic> & jmodel,
        JointDataBase<typename JointModelMimic::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<Matrix6xLike> &)
      {
        typedef typename Model::JointIndex JointIndex;
        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        data.iMf[parent] = data.liMi[i] * data.iMf[i];
      }
    };

    /// Modify the jacobian to add mimic joint effect
    /// Modify J
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLike>
    struct JointJacobianLocalMimicStep
    : public fusion::JointUnaryVisitorBase<
        JointJacobianLocalMimicStep<Scalar, Options, JointCollectionTpl, Matrix6xLike>>
    {
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<const Data &, Matrix6xLike &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Data & data,
        const Eigen::MatrixBase<Matrix6xLike> & J)
      {
        typedef typename Model::JointIndex JointIndex;
        const JointIndex & i = jmodel.id();
        Matrix6xLike & J_ = J.const_cast_derived();

        jmodel.jointCols(J_) += data.iMf[i].actInv(jdata.S());
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename Matrix6xLike>
    void computeJointJacobian(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const JointIndex jointId,
      const Eigen::MatrixBase<Matrix6xLike> & J)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.iMf[jointId].setIdentity();
      Matrix6xLike & J_ = J.const_cast_derived();
      typedef JointJacobianLocalBackwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, Matrix6xLike>
        BackwardPass;
      typedef JointJacobianLocalMimicStep<Scalar, Options, JointCollectionTpl, Matrix6xLike>
        MimicPass;

      // Fill the jacobian for normal joints
      for (JointIndex i = jointId; i > 0; i = model.parents[i])
      {
        BackwardPass::run(
          model.joints[i], data.joints[i],
          typename BackwardPass::ArgsType(model, data, q.derived(), J_));
      }

      // Patch the jacobian with mimic joint effect
      const typename Model::IndexVector & mimic_joint_support = model.mimic_joint_supports[jointId];
      for (size_t i = 1; i < mimic_joint_support.size(); i++)
      {
        MimicPass::run(
          model.joints[mimic_joint_support[i]], data.joints[mimic_joint_support[i]],
          typename MimicPass::ArgsType(data, J_));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    struct JointJacobiansTimeVariationForwardStep
    : public fusion::JointUnaryVisitorBase<JointJacobiansTimeVariationForwardStep<
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
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<TangentVectorType> & v)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::SE3 SE3;
        typedef typename Data::Motion Motion;

        const JointIndex & i = (JointIndex)jmodel.id();
        const JointIndex & parent = model.parents[i];

        SE3 & oMi = data.oMi[i];
        Motion & vJ = data.v[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        vJ = jdata.v();

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        if (parent > 0)
        {
          oMi = data.oMi[parent] * data.liMi[i];
          vJ += data.liMi[i].actInv(data.v[parent]);
        }
        else
        {
          oMi = data.liMi[i];
        }

        jmodel.jointExtendedModelCols(data.J) = oMi.act(jdata.S());

        // Spatial velocity of joint i expressed in the global frame o
        data.ov[i] = oMi.act(vJ);

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock dJcols = jmodel.jointExtendedModelCols(data.dJ);
        ColsBlock Jcols = jmodel.jointExtendedModelCols(data.J);

        motionSet::motionAction(data.ov[i], Jcols, dJcols);
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::Matrix6x &
    computeJointJacobiansTimeVariation(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The velocity vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      typedef JointJacobiansTimeVariationForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>
        Pass;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass::run(
          model.joints[i], data.joints[i],
          typename Pass::ArgsType(model, data, q.derived(), v.derived()));
      }

      return data.dJ;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xLike>
    void getJointJacobianTimeVariation(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const JointIndex jointId,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix6xLike> & dJ_)
    {
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef typename Data::SE3 SE3;
      typedef typename SE3::Vector3 Vector3;
      typedef typename Data::Motion Motion;

      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        jointId < JointIndex(model.njoints)
        && "jointId is larger than the number of joints contained in the model");

      Matrix6xLike & dJ = dJ_.const_cast_derived();
      ::pinocchio::details::translateJointJacobian(model, data, jointId, rf, data.dJ, dJ);

      // Add contribution for LOCAL and LOCAL_WORLD_ALIGNED
      switch (rf)
      {
      case LOCAL: {
        const SE3 & oMjoint = data.oMi[jointId];
        const Motion & v_joint = data.v[jointId];
        const int colRef = model.nvExtendeds[jointId] + model.idx_vExtendeds[jointId] - 1;
        for (Eigen::DenseIndex jExtended = colRef; jExtended >= 0;
             jExtended = data.parents_fromRow[(size_t)jExtended])
        {
          typedef typename Data::Matrix6x::ConstColXpr ConstColXprIn;
          typedef const MotionRef<ConstColXprIn> MotionIn;

          typedef typename Matrix6xLike::ColXpr ColXprOut;
          typedef MotionRef<ColXprOut> MotionOut;
          MotionIn v_in(data.J.col(jExtended));
          MotionOut v_out(dJ.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out -= v_joint.cross(oMjoint.actInv(v_in));
        }
        break;
      }
      case LOCAL_WORLD_ALIGNED: {
        const Motion & ov_joint = data.ov[jointId];
        const SE3 & oMjoint = data.oMi[jointId];
        const int colRef = model.nvExtendeds[jointId] + model.idx_vExtendeds[jointId] - 1;
        for (Eigen::DenseIndex jExtended = colRef; jExtended >= 0;
             jExtended = data.parents_fromRow[(size_t)jExtended])
        {
          typedef typename Data::Matrix6x::ConstColXpr ConstColXprIn;
          typedef const MotionRef<ConstColXprIn> MotionIn;

          typedef typename Matrix6xLike::ColXpr ColXprOut;
          typedef MotionRef<ColXprOut> MotionOut;
          MotionIn v_in(data.J.col(jExtended));
          MotionOut v_out(dJ.col(data.idx_vExtended_to_idx_v_fromRow[size_t(jExtended)]));

          v_out.linear() -=
            Vector3(ov_joint.linear() + ov_joint.angular().cross(oMjoint.translation()))
              .cross(v_in.angular());
        }
        break;
      }

      case WORLD:
      default:
        break;
      }
    }
  } // namespace impl

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::Matrix6x & computeJointJacobians(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    return impl::computeJointJacobians(model, data, make_const_ref(q));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6Like>
  void getJointJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const ReferenceFrame reference_frame,
    const Eigen::MatrixBase<Matrix6Like> & J)
  {
    impl::getJointJacobian(model, data, joint_id, reference_frame, make_ref(J));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename Matrix6Like>
  void computeJointJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const JointIndex joint_id,
    const Eigen::MatrixBase<Matrix6Like> & J)
  {
    impl::computeJointJacobian(model, data, make_const_ref(q), joint_id, make_ref(J));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::Matrix6x &
  computeJointJacobiansTimeVariation(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)

  {
    return impl::computeJointJacobiansTimeVariation(
      model, data, make_const_ref(q), make_const_ref(v));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6Like>
  void getJointJacobianTimeVariation(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const ReferenceFrame reference_frame,
    const Eigen::MatrixBase<Matrix6Like> & dJ)
  {
    impl::getJointJacobianTimeVariation(model, data, joint_id, reference_frame, make_ref(dJ));
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_algorithm_jacobian_hxx__
