//
// Copyright (c) 2017-2021 CNRS INRIA
//

#ifndef __pinocchio_algorithm_rnea_derivatives_hxx__
#define __pinocchio_algorithm_rnea_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  namespace impl
  {
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    struct ComputeGeneralizedGravityDerivativeForwardStep
    : public fusion::JointUnaryVisitorBase<ComputeGeneralizedGravityDerivativeForwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ConfigVectorType>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Motion Motion;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];
        const Motion & minus_gravity = data.oa_gf[0];

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];

        data.oYcrb[i] = data.oinertias[i] = data.oMi[i].act(model.inertias[i]);
        data.of[i] = data.oYcrb[i] * minus_gravity;

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock J_cols = jmodel.jointCols(data.J);
        ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
        J_cols = data.oMi[i].act(jdata.S());
        motionSet::motionAction(minus_gravity, J_cols, dAdq_cols);
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ReturnMatrixType>
    struct ComputeGeneralizedGravityDerivativeBackwardStep
    : public fusion::JointUnaryVisitorBase<ComputeGeneralizedGravityDerivativeBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ReturnMatrixType>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::
        vector<const Model &, Data &, typename Data::VectorXs &, ReturnMatrixType &>
          ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        Data & data,
        typename Data::VectorXs & g,
        const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef Eigen::Matrix<
          Scalar, JointModel::NV, 6, Options, JointModel::NV == Eigen::Dynamic ? 6 : JointModel::NV,
          6>
          MatrixNV6;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];

        typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) YS(jmodel.nv(), 6);

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;

        ColsBlock J_cols = jmodel.jointCols(data.J);
        ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
        ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
        ColsBlock Ag_cols = jmodel.jointCols(data.Ag);

        motionSet::inertiaAction(data.oYcrb[i], dAdq_cols, dFdq_cols);

        ReturnMatrixType & gravity_partial_dq_ =
          PINOCCHIO_EIGEN_CONST_CAST(ReturnMatrixType, gravity_partial_dq);
        gravity_partial_dq_.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i])
          .noalias() = J_cols.transpose() * data.dFdq.middleCols(jmodel.idx_v(), data.nvSubtree[i]);

        motionSet::act<ADDTO>(J_cols, data.of[i], dFdq_cols);

        motionSet::inertiaAction(data.oYcrb[i], J_cols, Ag_cols);
        for (int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()]; j >= 0;
             j = data.parents_fromRow[(typename Model::Index)j])
          gravity_partial_dq_.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() =
            Ag_cols.transpose() * data.dAdq.col(j);

        jmodel.jointVelocitySelector(g).noalias() = J_cols.transpose() * data.of[i].toVector();
        if (parent > 0)
        {
          data.oYcrb[parent] += data.oYcrb[i];
          data.of[parent] += data.of[i];
        }
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename ReturnMatrixType>
    void computeGeneralizedGravityDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(gravity_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(gravity_partial_dq.rows(), model.nv);
      assert(model.check(data) && "data is not consistent with model.");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.oa_gf[0] = -model.gravity; // minus_gravity used in the two Passes

      typedef ComputeGeneralizedGravityDerivativeForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
      }

      typedef ComputeGeneralizedGravityDerivativeBackwardStep<
        Scalar, Options, JointCollectionTpl, ReturnMatrixType>
        Pass2;
      ReturnMatrixType & gravity_partial_dq_ =
        PINOCCHIO_EIGEN_CONST_CAST(ReturnMatrixType, gravity_partial_dq);
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(
          model.joints[i], typename Pass2::ArgsType(model, data, data.g, gravity_partial_dq_));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename ReturnMatrixType>
    void computeStaticTorqueDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const container::aligned_vector<ForceTpl<Scalar, Options>> & fext,
      const Eigen::MatrixBase<ReturnMatrixType> & static_torque_partial_dq)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(static_torque_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(static_torque_partial_dq.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        fext.size(), (size_t)model.njoints, "The size of the external forces is not of right size");
      assert(model.check(data) && "data is not consistent with model.");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.oa_gf[0] = -model.gravity; // minus_gravity used in the two Passes

      typedef ComputeGeneralizedGravityDerivativeForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
        data.of[i] -= data.oMi[i].act(fext[i]);
      }

      typedef ComputeGeneralizedGravityDerivativeBackwardStep<
        Scalar, Options, JointCollectionTpl, ReturnMatrixType>
        Pass2;
      ReturnMatrixType & static_torque_partial_dq_ =
        PINOCCHIO_EIGEN_CONST_CAST(ReturnMatrixType, static_torque_partial_dq);
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(
          model.joints[i],
          typename Pass2::ArgsType(model, data, data.tau, static_torque_partial_dq_));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    struct ComputeRNEADerivativesForwardStep
    : public fusion::JointUnaryVisitorBase<ComputeRNEADerivativesForwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ConfigVectorType,
        TangentVectorType1,
        TangentVectorType2>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<
        const Model &,
        Data &,
        const ConfigVectorType &,
        const TangentVectorType1 &,
        const TangentVectorType2 &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & a)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Motion Motion;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];
        Motion & ov = data.ov[i];
        Motion & oa = data.oa[i];
        Motion & oa_gf = data.oa_gf[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        data.v[i] = jdata.v();

        if (parent > 0)
        {
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
          data.v[i] += data.liMi[i].actInv(data.v[parent]);
        }
        else
          data.oMi[i] = data.liMi[i];

        data.a[i] =
          jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
        if (parent > 0)
        {
          data.a[i] += data.liMi[i].actInv(data.a[parent]);
        }

        data.oYcrb[i] = data.oinertias[i] = data.oMi[i].act(model.inertias[i]);
        ov = data.oMi[i].act(data.v[i]);
        oa = data.oMi[i].act(data.a[i]);
        oa_gf = oa - model.gravity; // add gravity contribution

        data.oh[i] = data.oYcrb[i] * ov;
        data.of[i] = data.oYcrb[i] * oa_gf + ov.cross(data.oh[i]);

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock J_cols = jmodel.jointCols(data.J);
        ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
        ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
        ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);

        J_cols = data.oMi[i].act(jdata.S());
        motionSet::motionAction(ov, J_cols, dJ_cols);
        motionSet::motionAction(data.oa_gf[parent], J_cols, dAdq_cols);
        dAdv_cols = dJ_cols;
        if (parent > 0)
        {
          motionSet::motionAction(data.ov[parent], J_cols, dVdq_cols);
          motionSet::motionAction<ADDTO>(data.ov[parent], dVdq_cols, dAdq_cols);
          dAdv_cols.noalias() += dVdq_cols;
        }
        else
        {
          dVdq_cols.setZero();
        }

        // computes variation of inertias
        data.doYcrb[i] = data.oYcrb[i].variation(ov);

        addForceCrossMatrix(data.oh[i], data.doYcrb[i]);
      }

      template<typename ForceDerived, typename M6>
      static void
      addForceCrossMatrix(const ForceDense<ForceDerived> & f, const Eigen::MatrixBase<M6> & mout)
      {
        M6 & mout_ = PINOCCHIO_EIGEN_CONST_CAST(M6, mout);
        addSkew(
          -f.linear(), mout_.template block<3, 3>(ForceDerived::LINEAR, ForceDerived::ANGULAR));
        addSkew(
          -f.linear(), mout_.template block<3, 3>(ForceDerived::ANGULAR, ForceDerived::LINEAR));
        addSkew(
          -f.angular(), mout_.template block<3, 3>(ForceDerived::ANGULAR, ForceDerived::ANGULAR));
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename MatrixType1,
      typename MatrixType2,
      typename MatrixType3>
    struct ComputeRNEADerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<ComputeRNEADerivativesBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        MatrixType1,
        MatrixType2,
        MatrixType3>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::
        vector<const Model &, Data &, const MatrixType1 &, const MatrixType2 &, const MatrixType3 &>
          ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
        const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
        const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Matrix6x Matrix6x;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColsBlock;

        Matrix6x & dYtJ = data.Fcrb[0];
        ColsBlock J_cols = jmodel.jointCols(data.J);
        ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
        ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
        ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
        ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv);
        ColsBlock dFda_cols = jmodel.jointCols(data.dFda); // Also equals to Ag_cols
        ColsBlock dYtJ_cols = jmodel.jointCols(dYtJ);

        MatrixType1 & rnea_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType1, rnea_partial_dq);
        MatrixType2 & rnea_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType2, rnea_partial_dv);
        MatrixType3 & rnea_partial_da_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType3, rnea_partial_da);

        // tau
        jmodel.jointVelocitySelector(data.tau).noalias() =
          J_cols.transpose() * data.of[i].toVector();

        const Eigen::DenseIndex nv_subtree = data.nvSubtree[i];
        const Eigen::DenseIndex nv = jmodel.nv();
        const Eigen::DenseIndex idx_v = jmodel.idx_v();
        const Eigen::DenseIndex idx_v_plus = idx_v + nv;
        const Eigen::DenseIndex nv_subtree_plus = nv_subtree - nv;

        // dtau/da similar to data.M
        motionSet::inertiaAction(data.oYcrb[i], J_cols, dFda_cols);
        rnea_partial_da_.block(idx_v, idx_v, nv, nv_subtree).noalias() =
          J_cols.transpose() * data.dFda.middleCols(idx_v, nv_subtree);

        // dtau/dq
        if (parent > 0)
        {
          dFdq_cols.noalias() = data.doYcrb[i] * dVdq_cols;
          motionSet::inertiaAction<ADDTO>(data.oYcrb[i], dAdq_cols, dFdq_cols);
        }
        else
          motionSet::inertiaAction(data.oYcrb[i], dAdq_cols, dFdq_cols);

        dYtJ_cols.transpose().noalias() = J_cols.transpose() * data.doYcrb[i];
        rnea_partial_dq_.block(idx_v_plus, idx_v, nv_subtree_plus, nv).noalias() =
          data.dFda.middleCols(idx_v_plus, nv_subtree_plus).transpose() * dAdq_cols
          + dYtJ.middleCols(idx_v_plus, nv_subtree_plus).transpose() * dVdq_cols;

        rnea_partial_dq_.block(idx_v, idx_v, nv, nv_subtree).noalias() =
          J_cols.transpose() * data.dFdq.middleCols(idx_v, nv_subtree);

        motionSet::act<ADDTO>(J_cols, data.of[i], dFdq_cols);

        // dtau/dv
        dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
        motionSet::inertiaAction<ADDTO>(data.oYcrb[i], dAdv_cols, dFdv_cols);

        rnea_partial_dv_.block(idx_v_plus, idx_v, nv_subtree_plus, nv).noalias() =
          data.dFda.middleCols(idx_v_plus, nv_subtree_plus).transpose() * dAdv_cols
          + dYtJ.middleCols(idx_v_plus, nv_subtree_plus).transpose() * J_cols;

        rnea_partial_dv_.block(idx_v, idx_v, nv, nv_subtree).noalias() =
          J_cols.transpose() * data.dFdv.middleCols(idx_v, nv_subtree);

        if (parent > 0)
        {
          data.oYcrb[parent] += data.oYcrb[i];
          data.doYcrb[parent] += data.doYcrb[i];
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
      typename MatrixType1,
      typename MatrixType2,
      typename MatrixType3>
    void computeRNEADerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a,
      const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
      const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
      const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The joint configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        v.size(), model.nv, "The joint velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        a.size(), model.nv, "The joint acceleration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dq.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dv.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dv.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_da.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_da.rows(), model.nv);
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        isZero(model.gravity.angular()),
        "The gravity must be a pure force vector, no angular part");
      assert(model.check(data) && "data is not consistent with model.");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef typename Model::JointIndex JointIndex;

      data.oa_gf[0] = -model.gravity;

      typedef ComputeRNEADerivativesForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i],
          typename Pass1::ArgsType(model, data, q.derived(), v.derived(), a.derived()));
      }

      typedef ComputeRNEADerivativesBackwardStep<
        Scalar, Options, JointCollectionTpl, MatrixType1, MatrixType2, MatrixType3>
        Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(
          model.joints[i], typename Pass2::ArgsType(
                             model, data, PINOCCHIO_EIGEN_CONST_CAST(MatrixType1, rnea_partial_dq),
                             PINOCCHIO_EIGEN_CONST_CAST(MatrixType2, rnea_partial_dv),
                             PINOCCHIO_EIGEN_CONST_CAST(MatrixType3, rnea_partial_da)));
      }

      // Restore the status of dAdq_cols (remove gravity)
      for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
      {
        MotionRef<typename Data::Matrix6x::ColXpr> m_in(data.J.col(k));
        MotionRef<typename Data::Matrix6x::ColXpr> m_out(data.dAdq.col(k));
        m_out.linear() += model.gravity.linear().cross(m_in.angular());
      }

      // Add armature contribution
      data.tau.array() +=
        model.armature.array() * a.array(); // TODO: check if there is memory allocation
      PINOCCHIO_EIGEN_CONST_CAST(MatrixType3, rnea_partial_da).diagonal() += model.armature;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2,
      typename MatrixType1,
      typename MatrixType2,
      typename MatrixType3>
    void computeRNEADerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a,
      const container::aligned_vector<ForceTpl<Scalar, Options>> & fext,
      const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
      const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
      const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The joint configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        v.size(), model.nv, "The joint velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        a.size(), model.nv, "The joint acceleration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        fext.size(), (size_t)model.njoints, "The size of the external forces is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dq.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dv.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_dv.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_da.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(rnea_partial_da.rows(), model.nv);
      assert(model.check(data) && "data is not consistent with model.");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef typename Model::JointIndex JointIndex;

      data.oa_gf[0] = -model.gravity;

      typedef ComputeRNEADerivativesForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i],
          typename Pass1::ArgsType(model, data, q.derived(), v.derived(), a.derived()));
        data.of[i] -= data.oMi[i].act(fext[i]);
      }

      typedef ComputeRNEADerivativesBackwardStep<
        Scalar, Options, JointCollectionTpl, MatrixType1, MatrixType2, MatrixType3>
        Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(
          model.joints[i], typename Pass2::ArgsType(
                             model, data, PINOCCHIO_EIGEN_CONST_CAST(MatrixType1, rnea_partial_dq),
                             PINOCCHIO_EIGEN_CONST_CAST(MatrixType2, rnea_partial_dv),
                             PINOCCHIO_EIGEN_CONST_CAST(MatrixType3, rnea_partial_da)));
      }

      // Restore the status of dAdq_cols (remove gravity)
      for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
      {
        MotionRef<typename Data::Matrix6x::ColXpr> m_in(data.J.col(k));
        MotionRef<typename Data::Matrix6x::ColXpr> m_out(data.dAdq.col(k));
        m_out.linear() += model.gravity.linear().cross(m_in.angular());
      }

      // Add armature contribution
      data.tau.array() +=
        model.armature.array() * a.array(); // TODO: check if there is memory allocation
      data.M.diagonal() += model.armature;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    void computeRNEADerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a)
    {
      impl::computeRNEADerivatives(
        model, data, q.derived(), v.derived(), a.derived(), data.dtau_dq, data.dtau_dv, data.M);
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    void computeRNEADerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a,
      const container::aligned_vector<ForceTpl<Scalar, Options>> & fext)
    {
      impl::computeRNEADerivatives(
        model, data, q.derived(), v.derived(), a.derived(), fext, data.dtau_dq, data.dtau_dv,
        data.M);
    }

  } // namespace impl

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename ReturnMatrixType>
  void computeGeneralizedGravityDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq)
  {
    impl::computeGeneralizedGravityDerivatives(
      model, data, make_const_ref(q), make_ref(gravity_partial_dq));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename ReturnMatrixType>
  void computeStaticTorqueDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext,
    const Eigen::MatrixBase<ReturnMatrixType> & static_torque_partial_dq)
  {
    impl::computeStaticTorqueDerivatives(
      model, data, make_const_ref(q), fext, make_ref(static_torque_partial_dq));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3>
  void computeRNEADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a,
    const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
  {
    impl::computeRNEADerivatives(
      model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a),
      make_ref(rnea_partial_dq), make_ref(rnea_partial_dv), make_ref(rnea_partial_da));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3>
  void computeRNEADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext,
    const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
  {
    impl::computeRNEADerivatives(
      model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a), fext,
      make_ref(rnea_partial_dq), make_ref(rnea_partial_dv), make_ref(rnea_partial_da));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  void computeRNEADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    impl::computeRNEADerivatives(
      model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a), make_ref(data.dtau_dq),
      make_ref(data.dtau_dv), make_ref(data.M));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  void computeRNEADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext)
  {
    impl::computeRNEADerivatives(
      model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a), fext,
      make_ref(data.dtau_dq), make_ref(data.dtau_dv), make_ref(data.M));
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_rnea_derivatives_hxx__
