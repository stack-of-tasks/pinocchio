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
        jmodel.jointCols(J_) = data.oMi[i].act(jdata.S());
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
        jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
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
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jin.cols(), model.nv);

      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jout.rows(), 6);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Jout.cols(), model.nv);

      Matrix6xLikeOut & Jout_ = Jout.const_cast_derived();

      typedef typename Matrix6xLikeIn::ConstColXpr ConstColXprIn;
      typedef const MotionRef<ConstColXprIn> MotionIn;

      typedef typename Matrix6xLikeOut::ColXpr ColXprOut;
      typedef MotionRef<ColXprOut> MotionOut;

      const int colRef = nv(model.joints[joint_id]) + idx_v(model.joints[joint_id]) - 1;
      switch (rf)
      {
      case WORLD: {
        for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
        {
          MotionIn v_in(Jin.col(j));
          MotionOut v_out(Jout_.col(j));

          v_out = v_in;
        }
        break;
      }
      case LOCAL_WORLD_ALIGNED: {
        for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
        {
          MotionIn v_in(Jin.col(j));
          MotionOut v_out(Jout_.col(j));

          v_out = v_in;
          v_out.linear() -= placement.translation().cross(v_in.angular());
        }
        break;
      }
      case LOCAL: {
        for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
        {
          MotionIn v_in(Jin.col(j));
          MotionOut v_out(Jout_.col(j));

          v_out = placement.actInv(v_in);
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

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename Matrix6xLike>
    struct JointJacobianForwardStep
    : public fusion::JointUnaryVisitorBase<JointJacobianForwardStep<
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
        data.iMf[parent] = data.liMi[i] * data.iMf[i];

        Matrix6xLike & J_ = J.const_cast_derived();
        jmodel.jointCols(J_) = data.iMf[i].actInv(jdata.S());
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
      typedef JointJacobianForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, Matrix6xLike>
        Pass;
      for (JointIndex i = jointId; i > 0; i = model.parents[i])
      {
        Pass::run(
          model.joints[i], data.joints[i],
          typename Pass::ArgsType(model, data, q.derived(), J.const_cast_derived()));
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

        jmodel.jointCols(data.J) = oMi.act(jdata.S());

        // Spatial velocity of joint i expressed in the global frame o
        data.ov[i] = oMi.act(vJ);

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock dJcols = jmodel.jointCols(data.dJ);
        ColsBlock Jcols = jmodel.jointCols(data.J);

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
        const int colRef = nv(model.joints[jointId]) + idx_v(model.joints[jointId]) - 1;
        for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
        {
          typedef typename Data::Matrix6x::ConstColXpr ConstColXprIn;
          typedef const MotionRef<ConstColXprIn> MotionIn;

          typedef typename Matrix6xLike::ColXpr ColXprOut;
          typedef MotionRef<ColXprOut> MotionOut;
          MotionIn v_in(data.J.col(j));
          MotionOut v_out(dJ.col(j));

          v_out -= v_joint.cross(oMjoint.actInv(v_in));
        }
        break;
      }
      case LOCAL_WORLD_ALIGNED: {
        const Motion & ov_joint = data.ov[jointId];
        const SE3 & oMjoint = data.oMi[jointId];
        const int colRef = nv(model.joints[jointId]) + idx_v(model.joints[jointId]) - 1;
        for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
        {
          typedef typename Data::Matrix6x::ConstColXpr ConstColXprIn;
          typedef const MotionRef<ConstColXprIn> MotionIn;

          typedef typename Matrix6xLike::ColXpr ColXprOut;
          typedef MotionRef<ColXprOut> MotionOut;
          MotionIn v_in(data.J.col(j));
          MotionOut v_out(dJ.col(j));

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
