//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_kinematics_derivatives_hxx__
#define __pinocchio_algorithm_kinematics_derivatives_hxx__

#include "pinocchio/spatial/classic-acceleration.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

namespace pinocchio
{
  namespace impl
  {
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    struct ForwardKinematicsDerivativesForwardStep
    : public fusion::JointUnaryVisitorBase<ForwardKinematicsDerivativesForwardStep<
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
        typedef typename Data::SE3 SE3;
        typedef typename Data::Motion Motion;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];
        SE3 & oMi = data.oMi[i];
        Motion & vi = data.v[i];
        Motion & ai = data.a[i];
        Motion & ov = data.ov[i];
        Motion & oa = data.oa[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        if (parent > 0)
          oMi = data.oMi[parent] * data.liMi[i];
        else
          oMi = data.liMi[i];

        vi = jdata.v();
        if (parent > 0)
          vi += data.liMi[i].actInv(data.v[parent]);

        ai = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (vi ^ jdata.v());
        if (parent > 0)
          ai += data.liMi[i].actInv(data.a[parent]);

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock dJcols = jmodel.jointCols(data.dJ);
        ColsBlock Jcols = jmodel.jointCols(data.J);

        Jcols = oMi.act(jdata.S());
        ov = oMi.act(vi); // Spatial velocity of joint i expressed in the global frame o
        motionSet::motionAction(ov, Jcols, dJcols);
        oa = oMi.act(ai); // Spatial acceleration of joint i expressed in the global frame o
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    void computeForwardKinematicsDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        a.size(), model.nv, "The acceleration vector is not of right size");
      assert(model.check(data) && "data is not consistent with model.");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.v[0].setZero();
      data.a[0].setZero();

      typedef ForwardKinematicsDerivativesForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i],
          typename Pass1::ArgsType(model, data, q.derived(), v.derived(), a.derived()));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xOut1,
      typename Matrix6xOut2>
    struct JointVelocityDerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<JointVelocityDerivativesBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        Matrix6xOut1,
        Matrix6xOut2>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<
        const Model &,
        const Data &,
        const typename Model::JointIndex &,
        const ReferenceFrame &,
        Matrix6xOut1 &,
        Matrix6xOut2 &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        const Data & data,
        const typename Model::JointIndex & jointId,
        const ReferenceFrame & rf,
        const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
        const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::SE3 SE3;
        typedef typename Data::Motion Motion;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];
        Motion vtmp; // Temporary variable

        const SE3 & oMlast = data.oMi[jointId];
        const Motion & vlast = data.ov[jointId];

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<
          typename Data::Matrix6x>::ConstType ColsBlock;
        ColsBlock Jcols = jmodel.jointCols(data.J);

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut1>::Type
          ColsBlockOut1;
        Matrix6xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut2>::Type
          ColsBlockOut2;
        Matrix6xOut2 & v_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, v_partial_dv);

        // dvec/dv: this result is then needed by dvec/dq
        ColsBlockOut2 v_partial_dv_cols = jmodel.jointCols(v_partial_dv_);
        switch (rf)
        {
        case WORLD:
          v_partial_dv_cols = Jcols;
          break;
        case LOCAL_WORLD_ALIGNED:
          details::translateJointJacobian(oMlast, Jcols, v_partial_dv_cols);
          break;
        case LOCAL:
          motionSet::se3ActionInverse(oMlast, Jcols, v_partial_dv_cols);
          break;
        default:
          assert(false && "This must never happened");
        }

        // dvec/dq
        ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
        switch (rf)
        {
        case WORLD:
          if (parent > 0)
            vtmp = data.ov[parent] - vlast;
          else
            vtmp = -vlast;
          motionSet::motionAction(vtmp, Jcols, v_partial_dq_cols);
          break;
        case LOCAL_WORLD_ALIGNED:
          if (parent > 0)
            vtmp = data.ov[parent] - vlast;
          else
            vtmp = -vlast;
          vtmp.linear() += vtmp.angular().cross(oMlast.translation());
          motionSet::motionAction(vtmp, v_partial_dv_cols, v_partial_dq_cols);
          break;
        case LOCAL:
          if (parent > 0)
          {
            vtmp = oMlast.actInv(data.ov[parent]);
            motionSet::motionAction(vtmp, v_partial_dv_cols, v_partial_dq_cols);
          }
          break;
        default:
          assert(false && "This must never happened");
        }
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xOut1,
      typename Matrix6xOut2>
    void getJointVelocityDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Model::JointIndex jointId,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
      const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut1, Data::Matrix6x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut2, Data::Matrix6x);

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_partial_dv.cols(), model.nv);
      PINOCCHIO_CHECK_INPUT_ARGUMENT((int)jointId < model.njoints, "The joint id is invalid.");
      assert(model.check(data) && "data is not consistent with model.");

      typedef typename Model::JointIndex JointIndex;

      typedef JointVelocityDerivativesBackwardStep<
        Scalar, Options, JointCollectionTpl, Matrix6xOut1, Matrix6xOut2>
        Pass1;
      for (JointIndex i = jointId; i > 0; i = model.parents[i])
      {
        Pass1::run(
          model.joints[i],
          typename Pass1::ArgsType(
            model, data, jointId, rf, PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
            PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, v_partial_dv)));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xOut1,
      typename Matrix6xOut2,
      typename Matrix6xOut3,
      typename Matrix6xOut4>
    struct JointAccelerationDerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<JointAccelerationDerivativesBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        Matrix6xOut1,
        Matrix6xOut2,
        Matrix6xOut3,
        Matrix6xOut4>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<
        const Model &,
        const Data &,
        const typename Model::JointIndex &,
        const ReferenceFrame &,
        Matrix6xOut1 &,
        Matrix6xOut2 &,
        Matrix6xOut3 &,
        Matrix6xOut4 &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        const Data & data,
        const typename Model::JointIndex & jointId,
        const ReferenceFrame & rf,
        const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
        const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
        const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
        const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::SE3 SE3;
        typedef typename Data::Motion Motion;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];
        Motion vtmp; // Temporary variable
        Motion atmp; // Temporary variable

        const SE3 & oMlast = data.oMi[jointId];
        const Motion & vlast = data.ov[jointId];
        const Motion & alast = data.oa[jointId];

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<
          typename Data::Matrix6x>::ConstType ColsBlock;
        ColsBlock dJcols = jmodel.jointCols(data.dJ);
        ColsBlock Jcols = jmodel.jointCols(data.J);

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut1>::Type
          ColsBlockOut1;
        Matrix6xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut2>::Type
          ColsBlockOut2;
        Matrix6xOut2 & a_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, a_partial_dq);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut3>::Type
          ColsBlockOut3;
        Matrix6xOut3 & a_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3, a_partial_dv);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut4>::Type
          ColsBlockOut4;
        Matrix6xOut4 & a_partial_da_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4, a_partial_da);

        ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
        ColsBlockOut2 a_partial_dq_cols = jmodel.jointCols(a_partial_dq_);
        ColsBlockOut3 a_partial_dv_cols = jmodel.jointCols(a_partial_dv_);
        ColsBlockOut4 a_partial_da_cols = jmodel.jointCols(a_partial_da_);

        // dacc/da
        switch (rf)
        {
        case WORLD:
          a_partial_da_cols = Jcols;
          break;
        case LOCAL_WORLD_ALIGNED:
          details::translateJointJacobian(oMlast, Jcols, a_partial_da_cols);
          break;
        case LOCAL:
          motionSet::se3ActionInverse(oMlast, Jcols, a_partial_da_cols);
          break;
        }

        // dacc/dv
        switch (rf)
        {
        case WORLD:
          if (parent > 0)
            vtmp = data.ov[parent] - vlast;
          else
            vtmp = -vlast;

          // also computes dvec/dq
          motionSet::motionAction(vtmp, Jcols, v_partial_dq_cols);

          a_partial_dv_cols = v_partial_dq_cols + dJcols;
          break;
        case LOCAL_WORLD_ALIGNED:
          if (parent > 0)
            vtmp = data.ov[parent] - vlast;
          else
            vtmp = -vlast;
          vtmp.linear() += vtmp.angular().cross(oMlast.translation());

          // also computes dvec/dq
          motionSet::motionAction(vtmp, a_partial_da_cols, v_partial_dq_cols);

          details::translateJointJacobian(oMlast, dJcols, a_partial_dv_cols);
          //          a_partial_dv_cols += v_partial_dq_cols; // dJcols is required later
          break;
        case LOCAL:
          // also computes dvec/dq
          if (parent > 0)
          {
            vtmp = oMlast.actInv(data.ov[parent]);
            motionSet::motionAction(vtmp, a_partial_da_cols, v_partial_dq_cols);
          }

          if (parent > 0)
            vtmp -= data.v[jointId];
          else
            vtmp = -data.v[jointId];

          motionSet::motionAction(vtmp, a_partial_da_cols, a_partial_dv_cols);
          motionSet::se3ActionInverse<ADDTO>(oMlast, dJcols, a_partial_dv_cols);
          break;
        }

        // dacc/dq
        switch (rf)
        {
        case WORLD:
          if (parent > 0)
            atmp = data.oa[parent] - alast;
          else
            atmp = -alast;
          motionSet::motionAction(atmp, Jcols, a_partial_dq_cols);

          if (parent > 0)
            motionSet::motionAction<ADDTO>(vtmp, dJcols, a_partial_dq_cols);
          break;
        case LOCAL_WORLD_ALIGNED:
          if (parent > 0)
            atmp = data.oa[parent] - alast;
          else
            atmp = -alast;

          atmp.linear() += atmp.angular().cross(oMlast.translation());
          motionSet::motionAction(atmp, a_partial_da_cols, a_partial_dq_cols);

          if (parent > 0)
            motionSet::motionAction<ADDTO>(vtmp, a_partial_dv_cols, a_partial_dq_cols);

          a_partial_dv_cols += v_partial_dq_cols;
          break;
        case LOCAL:
          if (parent > 0)
          {
            atmp = oMlast.actInv(data.oa[parent]);
            motionSet::motionAction(atmp, a_partial_da_cols, a_partial_dq_cols);

            motionSet::motionAction<ADDTO>(vtmp, v_partial_dq_cols, a_partial_dq_cols);
          }

          break;
        }
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xOut1,
      typename Matrix6xOut2,
      typename Matrix6xOut3,
      typename Matrix6xOut4>
    void getJointAccelerationDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Model::JointIndex jointId,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
      const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
      const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
      const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da)
    {

      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut1, Data::Matrix6x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut2, Data::Matrix6x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut3, Data::Matrix6x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut4, Data::Matrix6x);

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(a_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(a_partial_dv.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(a_partial_da.cols(), model.nv);
      PINOCCHIO_CHECK_INPUT_ARGUMENT((int)jointId < model.njoints, "The joint id is invalid.");
      assert(model.check(data) && "data is not consistent with model.");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      typedef JointAccelerationDerivativesBackwardStep<
        Scalar, Options, JointCollectionTpl, Matrix6xOut1, Matrix6xOut2, Matrix6xOut3, Matrix6xOut4>
        Pass1;
      for (JointIndex i = jointId; i > 0; i = model.parents[i])
      {
        Pass1::run(
          model.joints[i],
          typename Pass1::ArgsType(
            model, data, jointId, rf, PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
            PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, a_partial_dq),
            PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3, a_partial_dv),
            PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4, a_partial_da)));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix6xOut1,
      typename Matrix6xOut2,
      typename Matrix6xOut3,
      typename Matrix6xOut4,
      typename Matrix6xOut5>
    void getJointAccelerationDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Model::JointIndex jointId,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
      const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv,
      const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dq,
      const Eigen::MatrixBase<Matrix6xOut4> & a_partial_dv,
      const Eigen::MatrixBase<Matrix6xOut5> & a_partial_da)
    {
      impl::getJointAccelerationDerivatives(
        model, data, jointId, rf, PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3, a_partial_dq),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4, a_partial_dv),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut5, a_partial_da));

      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, v_partial_dv) = a_partial_da;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix3xOut1,
      typename Matrix3xOut2>
    struct PointVelocityDerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<PointVelocityDerivativesBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        Matrix3xOut1,
        Matrix3xOut2>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<
        const Model &,
        const Data &,
        const typename Data::SE3 &,
        const typename Data::Motion &,
        const ReferenceFrame &,
        Matrix3xOut1 &,
        Matrix3xOut2 &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        const Data & data,
        const typename Data::SE3 & oMpoint,
        const typename Data::Motion & spatial_point_velocity,
        const ReferenceFrame & rf,
        const Eigen::MatrixBase<Matrix3xOut1> & v_partial_dq,
        const Eigen::MatrixBase<Matrix3xOut2> & v_partial_dv)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::SE3 SE3;
        typedef typename Data::Motion Motion;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];
        Motion vtmp; // Temporary variable

        const SE3 & oMlast = oMpoint;

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<
          typename Data::Matrix6x>::ConstType ColsBlock;
        ColsBlock Jcols = jmodel.jointCols(data.J);

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut1>::Type
          ColsBlockOut1;
        Matrix3xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut1, v_partial_dq);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut2>::Type
          ColsBlockOut2;
        Matrix3xOut2 & v_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut2, v_partial_dv);

        ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
        ColsBlockOut2 v_partial_dv_cols = jmodel.jointCols(v_partial_dv_);

        const int nv = jmodel.nv();
        Eigen::Matrix<Scalar, 6, JointModel::NV, Options> v_spatial_partial_dv_cols(6, nv);

#define FOR_NV() for (Eigen::DenseIndex j = 0; j < nv; ++j)
#define GET_LINEAR(vec6) vec6.template segment<3>(Motion::LINEAR)
#define GET_ANGULAR(vec6) vec6.template segment<3>(Motion::ANGULAR)

        // dvec/dv
        motionSet::se3ActionInverse(oMlast, Jcols, v_spatial_partial_dv_cols);
        v_partial_dv_cols = v_spatial_partial_dv_cols.template middleRows<3>(Motion::LINEAR);

        // dvec/dq
        if (parent > 0)
        {
          vtmp = oMlast.actInv(data.ov[parent]);
          FOR_NV()
          v_partial_dq_cols.col(j).noalias() =
            vtmp.angular().cross(GET_LINEAR(v_spatial_partial_dv_cols.col(j)))
            + vtmp.linear().cross(GET_ANGULAR(v_spatial_partial_dv_cols.col(j)));
        }
        else
          v_partial_dq_cols.setZero();

        if (rf == LOCAL_WORLD_ALIGNED)
        {
          FOR_NV()
          v_partial_dq_cols.col(j) = oMlast.rotation()
                                     * (v_partial_dq_cols.col(j)
                                        + GET_ANGULAR(v_spatial_partial_dv_cols.col(j))
                                            .cross(spatial_point_velocity.linear()));
          FOR_NV()
          v_partial_dv_cols.col(j) = oMlast.rotation() * v_partial_dv_cols.col(j);
        }

#undef FOR_NV
#undef GET_LINEAR
#undef GET_ANGULAR
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix3xOut1,
      typename Matrix3xOut2>
    void getPointVelocityDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Model::JointIndex joint_id,
      const SE3Tpl<Scalar, Options> & placement,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix3xOut1> & v_point_partial_dq,
      const Eigen::MatrixBase<Matrix3xOut2> & v_point_partial_dv)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut1, Data::Matrix3x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut2, Data::Matrix3x);

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_point_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_point_partial_dv.cols(), model.nv);
      PINOCCHIO_CHECK_INPUT_ARGUMENT((int)joint_id < model.njoints, "The joint id is invalid.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        rf == LOCAL || rf == LOCAL_WORLD_ALIGNED,
        "The reference frame is not valid, expected LOCAL or LOCAL_WORLD_ALIGNED");
      assert(model.check(data) && "data is not consistent with model.");

      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;

      const SE3 oMpoint = data.oMi[joint_id] * placement;
      const Motion spatial_velocity = oMpoint.actInv(data.ov[joint_id]);

      typedef PointVelocityDerivativesBackwardStep<
        Scalar, Options, JointCollectionTpl, Matrix3xOut1, Matrix3xOut2>
        Pass1;
      for (JointIndex i = joint_id; i > 0; i = model.parents[i])
      {
        Pass1::run(
          model.joints[i], typename Pass1::ArgsType(
                             model, data, oMpoint, spatial_velocity, rf,
                             PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut1, v_point_partial_dq),
                             PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut2, v_point_partial_dv)));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix3xOut1,
      typename Matrix3xOut2,
      typename Matrix3xOut3,
      typename Matrix3xOut4>
    struct PointClassicAccelerationDerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<PointClassicAccelerationDerivativesBackwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        Matrix3xOut1,
        Matrix3xOut2,
        Matrix3xOut3,
        Matrix3xOut4>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<
        const Model &,
        const Data &,
        const typename Data::SE3 &,
        const typename Data::Motion &,
        const typename Data::Motion::Vector3 &,
        const ReferenceFrame &,
        Matrix3xOut1 &,
        Matrix3xOut2 &,
        Matrix3xOut3 &,
        Matrix3xOut4 &>
        ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        const Data & data,
        const typename Data::SE3 & oMpoint,
        const typename Data::Motion & spatial_point_velocity,
        const typename Data::Motion::Vector3 & point_classic_acceleration,
        const ReferenceFrame & rf,
        const Eigen::MatrixBase<Matrix3xOut1> & v_partial_dq,
        const Eigen::MatrixBase<Matrix3xOut2> & a_partial_dq,
        const Eigen::MatrixBase<Matrix3xOut3> & a_partial_dv,
        const Eigen::MatrixBase<Matrix3xOut4> & a_partial_da)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::SE3 SE3;
        typedef typename Data::Motion Motion;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];
        Motion vtmp; // Temporary variable
        Motion atmp; // Temporary variable

        const SE3 & oMlast = oMpoint;
        const Motion & v_last = spatial_point_velocity;

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<
          typename Data::Matrix6x>::ConstType ColsBlock;
        ColsBlock dJcols = jmodel.jointCols(data.dJ);
        ColsBlock Jcols = jmodel.jointCols(data.J);

        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut1>::Type
          ColsBlockOut1;
        Matrix3xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut1, v_partial_dq);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut2>::Type
          ColsBlockOut2;
        Matrix3xOut2 & a_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut2, a_partial_dq);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut3>::Type
          ColsBlockOut3;
        Matrix3xOut3 & a_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut3, a_partial_dv);
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut4>::Type
          ColsBlockOut4;
        Matrix3xOut4 & a_partial_da_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut4, a_partial_da);

        ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
        ColsBlockOut2 a_partial_dq_cols = jmodel.jointCols(a_partial_dq_);
        ColsBlockOut3 a_partial_dv_cols = jmodel.jointCols(a_partial_dv_);
        ColsBlockOut4 a_partial_da_cols = jmodel.jointCols(a_partial_da_);

        const int nv = jmodel.nv();
        Eigen::Matrix<Scalar, 6, JointModel::NV, Options> a_spatial_partial_da_cols(6, nv);
        Eigen::Matrix<Scalar, 6, JointModel::NV, Options> v_spatial_partial_dq_cols(6, nv);

#define FOR_NV() for (Eigen::DenseIndex j = 0; j < nv; ++j)
#define GET_LINEAR(vec6) vec6.template segment<3>(Motion::LINEAR)
#define GET_ANGULAR(vec6) vec6.template segment<3>(Motion::ANGULAR)

        // dacc/da
        motionSet::se3ActionInverse(oMlast, Jcols, a_spatial_partial_da_cols);
        a_partial_da_cols = a_spatial_partial_da_cols.template middleRows<3>(Motion::LINEAR);

        // dacc/dv
        // also computes dvec/dq
        if (parent > 0)
        {
          vtmp = oMlast.actInv(data.ov[parent]);
          motionSet::motionAction(vtmp, a_spatial_partial_da_cols, v_spatial_partial_dq_cols);
          v_partial_dq_cols = v_spatial_partial_dq_cols.template middleRows<3>(Motion::LINEAR);
        }
        else
          v_partial_dq_cols.setZero();

        if (parent > 0)
          vtmp -= v_last;
        else
          vtmp = -v_last;

        //      motionSet::motionAction(vtmp,a_partial_da_cols,a_partial_dv_cols);
        FOR_NV()
        a_partial_dv_cols.col(j).noalias() =
          vtmp.angular().cross(GET_LINEAR(a_spatial_partial_da_cols.col(j)))
          + vtmp.linear().cross(GET_ANGULAR(a_spatial_partial_da_cols.col(j)));
        //      motionSet::se3ActionInverse<ADDTO>(oMlast,dJcols,a_partial_dv_cols);
        FOR_NV()
        a_partial_dv_cols.col(j) +=
          oMlast.rotation().transpose()
          * (GET_LINEAR(dJcols.col(j)) + GET_ANGULAR(dJcols.col(j)).cross(oMlast.translation()));
        // wxv
        FOR_NV()
        a_partial_dv_cols.col(j) +=
          v_last.angular().cross(GET_LINEAR(a_spatial_partial_da_cols.col(j)))
          - v_last.linear().cross(GET_ANGULAR(a_spatial_partial_da_cols.col(j)));

        // dacc/dq
        if (parent > 0)
        {
          atmp = oMlast.actInv(data.oa[parent]);
          //        motionSet::motionAction(atmp,a_partial_da_cols,a_partial_dq_cols);
          FOR_NV()
          a_partial_dq_cols.col(j).noalias() =
            atmp.angular().cross(GET_LINEAR(a_spatial_partial_da_cols.col(j)))
            + atmp.linear().cross(GET_ANGULAR(a_spatial_partial_da_cols.col(j)));

          //        motionSet::motionAction<ADDTO>(vtmp,v_partial_dq_cols,a_partial_dq_cols);
          FOR_NV()
          a_partial_dq_cols.col(j) +=
            vtmp.angular().cross(GET_LINEAR(v_spatial_partial_dq_cols.col(j)))
            + vtmp.linear().cross(GET_ANGULAR(v_spatial_partial_dq_cols.col(j)));

          // wxv
          FOR_NV()
          a_partial_dq_cols.col(j) +=
            v_last.angular().cross(GET_LINEAR(v_spatial_partial_dq_cols.col(j)))
            - v_last.linear().cross(GET_ANGULAR(v_spatial_partial_dq_cols.col(j)));
        }
        else
          a_partial_dq_cols.setZero();

        if (rf == LOCAL_WORLD_ALIGNED)
        {

          FOR_NV()
          v_partial_dq_cols.col(j) = oMlast.rotation()
                                     * (v_partial_dq_cols.col(j)
                                        + GET_ANGULAR(a_spatial_partial_da_cols.col(j))
                                            .cross(spatial_point_velocity.linear()));
          FOR_NV()
          a_partial_dq_cols.col(j) =
            oMlast.rotation()
            * (a_partial_dq_cols.col(j) + GET_ANGULAR(a_spatial_partial_da_cols.col(j)).cross(point_classic_acceleration));
          FOR_NV()
          a_partial_dv_cols.col(j) = oMlast.rotation() * a_partial_dv_cols.col(j);
          FOR_NV()
          a_partial_da_cols.col(j) = oMlast.rotation() * a_partial_da_cols.col(j);
        }

#undef FOR_NV
#undef GET_LINEAR
#undef GET_ANGULAR
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix3xOut1,
      typename Matrix3xOut2,
      typename Matrix3xOut3,
      typename Matrix3xOut4>
    void getPointClassicAccelerationDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Model::JointIndex joint_id,
      const SE3Tpl<Scalar, Options> & placement,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix3xOut1> & v_point_partial_dq,
      const Eigen::MatrixBase<Matrix3xOut2> & a_point_partial_dq,
      const Eigen::MatrixBase<Matrix3xOut3> & a_point_partial_dv,
      const Eigen::MatrixBase<Matrix3xOut4> & a_point_partial_da)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut1, Data::Matrix3x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut2, Data::Matrix3x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut3, Data::Matrix3x);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut4, Data::Matrix3x);

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;
      typedef typename SE3::Vector3 Vector3;
      typedef typename Model::JointIndex JointIndex;

      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_point_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(a_point_partial_dq.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(a_point_partial_dv.cols(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(a_point_partial_da.cols(), model.nv);
      PINOCCHIO_CHECK_INPUT_ARGUMENT((int)joint_id < model.njoints, "The joint id is invalid.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        rf == LOCAL || rf == LOCAL_WORLD_ALIGNED,
        "The reference frame is not valid, expected LOCAL or LOCAL_WORLD_ALIGNED");
      assert(model.check(data) && "data is not consistent with model.");

      const SE3 oMpoint = data.oMi[joint_id] * placement;
      const Motion spatial_velocity = oMpoint.actInv(data.ov[joint_id]);
      const Motion spatial_acceleration = oMpoint.actInv(data.oa[joint_id]);
      const Vector3 point_acc = classicAcceleration(spatial_velocity, spatial_acceleration);

      typedef PointClassicAccelerationDerivativesBackwardStep<
        Scalar, Options, JointCollectionTpl, Matrix3xOut1, Matrix3xOut2, Matrix3xOut3, Matrix3xOut4>
        Pass1;
      for (JointIndex i = joint_id; i > 0; i = model.parents[i])
      {
        Pass1::run(
          model.joints[i], typename Pass1::ArgsType(
                             model, data, oMpoint, spatial_velocity, point_acc, rf,
                             PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut1, v_point_partial_dq),
                             PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut2, a_point_partial_dq),
                             PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut3, a_point_partial_dv),
                             PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut4, a_point_partial_da)));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename Matrix3xOut1,
      typename Matrix3xOut2,
      typename Matrix3xOut3,
      typename Matrix3xOut4,
      typename Matrix3xOut5>
    void getPointClassicAccelerationDerivatives(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Model::JointIndex joint_id,
      const SE3Tpl<Scalar, Options> & placement,
      const ReferenceFrame rf,
      const Eigen::MatrixBase<Matrix3xOut1> & v_point_partial_dq,
      const Eigen::MatrixBase<Matrix3xOut2> & v_point_partial_dv,
      const Eigen::MatrixBase<Matrix3xOut3> & a_point_partial_dq,
      const Eigen::MatrixBase<Matrix3xOut4> & a_point_partial_dv,
      const Eigen::MatrixBase<Matrix3xOut5> & a_point_partial_da)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut2, Data::Matrix3x);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v_point_partial_dv.cols(), model.nv);
      impl::getPointClassicAccelerationDerivatives(
        model, data, joint_id, placement, rf,
        PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut1, v_point_partial_dq),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut3, a_point_partial_dq),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut4, a_point_partial_dv),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut5, a_point_partial_da));

      PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut2, v_point_partial_dv) = a_point_partial_da;
    }

  } // namespace impl
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void computeJointKinematicHessians(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef MotionRef<const typename Data::Matrix6x::ConstColXpr> MotionIn;

    typedef typename Data::Motion Motion;
    typedef Eigen::Map<typename Motion::Vector6> MapVector6;
    typedef MotionRef<MapVector6> MotionOut;

    const typename Data::Matrix6x & J = data.J;
    typename Data::Tensor3x & kinematic_hessians = data.kinematic_hessians;
    const Eigen::DenseIndex slice_matrix_size = 6 * model.nv;

    for (size_t joint_id = 1; joint_id < (size_t)model.njoints; ++joint_id)
    {
      const std::vector<typename Model::JointIndex> & subtree = model.subtrees[joint_id];
      const std::vector<typename Model::JointIndex> & support = model.supports[joint_id];

      const int nv = model.nvs[joint_id];
      const int idx_v = model.idx_vs[joint_id];

      for (int joint_row = 0; joint_row < nv; ++joint_row)
      {
        const Eigen::DenseIndex outer_row_id = idx_v + joint_row;

        for (size_t support_id = 0; support_id < support.size() - 1; ++support_id)
        {
          const typename Model::JointIndex joint_id_support = support[support_id];

          const int inner_nv = model.nvs[joint_id_support];
          const int inner_idx_v = model.idx_vs[joint_id_support];
          for (int inner_joint_row = 0; inner_joint_row < inner_nv; ++inner_joint_row)
          {
            const Eigen::DenseIndex inner_row_id = inner_idx_v + inner_joint_row;
            assert(inner_row_id < outer_row_id);

            MapVector6 motion_vec_in(
              kinematic_hessians.data() + inner_row_id * slice_matrix_size + outer_row_id * 6);
            MapVector6 motion_vec_out(
              kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);

            motion_vec_out = -motion_vec_in;
          }
        }

        const MotionIn S1(J.col(outer_row_id));

        // Computations already done
        for (int inner_joint_row = 0; inner_joint_row < joint_row; ++inner_joint_row)
        {
          const Eigen::DenseIndex inner_row_id = idx_v + inner_joint_row;
          MapVector6 motion_vec_in(
            kinematic_hessians.data() + inner_row_id * slice_matrix_size + outer_row_id * 6);
          MapVector6 motion_vec_out(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);

          motion_vec_out = -motion_vec_in;
        }

        for (int inner_joint_row = joint_row + 1; inner_joint_row < nv; ++inner_joint_row)
        {
          const Eigen::DenseIndex inner_row_id = idx_v + inner_joint_row;
          const MotionIn S2(J.col(inner_row_id));

          MapVector6 motion_vec_out(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionOut S1xS2(motion_vec_out);

          S1xS2 = S1.cross(S2);
        }

        for (size_t subtree_id = 1; subtree_id < subtree.size(); ++subtree_id)
        {
          const typename Model::JointIndex joint_id_subtree = subtree[subtree_id];

          const int inner_nv = model.nvs[joint_id_subtree];
          const int inner_idx_v = model.idx_vs[joint_id_subtree];
          for (int inner_joint_row = 0; inner_joint_row < inner_nv; ++inner_joint_row)
          {
            const Eigen::DenseIndex inner_row_id = inner_idx_v + inner_joint_row;
            assert(inner_row_id > outer_row_id);
            const MotionIn S2(J.col(inner_row_id));

            MapVector6 motion_vec_out(
              kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
            MotionOut S1xS2(motion_vec_out);

            S1xS2 = S1.cross(S2);
          }
        }
      }
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void getJointKinematicHessian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const ReferenceFrame rf,
    Tensor<Scalar, 3, Options> & kinematic_hessian)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(
      joint_id < model.joints.size() && joint_id > 0
      && "joint_id is outside the valid index for a joint in model.joints");

    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Data::SE3 SE3;
    typedef typename Data::Motion Motion;

    const typename Data::Matrix6x & J = data.J;
    const typename Data::Tensor3x & kinematic_hessians = data.kinematic_hessians;

    // Allocate memory
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      kinematic_hessian.dimension(0), 6, "The result tensor is not of the right dimension.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      kinematic_hessian.dimension(1), model.nv, "The result tensor is not of the right dimension.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      kinematic_hessian.dimension(2), model.nv, "The result tensor is not of the right dimension.");

    const int idx_vj = model.joints[joint_id].idx_v();
    const int nvj = model.joints[joint_id].nv();
    const Eigen::DenseIndex slice_matrix_size = 6 * model.nv;

    typedef std::vector<int> IndexVector;
    const Eigen::DenseIndex last_idx = idx_vj + nvj - 1;
    const std::vector<int> & supporting_indexes =
      data.supports_fromRow[(size_t)(last_idx)]; // until the last element of the joint size (nvj)

    typedef Eigen::Map<typename Motion::Vector6> MapVector6;
    typedef MotionRef<MapVector6> MotionOut;
    typedef Eigen::Map<const typename Motion::Vector6> ConstMapVector6;
    typedef MotionRef<ConstMapVector6> MotionIn;

    switch (rf)
    {
    case WORLD: {
      for (size_t i = 0; i < supporting_indexes.size(); ++i)
      {
        const Eigen::DenseIndex outer_row_id = supporting_indexes[i];

        // Take into account parent indexes of the current joint motion subspace
        for (int subspace_idx = data.start_idx_v_fromRow[(size_t)outer_row_id];
             subspace_idx < outer_row_id; ++subspace_idx)
        {
          ConstMapVector6 vec_in(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + subspace_idx * 6);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + subspace_idx * 6);

          vec_out = vec_in;
        }

        for (size_t j = i + 1; j < supporting_indexes.size(); ++j)
        {
          const Eigen::DenseIndex inner_row_id = supporting_indexes[j];

          ConstMapVector6 vec_in(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);

          vec_out = vec_in;
        }
      }
      break;
    }
    case LOCAL_WORLD_ALIGNED: {
      typedef MotionRef<const typename Data::Matrix6x::ConstColXpr> MotionColRef;
      const SE3 & oMlast = data.oMi[joint_id];

      for (size_t i = 0; i < supporting_indexes.size(); ++i)
      {
        const Eigen::DenseIndex outer_row_id = supporting_indexes[i];
        const MotionColRef S1(J.col(outer_row_id));

        for (size_t j = 0; j < supporting_indexes.size(); ++j)
        {
          const Eigen::DenseIndex inner_row_id = supporting_indexes[j];
          if (inner_row_id >= data.start_idx_v_fromRow[(size_t)outer_row_id])
            break;

          MotionColRef S2(J.col(inner_row_id));

          ConstMapVector6 vec_in(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionIn S1xS2(vec_in);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionOut m_out(vec_out);

          m_out.linear() =
            -(S1.linear() - oMlast.translation().cross(S1.angular())).cross(S2.angular());
        }

        // Take into account parent indexes of the current joint motion subspace
        for (int inner_row_id = data.start_idx_v_fromRow[(size_t)outer_row_id];
             inner_row_id < outer_row_id; ++inner_row_id)
        {
          MotionColRef S2(J.col(inner_row_id));

          ConstMapVector6 vec_in(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionIn S1xS2(vec_in);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionOut m_out(vec_out);

          vec_out = vec_in;
          m_out.linear() -=
            (S1.linear() - oMlast.translation().cross(S1.angular())).cross(S2.angular())
            + oMlast.translation().cross(S1xS2.angular());
        }

        // case: outer_row_id == inner_row_id
        {
          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + outer_row_id * 6);
          MotionOut m_out(vec_out);

          m_out.linear() =
            -(S1.linear() - oMlast.translation().cross(S1.angular())).cross(S1.angular());
        }

        for (size_t j = i + 1; j < supporting_indexes.size(); ++j)
        {
          const Eigen::DenseIndex inner_row_id = supporting_indexes[j];
          MotionColRef S2(J.col(inner_row_id));

          ConstMapVector6 vec_in(
            kinematic_hessians.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionIn S1xS2(vec_in);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionOut m_out(vec_out);

          vec_out = vec_in;
          m_out.linear() -=
            (S1.linear() - oMlast.translation().cross(S1.angular())).cross(S2.angular())
            + oMlast.translation().cross(S1xS2.angular());
        }
      }
      break;
    }
    case LOCAL: {
      const SE3 & oMlast = data.oMi[joint_id];

      for (IndexVector::const_reverse_iterator rit = supporting_indexes.rbegin();
           rit != supporting_indexes.rend(); ++rit)
      {
        const Eigen::DenseIndex outer_row_id = *rit;

        // This corresponds to the joint connected to the world, we can skip
        if (data.parents_fromRow[(size_t)data.start_idx_v_fromRow[(size_t)outer_row_id]] < 0)
          continue;

        // Take into account current joint motion subspace
        for (int subspace_idx = data.end_idx_v_fromRow[(size_t)outer_row_id];
             subspace_idx > outer_row_id; --subspace_idx)
        {
          ConstMapVector6 vec_in(
            kinematic_hessians.data() + subspace_idx * slice_matrix_size + outer_row_id * 6);
          MotionIn m_in(vec_in);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + subspace_idx * 6);
          MotionOut m_out(vec_out);

          m_out = oMlast.actInv(m_in);
        }

        IndexVector::const_reverse_iterator inner_rit = rit;
        for (++inner_rit; inner_rit != supporting_indexes.rend(); ++inner_rit)
        {
          const Eigen::DenseIndex inner_row_id = *inner_rit;

          ConstMapVector6 vec_in(
            kinematic_hessians.data() + inner_row_id * slice_matrix_size + outer_row_id * 6);

          MotionIn m_in(vec_in);

          MapVector6 vec_out(
            kinematic_hessian.data() + outer_row_id * slice_matrix_size + inner_row_id * 6);
          MotionOut m_out(vec_out);

          m_out = oMlast.actInv(m_in);
        }
      }

      break;
    }
    default:
      assert(false && "must never happened");
      break;
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  void computeForwardKinematicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    impl::computeForwardKinematicsDerivatives(
      model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2>
  void getJointVelocityDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Model::JointIndex jointId,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
  {
    impl::getJointVelocityDerivatives(
      model, data, jointId, rf, make_ref(v_partial_dq), make_ref(v_partial_dv));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2,
    typename Matrix6xOut3,
    typename Matrix6xOut4>
  void getJointAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Model::JointIndex jointId,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da)
  {
    impl::getJointAccelerationDerivatives(
      model, data, jointId, rf, make_ref(v_partial_dq), make_ref(a_partial_dq),
      make_ref(a_partial_dv), make_ref(a_partial_da));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2,
    typename Matrix6xOut3,
    typename Matrix6xOut4,
    typename Matrix6xOut5>
  void getJointAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Model::JointIndex jointId,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut4> & a_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut5> & a_partial_da)
  {
    impl::getJointAccelerationDerivatives(
      model, data, jointId, rf, make_ref(v_partial_dq), make_ref(v_partial_dv),
      make_ref(a_partial_dq), make_ref(a_partial_dv), make_ref(a_partial_da));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix3xOut1,
    typename Matrix3xOut2>
  void getPointVelocityDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Model::JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix3xOut1> & v_point_partial_dq,
    const Eigen::MatrixBase<Matrix3xOut2> & v_point_partial_dv)
  {
    impl::getPointVelocityDerivatives(
      model, data, joint_id, placement, rf, make_ref(v_point_partial_dq),
      make_ref(v_point_partial_dv));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix3xOut1,
    typename Matrix3xOut2,
    typename Matrix3xOut3,
    typename Matrix3xOut4>
  void getPointClassicAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Model::JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix3xOut1> & v_point_partial_dq,
    const Eigen::MatrixBase<Matrix3xOut2> & a_point_partial_dq,
    const Eigen::MatrixBase<Matrix3xOut3> & a_point_partial_dv,
    const Eigen::MatrixBase<Matrix3xOut4> & a_point_partial_da)
  {
    impl::getPointClassicAccelerationDerivatives(
      model, data, joint_id, placement, rf, make_ref(v_point_partial_dq),
      make_ref(a_point_partial_dq), make_ref(a_point_partial_dv), make_ref(a_point_partial_da));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix3xOut1,
    typename Matrix3xOut2,
    typename Matrix3xOut3,
    typename Matrix3xOut4,
    typename Matrix3xOut5>
  void getPointClassicAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Model::JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix3xOut1> & v_point_partial_dq,
    const Eigen::MatrixBase<Matrix3xOut2> & v_point_partial_dv,
    const Eigen::MatrixBase<Matrix3xOut3> & a_point_partial_dq,
    const Eigen::MatrixBase<Matrix3xOut4> & a_point_partial_dv,
    const Eigen::MatrixBase<Matrix3xOut5> & a_point_partial_da)
  {
    impl::getPointClassicAccelerationDerivatives(
      model, data, joint_id, placement, rf, make_ref(v_point_partial_dq),
      make_ref(v_point_partial_dv), make_ref(a_point_partial_dq), make_ref(a_point_partial_dv),
      make_ref(a_point_partial_da));
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_kinematics_derivatives_hxx__
