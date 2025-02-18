//
// Copyright (c) 2015-2022 CNRS INRIA
//

#ifndef __pinocchio_algorithm_rnea_hxx__
#define __pinocchio_algorithm_rnea_hxx__

/// @cond DEV

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
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    struct RneaForwardStep
    : public fusion::JointUnaryVisitorBase<RneaForwardStep<
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

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        data.v[i] = jdata.v();
        if (parent > 0)
          data.v[i] += data.liMi[i].actInv(data.v[parent]);

        data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
        data.a_gf[i] += jdata.S() * jmodel.jointVelocitySelector(a);
        data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
        //
        //      data.f[i] = model.inertias[i]*data.a_gf[i];// + model.inertias[i].vxiv(data.v[i]);
        //      // -f_ext data.h[i] = model.inertias[i]*data.v[i];
        model.inertias[i].__mult__(data.v[i], data.h[i]);
        model.inertias[i].__mult__(data.a_gf[i], data.f[i]);
        data.f[i] += data.v[i].cross(data.h[i]);
        //      data.h[i].motionAction(data.v[i],data.f[i]);
        //      data.f[i] = model.inertias[i].vxiv(data.v[i]);
        //      data.f[i].setZero();
      }
    };

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct RneaBackwardStep
    : public fusion::JointUnaryVisitorBase<RneaBackwardStep<Scalar, Options, JointCollectionTpl>>
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

        jmodel.jointVelocitySelector(data.tau) = jdata.S().transpose() * data.f[i];

        if (parent > 0)
          data.f[parent] += data.liMi[i].act(data.f[i]);
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & rnea(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        a.size(), model.nv, "The acceleration vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.v[0].setZero();
      data.a_gf[0] = -model.gravity;

      typedef RneaForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2>
        Pass1;
      typename Pass1::ArgsType arg1(model, data, q.derived(), v.derived(), a.derived());
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(model.joints[i], data.joints[i], arg1);
      }

      typedef RneaBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      typename Pass2::ArgsType arg2(model, data);
      for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
      {
        Pass2::run(model.joints[i], data.joints[i], arg2);
      }

      // Add rotorinertia contribution
      data.tau.array() += model.armature.array() * a.array(); // Check if there is memory allocation

      return data.tau;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2,
      typename ForceDerived>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & rnea(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a,
      const container::aligned_vector<ForceDerived> & fext)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(fext.size(), model.joints.size());
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        a.size(), model.nv, "The acceleration vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.v[0].setZero();
      data.a_gf[0] = -model.gravity;

      typedef RneaForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i],
          typename Pass1::ArgsType(model, data, q.derived(), v.derived(), a.derived()));
        data.f[i] -= fext[i];
      }

      typedef RneaBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
      {
        Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));
      }

      // Add armature contribution
      data.tau.array() +=
        model.armature.array() * a.array(); // TODO: check if there is memory allocation

      return data.tau;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    struct NLEForwardStep
    : public fusion::JointUnaryVisitorBase<
        NLEForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>>
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
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        data.v[i] = jdata.v();
        if (parent > 0)
          data.v[i] += data.liMi[i].actInv(data.v[parent]);

        data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
        data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);

        data.f[i] = model.inertias[i] * data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
      }
    };

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct NLEBackwardStep
    : public fusion::JointUnaryVisitorBase<NLEBackwardStep<Scalar, Options, JointCollectionTpl>>
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

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.jointVelocitySelector(data.nle) = jdata.S().transpose() * data.f[i];
        if (parent > 0)
          data.f[parent] += data.liMi[i].act(data.f[i]);
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
    nonLinearEffects(
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

      data.v[0].setZero();
      data.a_gf[0] = -model.gravity;

      typedef NLEForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i],
          typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
      }

      typedef NLEBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));
      }

      return data.nle;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    struct ComputeGeneralizedGravityForwardStep
    : public fusion::JointUnaryVisitorBase<
        ComputeGeneralizedGravityForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &> ArgsType;

      template<typename JointModel>
      static void algo(
        const pinocchio::JointModelBase<JointModel> & jmodel,
        pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        data.a_gf[i] = data.liMi[i].actInv(data.a_gf[(size_t)parent]);
        data.f[i] = model.inertias[i] * data.a_gf[i];
      }
    };

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct ComputeGeneralizedGravityBackwardStep
    : public fusion::JointUnaryVisitorBase<
        ComputeGeneralizedGravityBackwardStep<Scalar, Options, JointCollectionTpl>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

      typedef boost::fusion::vector<const Model &, Data &, typename Data::VectorXs &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        typename Data::VectorXs & g)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.jointVelocitySelector(g) = jdata.S().transpose() * data.f[i];
        if (parent > 0)
          data.f[(size_t)parent] += data.liMi[i].act(data.f[i]);
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
    computeGeneralizedGravity(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.a_gf[0] = -model.gravity;

      typedef ComputeGeneralizedGravityForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
      }

      typedef ComputeGeneralizedGravityBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data, data.g));
      }

      return data.g;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
    computeStaticTorque(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const container::aligned_vector<ForceTpl<Scalar, Options>> & fext)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        fext.size(), (size_t)model.njoints, "The size of the external forces is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.a_gf[0] = -model.gravity;

      typedef ComputeGeneralizedGravityForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
        data.f[i] -= fext[i];
      }

      typedef ComputeGeneralizedGravityBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(
          model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data, data.tau));
      }

      return data.tau;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    struct CoriolisMatrixForwardStep
    : public fusion::JointUnaryVisitorBase<CoriolisMatrixForwardStep<
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
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];

        // express quantities in the world frame
        data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);

        data.v[i] = jdata.v();
        if (parent > 0)
          data.v[i] += data.liMi[i].actInv(data.v[parent]);
        data.ov[i] = data.oMi[i].act(data.v[i]);
        data.oh[i] = data.oYcrb[i] * data.ov[i];

        // computes S expressed at the world frame
        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock J_cols = jmodel.jointCols(data.J);
        J_cols = data.oMi[i].act(jdata.S()); // collection of S expressed at the world frame

        // computes vxS expressed at the world frame
        ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
        motionSet::motionAction(data.ov[i], J_cols, dJ_cols);

        data.B[i] = data.oYcrb[i].variation(Scalar(0.5) * data.ov[i]);
        addForceCrossMatrix(Scalar(0.5) * data.oh[i], data.B[i]);
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

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct CoriolisMatrixBackwardStep
    : public fusion::JointUnaryVisitorBase<
        CoriolisMatrixBackwardStep<Scalar, Options, JointCollectionTpl>>
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

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];

        typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) Mat_tmp(jmodel.nv(), 6);

        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
        ColsBlock J_cols = jmodel.jointCols(data.J);
        ColsBlock Ag_cols = jmodel.jointCols(data.Ag);

        motionSet::inertiaAction(data.oYcrb[i], dJ_cols, jmodel.jointCols(data.dFdv));
        jmodel.jointCols(data.dFdv).noalias() += data.B[i] * J_cols;

        data.C.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
          J_cols.transpose() * data.dFdv.middleCols(jmodel.idx_v(), data.nvSubtree[i]);

        motionSet::inertiaAction(data.oYcrb[i], J_cols, Ag_cols);
        for (int j = data.parents_fromRow[(JointIndex)jmodel.idx_v()]; j >= 0;
             j = data.parents_fromRow[(JointIndex)j])
          data.C.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() =
            Ag_cols.transpose() * data.dJ.col(j);

        Mat_tmp.topRows(jmodel.nv()).noalias() = J_cols.transpose() * data.B[i];
        for (int j = data.parents_fromRow[(JointIndex)jmodel.idx_v()]; j >= 0;
             j = data.parents_fromRow[(JointIndex)j])
          data.C.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() +=
            Mat_tmp * data.J.col(j);

        if (parent > 0)
        {
          data.oYcrb[parent] += data.oYcrb[i];
          data.B[parent] += data.B[i];
        }
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & computeCoriolisMatrix(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      typedef CoriolisMatrixForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i],
          typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
      }

      typedef CoriolisMatrixBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
      }

      return data.C;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int>
      class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2,
      typename TangentVectorType3>
    struct PassivityRneaForwardStep
    : public fusion::JointUnaryVisitorBase<PassivityRneaForwardStep<
        Scalar,
        Options,
        JointCollectionTpl,
        ConfigVectorType,
        TangentVectorType1,
        TangentVectorType2,
        TangentVectorType3>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef ForceTpl<Scalar, Options> Force;

      typedef boost::fusion::vector<
        const Model &,
        Data &,
        const ConfigVectorType &,
        const TangentVectorType1 &,
        const TangentVectorType2 &,
        const TangentVectorType3 &>
        ArgsType;

      typedef impl::CoriolisMatrixForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1>
        CoriolisPass1;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & v_r,
        const Eigen::MatrixBase<TangentVectorType3> & a_r)
      {
        typedef typename Model::JointIndex JointIndex;

        const JointIndex i = jmodel.id();
        const JointIndex parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());
        data.v[i] = jdata.v();

        jmodel.calc(jdata.derived(), q.derived(), v_r.derived());
        data.v_r[i] = jdata.v();

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        if (parent > 0) {
          data.v[i] += data.liMi[i].actInv(data.v[parent]);
          data.v_r[i] += data.liMi[i].actInv(data.v_r[parent]);
        }

        data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
        data.a_gf[i] += jdata.S() * jmodel.jointVelocitySelector(a_r);
        data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);

        // // option 1
        // model.inertias[i].__mult__(data.v_r[i], data.h[i]); 
        // // option 1

        // // option 2
        // data.B[i].setZero();
        // model.inertias[i].__mult__(data.v[i], data.h[i]); 
        // CoriolisPass1::addForceCrossMatrix(data.h[i], data.B[i]); 
        // // option 2

        // option 3 (Christoffel-consistent factorization)
        data.B[i] = model.inertias[i].variation(Scalar(0.5) * data.v[i]);
        model.inertias[i].__mult__(data.v[i], data.h[i]); 
        CoriolisPass1::addForceCrossMatrix(Scalar(0.5) * data.h[i], data.B[i]); 
        // option 3 (Christoffel-consistent factorization)
        
        model.inertias[i].__mult__(data.a_gf[i], data.f[i]);

        // // option 1
        // data.f[i] += data.v[i].cross(data.h[i]); 
        // // option 1

        // // option 2
        // data.f[i] += Force(data.B[i] * data.v_r[i].toVector()); 
        // // option 2

        // option 3 (Christoffel-consistent factorization)
        data.f[i] += Force(data.B[i] * data.v_r[i].toVector()); 
        // option 3 (Christoffel-consistent factorization)
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int>
      class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2,
      typename TangentVectorType3>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & passivityRNEA(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & v_r,
      const Eigen::MatrixBase<TangentVectorType3> & a_r)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        v.size(), model.nv, "The velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        v_r.size(), model.nv, "The auxiliary velocity vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        a_r.size(), model.nv, "The auxiliary acceleration vector is not of right size");

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      data.v[0].setZero();
      data.v_r[0].setZero();
      data.a_gf[0] = -model.gravity;

      typedef PassivityRneaForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2, TangentVectorType3>
        Pass1;
      typename Pass1::ArgsType arg1(model, data, q.derived(), v.derived(), v_r.derived(), a_r.derived());
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(model.joints[i], data.joints[i], arg1);
      }

      typedef RneaBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      typename Pass2::ArgsType arg2(model, data);
      for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
      {
        Pass2::run(model.joints[i], data.joints[i], arg2);
      }

      // Add rotorinertia contribution
      data.tau.array() += model.armature.array() * a_r.array(); // Check if there is memory allocation

      return data.tau;
    }
  } // namespace impl
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct GetCoriolisMatrixBackwardStep
  : public fusion::JointUnaryVisitorBase<
      GetCoriolisMatrixBackwardStep<Scalar, Options, JointCollectionTpl>>
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

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) Mat_tmp(jmodel.nv(), 6);

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;
      ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock Ag_cols = jmodel.jointCols(data.Ag);
      typename Data::Matrix6x & dFdv = data.Fcrb[0];
      ColsBlock dFdv_cols = jmodel.jointCols(dFdv);

      motionSet::inertiaAction(data.oYcrb[i], dJ_cols, dFdv_cols);
      dFdv_cols.noalias() += data.B[i] * J_cols;

      data.C.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
        J_cols.transpose() * dFdv.middleCols(jmodel.idx_v(), data.nvSubtree[i]);

      motionSet::inertiaAction(data.oYcrb[i], J_cols, Ag_cols);
      for (int j = data.parents_fromRow[(JointIndex)jmodel.idx_v()]; j >= 0;
           j = data.parents_fromRow[(JointIndex)j])
        data.C.middleRows(jmodel.idx_v(), jmodel.nv()).col(j).noalias() =
          Ag_cols.transpose() * data.dJ.col(j);

      Mat_tmp.topRows(jmodel.nv()).noalias() = J_cols.transpose() * data.B[i];
      for (int j = data.parents_fromRow[(JointIndex)jmodel.idx_v()]; j >= 0;
           j = data.parents_fromRow[(JointIndex)j])
        data.C.middleRows(jmodel.idx_v(), jmodel.nv()).col(j) += Mat_tmp * data.J.col(j);

      if (parent > 0)
        data.B[parent] += data.B[i];
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & getCoriolisMatrix(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::ConfigVectorType ConfigVectorType;
    typedef typename Model::TangentVectorType TangentVectorType;

    typedef impl::CoriolisMatrixForwardStep<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      typedef typename Data::Inertia Inertia;
      const Inertia oY = data.oMi[i].act(model.inertias[i]);
      data.B[i] = oY.variation(Scalar(0.5) * data.ov[i]);
      Pass1::addForceCrossMatrix(Scalar(0.5) * data.oh[i], data.B[i]);
    }

    typedef GetCoriolisMatrixBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
    for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
    {
      Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
    }

    return data.C;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & rnea(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    return impl::rnea(model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename ForceDerived>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & rnea(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a,
    const container::aligned_vector<ForceDerived> & fext)
  {
    return impl::rnea(model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a), fext);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & nonLinearEffects(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return impl::nonLinearEffects(model, data, make_const_ref(q), make_const_ref(v));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
  computeGeneralizedGravity(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    return impl::computeGeneralizedGravity(model, data, make_const_ref(q));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType &
  computeStaticTorque(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext)
  {
    return impl::computeStaticTorque(model, data, make_const_ref(q), fext);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & computeCoriolisMatrix(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return impl::computeCoriolisMatrix(model, data, make_const_ref(q), make_const_ref(v));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename TangentVectorType3>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & passivityRNEA(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & v_r,
    const Eigen::MatrixBase<TangentVectorType3> & a_r)
  {
    return impl::passivityRNEA(model, data, make_const_ref(q), make_const_ref(v), make_const_ref(v_r), make_const_ref(a_r));
  }
} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_algorithm_rnea_hxx__
