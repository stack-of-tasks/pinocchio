//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_kinematics_hxx__
#define __pinocchio_kinematics_hxx__

#include "model.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void updateGlobalPlacements(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      const JointIndex & parent = model.parents[i];

      if (parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
  }

  namespace impl
  {
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    struct ForwardKinematicZeroStep
    : fusion::JointUnaryVisitorBase<
        ForwardKinematicZeroStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>>
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

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    void forwardKinematics(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      assert(model.check(data) && "data is not consistent with model.");

      typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

      typedef ForwardKinematicZeroStep<Scalar, Options, JointCollectionTpl, ConfigVectorType> Algo;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Algo::run(
          model.joints[i], data.joints[i], typename Algo::ArgsType(model, data, q.derived()));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    struct ForwardKinematicFirstStep
    : fusion::JointUnaryVisitorBase<ForwardKinematicFirstStep<
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
        typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.v[i] = jdata.v();
        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        if (parent > 0)
        {
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
          data.v[i] += data.liMi[i].actInv(data.v[parent]);
        }
        else
          data.oMi[i] = data.liMi[i];
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType>
    void forwardKinematics(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The velocity vector is not of right size");
      assert(model.check(data) && "data is not consistent with model.");

      typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

      data.v[0].setZero();

      typedef ForwardKinematicFirstStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>
        Algo;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Algo::run(
          model.joints[i], data.joints[i],
          typename Algo::ArgsType(model, data, q.derived(), v.derived()));
      }
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    struct ForwardKinematicSecondStep
    : fusion::JointUnaryVisitorBase<ForwardKinematicSecondStep<
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

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];

        jmodel.calc(jdata.derived(), q.derived(), v.derived());

        data.v[i] = jdata.v();
        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        if (parent > 0)
        {
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
          data.v[i] += data.liMi[i].actInv(data.v[parent]);
        }
        else
          data.oMi[i] = data.liMi[i];

        data.a[i] =
          jdata.S() * jmodel.JointMappedVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
        data.a[i] += data.liMi[i].actInv(data.a[parent]);
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType,
      typename TangentVectorType1,
      typename TangentVectorType2>
    void forwardKinematics(
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

      typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

      data.v[0].setZero();
      data.a[0].setZero();

      typedef ForwardKinematicSecondStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
        TangentVectorType2>
        Algo;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Algo::run(
          model.joints[i], data.joints[i],
          typename Algo::ArgsType(model, data, q.derived(), v.derived(), a.derived()));
      }
    }
  } // namespace impl

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  SE3Tpl<Scalar, Options> getRelativePlacement(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex jointIdRef,
    const JointIndex jointIdTarget,
    const Convention convention)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(jointIdRef >= 0 && jointIdRef < (JointIndex)model.njoints && "invalid joint id");
    assert(jointIdTarget >= 0 && jointIdTarget < (JointIndex)model.njoints && "invalid joint id");
    switch (convention)
    {
    case Convention::LOCAL: {
      SE3Tpl<Scalar, Options> ancestorMref(1);    // Initialize to Identity
      SE3Tpl<Scalar, Options> ancestorMtarget(1); // Initialize to Identity

      size_t ancestor_ref, ancestor_target;
      findCommonAncestor(model, jointIdRef, jointIdTarget, ancestor_ref, ancestor_target);

      // Traverse the kinematic chain backward from Ref to the common ancestor
      for (size_t i = model.supports[jointIdRef].size() - 1; i > ancestor_ref; i--)
      {
        const JointIndex j = model.supports[jointIdRef][(size_t)i];
        ancestorMref = data.liMi[j].act(ancestorMref);
      }

      // Traverse the kinematic chain backward from Target to the common ancestor
      for (size_t i = model.supports[jointIdTarget].size() - 1; i > ancestor_target; i--)
      {
        const JointIndex j = model.supports[jointIdTarget][(size_t)i];
        ancestorMtarget = data.liMi[j].act(ancestorMtarget);
      }

      return ancestorMref.actInv(ancestorMtarget);
    }
    case Convention::WORLD:
      return data.oMi[jointIdRef].actInv(data.oMi[jointIdTarget]);
    default:
      throw std::invalid_argument("Bad convention.");
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  MotionTpl<Scalar, Options> getVelocity(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex jointId,
    const ReferenceFrame rf)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_UNUSED_VARIABLE(model);
    switch (rf)
    {
    case LOCAL:
      return data.v[jointId];
    case WORLD:
      return data.oMi[jointId].act(data.v[jointId]);
    case LOCAL_WORLD_ALIGNED:
      return MotionTpl<Scalar, Options>(
        data.oMi[jointId].rotation() * data.v[jointId].linear(),
        data.oMi[jointId].rotation() * data.v[jointId].angular());
    default:
      throw std::invalid_argument("Bad reference frame.");
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  MotionTpl<Scalar, Options> getAcceleration(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex jointId,
    const ReferenceFrame rf)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_UNUSED_VARIABLE(model);
    switch (rf)
    {
    case LOCAL:
      return data.a[jointId];
    case WORLD:
      return data.oMi[jointId].act(data.a[jointId]);
    case LOCAL_WORLD_ALIGNED:
      return MotionTpl<Scalar, Options>(
        data.oMi[jointId].rotation() * data.a[jointId].linear(),
        data.oMi[jointId].rotation() * data.a[jointId].angular());
    default:
      throw std::invalid_argument("Bad reference frame.");
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  MotionTpl<Scalar, Options> getClassicalAcceleration(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex jointId,
    const ReferenceFrame rf)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef MotionTpl<Scalar, Options> Motion;
    Motion vel = getVelocity(model, data, jointId, rf);
    Motion acc = getAcceleration(model, data, jointId, rf);

    acc.linear() += vel.angular().cross(vel.linear());

    return acc;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  void forwardKinematics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    impl::forwardKinematics(model, data, make_const_ref(q));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  void forwardKinematics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    impl::forwardKinematics(model, data, make_const_ref(q), make_const_ref(v));
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  void forwardKinematics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    impl::forwardKinematics(model, data, make_const_ref(q), make_const_ref(v), make_const_ref(a));
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_kinematics_hxx__
