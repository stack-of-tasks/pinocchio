//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_crba_hxx__
#define __pinocchio_crba_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
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
      typename ConfigVectorType>
    struct CrbaWorldConventionForwardStep
    : public fusion::JointUnaryVisitorBase<
        CrbaWorldConventionForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>>
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
        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();

        const JointIndex & parent = model.parents[i];
        if (parent > 0)
          data.oMi[i] = data.oMi[parent] * data.liMi[i];
        else
          data.oMi[i] = data.liMi[i];

        jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());

        data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      }
    };

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct CrbaWorldConventionBackwardStep
    : public fusion::JointUnaryVisitorBase<
        CrbaWorldConventionBackwardStep<Scalar, Options, JointCollectionTpl>>
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
        const JointIndex & i = jmodel.id();

        // Centroidal momentum map
        ColsBlock Ag_cols = jmodel.jointCols(data.Ag);
        ColsBlock J_cols = jmodel.jointCols(data.J);
        motionSet::inertiaAction(data.oYcrb[i], J_cols, Ag_cols);

        // Joint Space Inertia Matrix
        data.M.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
          J_cols.transpose() * data.Ag.middleCols(jmodel.idx_v(), data.nvSubtree[i]);

        const JointIndex & parent = model.parents[i];
        data.oYcrb[parent] += data.oYcrb[i];
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    struct CrbaLocalConventionForwardStep
    : public fusion::JointUnaryVisitorBase<
        CrbaLocalConventionForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>>
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
        jmodel.calc(jdata.derived(), q.derived());

        data.liMi[i] = model.jointPlacements[i] * jdata.M();
        data.Ycrb[i] = model.inertias[i];
      }
    };

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct CrbaLocalConventionBackwardStep
    : public fusion::JointUnaryVisitorBase<
        CrbaLocalConventionBackwardStep<Scalar, Options, JointCollectionTpl>>
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
        /*
         * F[1:6,i] = Y*S
         * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
         * if li>0
         *   Yli += liXi Yi
         *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
         */

        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Matrix6x::ColsBlockXpr Block;
        const JointIndex & i = jmodel.id();

        /* F[1:6,i] = Y*S */
        // data.Fcrb[i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[i] * jdata.S();
        jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

        /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
        data.M.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
          jdata.S().transpose() * data.Fcrb[i].middleCols(jmodel.idx_v(), data.nvSubtree[i]);

        const JointIndex & parent = model.parents[i];
        if (parent > 0)
        {
          /*   Yli += liXi Yi */
          data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);

          /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
          Block jF = data.Fcrb[parent].middleCols(jmodel.idx_v(), data.nvSubtree[i]);
          Block iF = data.Fcrb[i].middleCols(jmodel.idx_v(), data.nvSubtree[i]);
          forceSet::se3Action(data.liMi[i], iF, jF);
        }
      }
    };

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & crbaLocalConvention(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");

      typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

      typedef CrbaLocalConventionForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)(model.njoints); ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
      }

      typedef CrbaLocalConventionBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));
      }

      // Add the armature contribution
      data.M.diagonal() += model.armature;

      return data.M;
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & crbaWorldConvention(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_ARGUMENT_SIZE(
        q.size(), model.nq, "The configuration vector is not of right size");

      typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

      data.oYcrb[0].setZero();
      typedef CrbaWorldConventionForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)(model.njoints); ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
      }

      typedef CrbaWorldConventionBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
      }

      // Add the armature contribution
      data.M.diagonal() += model.armature;

      // Retrieve the Centroidal Momemtum map
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef typename Data::Force Force;
      typedef Eigen::Block<typename Data::Matrix6x, 3, -1> Block3x;

      data.mass[0] = data.oYcrb[0].mass();
      data.com[0] = data.oYcrb[0].lever();

      const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
      Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
      for (long i = 0; i < model.nv; ++i)
        Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);

      return data.M;
    }
  } // namespace impl
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------

  namespace internal
  {
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    inline bool isDescendant(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex j,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex root)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      if (j >= (JointIndex)model.njoints)
        return false;
      if (j == 0)
        return root == 0;
      return (j == root) || isDescendant(model, model.parents[j], root);
    }
  } // namespace internal

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool
  CRBAChecker::checkModel_impl(const ModelTpl<Scalar, Options, JointCollectionTpl> & model) const
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    // For CRBA, the tree must be "compact", i.e. all descendants of a node i are stored
    // immediately after i in the "parents" map, i.e. forall joint i, the interval i+1..n-1
    // can be separated in two intervals [i+1..k] and [k+1..n-1], where any [i+1..k] is a descendant
    // of i and none of [k+1..n-1] is a descendant of i.
    for (JointIndex i = 1; i < (JointIndex)(model.njoints - 1);
         ++i) // no need to check joints 0 and N-1
    {
      JointIndex k = i + 1;
      while (internal::isDescendant(model, k, i))
        ++k;
      for (; int(k) < model.njoints; ++k)
        if (internal::isDescendant(model, k, i))
          return false;
    }
    return true;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & crba(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Convention convention)
  {
    switch (convention)
    {
    case Convention::LOCAL:
      return ::pinocchio::impl::crbaLocalConvention(model, data, make_const_ref(q));
    case Convention::WORLD:
      return ::pinocchio::impl::crbaWorldConvention(model, data, make_const_ref(q));
    }
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_crba_hxx__
