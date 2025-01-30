//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_crba_hxx__
#define __pinocchio_crba_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"

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

        jmodel.jointExtendedModelCols(data.J) = data.oMi[i].act(jdata.S());

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
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Model &, Data &> ArgsType;

      template<typename JointModel>
      static void algo(const JointModelBase<JointModel> & jmodel, const Model & model, Data & data)
      {
        algo_impl(jmodel, model, data);
      }

      template<typename JointModel>
      static void
      algo_impl(const JointModelBase<JointModel> & jmodel, const Model & model, Data & data)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef
          typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
            ColsBlock;
        const JointIndex & i = jmodel.id();

        // Centroidal momentum map
        ColsBlock Ag_cols = jmodel.jointCols(data.Ag);
        ColsBlock J_cols = jmodel.jointExtendedModelCols(data.J);
        motionSet::inertiaAction(data.oYcrb[i], J_cols, Ag_cols);

        // Joint Space Inertia Matrix
        data.M.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nv(), data.nvSubtree[i]).noalias() =
          J_cols.transpose() * data.Ag.middleCols(jmodel.idx_v(), data.nvSubtree[i]);

        const JointIndex & parent = model.parents[i];
        data.oYcrb[parent] += data.oYcrb[i];
      }

      static void
      algo_impl(const JointModelBase<JointModelMimic> & jmodel, const Model & model, Data & data)
      {
        typedef typename Model::JointIndex JointIndex;
        const JointIndex & i = jmodel.id();

        const JointIndex & parent = model.parents[i];
        data.oYcrb[parent] += data.oYcrb[i];
      }
    };

    // In case of a tree with a mimic joint, as seen above, the backward pass, is truncated, in
    // order not to replace the mimicked joint(s) info in the matrix. This step allows for the
    // mimicking joint(s) contribution to be added. It is done in 3 steps:
    // - First we compute the mimicking joint(s) Force matrix
    // - Then we compute the "old" row of the mimicking joint in the matrix, by getting the subtree
    // of mimicking joint
    // - Since here only the upper part of the matrix is computed, we need to go over the support of
    // the mimicking joint, and compute how the force matrix of the mimicking joint is affecting
    // each joint ATTENTION : the last loop is spilt in 2, to only fill the upper part of the matrix
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct CrbaWorldConventionMimicStep
    : public fusion::JointUnaryVisitorBase<
        CrbaWorldConventionMimicStep<Scalar, Options, JointCollectionTpl>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Model &, Data &, const size_t &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        const Model & model,
        Data & data,
        const size_t & mims_id)
      {
        algo_impl(jmodel, model, data, mims_id);
      }

      template<typename JointModel>
      static void
      algo_impl(const JointModelBase<JointModel> &, const Model &, Data &, const size_t &)
      {
      }

      static void algo_impl(
        const JointModelBase<JointModelMimic> & jmodel,
        const Model & model,
        Data & data,
        const size_t & mims_id)
      {
        typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModel;

        JointIndex mimicking_id = jmodel.id();
        JointIndex mimicked_id = jmodel.derived().jmodel().id();

        Eigen::Ref<Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options, 6, JointModel::MaxNVMimicked>>
          J_cols = jmodel.jointExtendedModelCols(data.J);
        Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options, 6, JointModel::MaxNVMimicked> Ag_sec(
          6, jmodel.nvExtended());

        motionSet::inertiaAction(data.oYcrb[mimicking_id], J_cols, Ag_sec);

        // Compute mimicking terms that were previously in the diagonal
        data.M.block(jmodel.idx_v(), jmodel.idx_v(), jmodel.nvExtended(), jmodel.nvExtended())
          .noalias() += J_cols.transpose() * Ag_sec;

        // Compute for the "subtree" of the mimicking
        JointIndex sub_mimic_id = data.mimic_subtree_joint[mims_id];
        if (sub_mimic_id != 0)
          jmodel.jointRows(data.M)
            .middleCols(model.idx_vs[sub_mimic_id], data.nvSubtree[sub_mimic_id])
            .noalias() +=
            J_cols.transpose()
            * data.Ag.middleCols(model.idx_vs[sub_mimic_id], data.nvSubtree[sub_mimic_id]);

        const auto & supports = model.supports[mimicking_id];
        size_t j = supports.size() - 2;
        Eigen::Matrix<
          Scalar, Eigen::Dynamic, Eigen::Dynamic, Options, JointModel::MaxNVMimicked,
          JointModel::MaxNVMimicked>
          temp_JAG;
        for (; model.idx_vs[supports[j]] >= jmodel.idx_v(); j--)
        {
          int sup_idx_v = model.idx_vs[supports[j]];
          int sup_nv = model.nvs[supports[j]];
          int sup_idx_vExtended = model.idx_vExtendeds[supports[j]];
          int sup_nvExtended = model.nvExtendeds[supports[j]];

          temp_JAG.noalias() =
            data.J.middleCols(sup_idx_vExtended, sup_nvExtended).transpose() * Ag_sec;
          data.M.block(jmodel.idx_v(), sup_idx_v, jmodel.nvExtended(), sup_nv).noalias() +=
            temp_JAG;
          if (supports[j] == mimicked_id)
            data.M.block(jmodel.idx_v(), sup_idx_v, jmodel.nvExtended(), sup_nv).noalias() +=
              temp_JAG;
        }

        for (; j > 0; j--)
        {
          int sup_idx_v = model.idx_vs[supports[j]];
          int sup_nv = model.nvs[supports[j]];
          int sup_idx_vExtended = model.idx_vExtendeds[supports[j]];
          int sup_nvExtended = model.nvExtendeds[supports[j]];

          data.M.block(sup_idx_v, jmodel.idx_v(), sup_nv, jmodel.nvExtended()).noalias() +=
            data.J.middleCols(sup_idx_vExtended, sup_nvExtended).transpose() * Ag_sec;
        }

        // Mimic joint also have an effect on the centroidal map momentum, so it's important to
        // compute it and add it to its mimicked. It is done here Ag[prim] += oYCRB[sec] * J[sec]
        motionSet::inertiaAction<ADDTO>(
          data.oYcrb[mimicking_id], J_cols, jmodel.jointCols(data.Ag));
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
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Model &, Data &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data)
      {
        algo_impl(jmodel, jdata, model, data);
      }

      template<typename JointModel>
      static void algo_impl(
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

      static void algo_impl(
        const JointModelBase<JointModelMimic> & jmodel,
        JointDataBase<typename JointModelMimic::JointDataDerived> &
        /* jdata */,
        const Model & model,
        Data & data)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Matrix6x & Matrix;

        const JointIndex & i = jmodel.id();
        const JointIndex & parent = model.parents[i];
        if (parent > 0)
        {
          /*   Yli += liXi Yi */
          data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);

          /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
          Matrix jF = data.Fcrb[parent];
          Matrix iF = data.Fcrb[i];
          // Since we don't just copy columns, we need to use ADDTO method, to avoid ecrasing
          // already filled columns, in case of parallel arms, it is important to not ecrase already
          // filled data from the other part of the tree
          // By using data.mimic_subtree_joint, we could do the same as for a non-mimic joint.
          // But to do so, we need to change the either this visitor or data.mimic_subtree_joint
          forceSet::se3Action<ADDTO>(data.liMi[i], iF, jF);
        }
      }
    };

    // In case of a tree with a mimic joint, as seen above, the backward pass, is truncated, in
    // order not to replace the mimicked joint(s) info in the matrix. This step allows for the
    // mimicking joint(s) contribution to be added. It is done in 3 steps:
    // - First we compute the mimicking joint(s) Force matrix
    // - Then we compute the "old" row of the mimicking joint in the matrix, by getting the subtree
    // of mimicking joint
    // - Since here only the upper part of the matrix is computed, we need to go over the support of
    // the mimicking joint, and compute how the force matrix of the mimicking joint is affecting
    // each joint ATTENTION : the last loop is spilt in 2, to only fill the upper part of the matrix
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct CrbaLocalConventionMimicStep
    : public fusion::JointUnaryVisitorBase<
        CrbaLocalConventionMimicStep<Scalar, Options, JointCollectionTpl>>
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
      typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

      typedef boost::fusion::vector<const Model &, Data &, const size_t &> ArgsType;

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel,
        JointDataBase<typename JointModel::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const size_t & mims_id)
      {
        algo_impl(jmodel, jdata, model, data, mims_id);
      }

      template<typename JointModel>
      static void algo_impl(
        const JointModelBase<JointModel> &,
        JointDataBase<typename JointModel::JointDataDerived> &,
        const Model &,
        Data &,
        const size_t &)
      {
      }

      static void algo_impl(
        const JointModelBase<JointModelMimic> & jmodel,
        JointDataBase<typename JointModelMimic::JointDataDerived> & jdata,
        const Model & model,
        Data & data,
        const size_t & mims_id)
      {
        typedef JointModelMimic JointModel;

        const JointIndex & mimicking_id = jmodel.id();
        const JointIndex & mimicked_id = jmodel.derived().jmodel().id();
        auto & F = data.Fcrb[mimicking_id];

        /* F[1:6,i] = Y*S */
        jmodel.jointCols(F) = data.Ycrb[mimicking_id] * jdata.S();

        // Add mimicking joint contribution to the Matrix
        jmodel.jointBlock(data.M).noalias() += jdata.S().transpose() * jmodel.jointCols(F);

        // Add mimicking subtree contribution to the matrix.
        JointIndex sub_mimic_id = data.mimic_subtree_joint[mims_id];
        if (sub_mimic_id != 0)
        {
          jmodel.jointRows(data.M)
            .middleCols(model.idx_vs[sub_mimic_id], data.nvSubtree[sub_mimic_id])
            .noalias() += jdata.S().transpose()
                          * F.middleCols(model.idx_vs[sub_mimic_id], data.nvSubtree[sub_mimic_id]);
        }

        const auto & supports = model.supports[mimicking_id];
        size_t j = supports.size() - 2;
        Eigen::Matrix<
          Scalar, Eigen::Dynamic, Eigen::Dynamic, Options, JointModel::MaxNVMimicked,
          JointModel::MaxNVMimicked>
          temp_JAG;
        for (; model.idx_vs[supports[j]] >= jmodel.idx_v(); j--)
        {
          const JointIndex & li = supports[j];
          const JointIndex & i = supports[j + 1]; // Child link to compute placement

          forceSet::se3Action(data.liMi[i], jmodel.jointCols(F), jmodel.jointCols(F));
          int row_idx = jmodel.idx_v();
          int col_idx = model.idx_vs[li];

          applyConstraintOnForceVisitor<SETTO>(
            data.joints[li], jmodel.jointCols(F), temp_JAG.noalias());
          data.M.block(row_idx, col_idx, jmodel.nvExtended(), model.nvs[li]).noalias() += temp_JAG;
          if (li == mimicked_id)
            data.M.block(row_idx, col_idx, jmodel.nvExtended(), model.nvs[li]).noalias() +=
              temp_JAG;
        }
        for (; j > 0; j--)
        {
          const JointIndex & li = supports[j];
          const JointIndex & i = supports[j + 1]; // Child link to compute placement

          forceSet::se3Action(data.liMi[i], jmodel.jointCols(F), jmodel.jointCols(F));
          int col_idx = jmodel.idx_v();
          int row_idx = model.idx_vs[li];

          applyConstraintOnForceVisitor<ADDTO>(
            data.joints[li], jmodel.jointCols(F),
            data.M.block(row_idx, col_idx, model.nvs[li], jmodel.nvExtended()).noalias());
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

      typedef CrbaLocalConventionMimicStep<Scalar, Options, JointCollectionTpl> Pass3;
      for (size_t i = 0; i < model.mimicking_joints.size(); i++)
      {
        Pass3::run(
          model.joints[model.mimicking_joints[i]], data.joints[model.mimicking_joints[i]],
          typename Pass3::ArgsType(model, data, i));
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

      typedef CrbaWorldConventionMimicStep<Scalar, Options, JointCollectionTpl> Pass3;
      for (size_t i = 0; i < model.mimicking_joints.size(); i++)
      {
        Pass3::run(
          model.joints[model.mimicking_joints[i]], typename Pass3::ArgsType(model, data, i));
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
    default:
      throw std::invalid_argument("Bad convention.");
    }
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_crba_hxx__
