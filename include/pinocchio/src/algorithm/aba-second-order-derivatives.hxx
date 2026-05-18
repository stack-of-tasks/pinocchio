//
// Copyright (c) 2018-2024 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/algorithm/aba-second-order-derivatives.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/algorithm/aba-second-order-derivatives.hpp"
#endif // PINOCCHIO_LSP

#include "pinocchio/spatial/act-on-set.hpp"

namespace pinocchio
{

  // Forward sweep used by ComputeABASecondOrderDerivatives to propagate the
  // input acceleration `a` through the chain. Only acceleration-dependent
  // quantities are recomputed per call:
  //   - data.oa[i]: spatial acceleration with zero gravity at the root, so
  //     the backward sweep yields (dM/dq)*a only (no gravity contribution).
  //   - ddJ_cols = oa x J_cols.
  //   - data.of[i] = oYcrb_leaf[i] * oa, the spatial force at body i.
  //   - data.oYcrb[i] is reset to oYcrb_leaf[i], since the backward sweep
  //     composites oYcrb in place and we need the leaf values at the start
  //     of each per-column iteration.
  // q-dependent quantities (data.oMi, data.liMi, data.J, data.joints[i] with
  // a valid jdata.S(), and the world-frame leaf inertia oYcrb_leaf cached by
  // the caller) are assumed to already be set by the prior computeABADerivatives
  // and ComputeRNEASecondOrderDerivatives calls, so we deliberately skip
  // jmodel.calc and the liMi/oMi/J_cols updates that a standard ABA forward
  // sweep would perform.
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename InputAccType>
  struct ComputeABASecondOrderDerivativesInnerForwardStep
  : public fusion::JointUnaryVisitorBase<ComputeABASecondOrderDerivativesInnerForwardStep<
      Scalar,
      Options,
      JointCollectionTpl,
      InputAccType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Data::Inertia Inertia;
    typedef std::vector<Inertia> InertiaVector;

    typedef boost::fusion::
      vector<const Model &, Data &, const InputAccType &, const InertiaVector &>
        ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<InputAccType> & a,
      const InertiaVector & oYcrb_leaf)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      Motion & oa = data.oa[i];

      if (parent > 0)
        oa = data.oa[parent];
      else
        oa.setZero(); // zero gravity: result is (dM/dq)*a, not (dM/dq)*a + dh/dq

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock ddJ_cols = jmodel.jointCols(data.ddJ);

      motionSet::motionAction(oa, J_cols, ddJ_cols);
      oa += data.oMi[i].act(jdata.S() * jmodel.jointVelocitySelector(a));

      data.oYcrb[i] = oYcrb_leaf[i];
      data.of[i] = data.oYcrb[i] * oa;
    }
  };

  // Backward sweep companion to ComputeABASecondOrderDerivativesInnerForwardStep.
  // Fills `dM_a_dq(j, k) = d/dq_k ( M(q) * a )_j` using `Ftmp1` and `Ftmp3`
  // as accumulation workspace (each sized 6 x model.nv).
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename FtmpType,
    typename OutputMatrixType>
  struct ComputeABASecondOrderDerivativesInnerBackwardStep
  : public fusion::JointUnaryVisitorBase<ComputeABASecondOrderDerivativesInnerBackwardStep<
      Scalar,
      Options,
      JointCollectionTpl,
      FtmpType,
      OutputMatrixType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &, FtmpType &, FtmpType &, OutputMatrixType &>
      ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      const Model & model,
      Data & data,
      FtmpType & Ftmp1,
      FtmpType & Ftmp3,
      OutputMatrixType & dM_a_dq)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock ddJ_cols = jmodel.jointCols(data.ddJ);
      ColsBlock tmp1 = jmodel.jointCols(Ftmp1);
      ColsBlock tmp3 = jmodel.jointCols(Ftmp3);

      const Eigen::Index joint_idx = (Eigen::Index)jmodel.idx_v();
      const Eigen::Index joint_dofs = (Eigen::Index)jmodel.nv();
      const Eigen::Index subtree_dofs = (Eigen::Index)data.nvSubtree[i];
      const Eigen::Index successor_idx = joint_idx + joint_dofs;
      const Eigen::Index successor_dofs = subtree_dofs - joint_dofs;

      typename Data::Inertia & oYcrb = data.oYcrb[i];
      motionSet::inertiaAction(oYcrb, J_cols, tmp1);
      motionSet::inertiaAction<ADDTO>(oYcrb, ddJ_cols, tmp3);
      motionSet::act<ADDTO>(J_cols, data.of[i], tmp3);

      if (successor_dofs > 0)
        dM_a_dq.block(joint_idx, successor_idx, joint_dofs, successor_dofs).noalias() =
          J_cols.transpose() * Ftmp3.middleCols(successor_idx, successor_dofs);

      dM_a_dq.block(joint_idx, joint_idx, subtree_dofs, joint_dofs).noalias() =
        Ftmp1.middleCols(joint_idx, subtree_dofs).transpose() * ddJ_cols;

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
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename Tensor1,
    typename Tensor2,
    typename Tensor3,
    typename Tensor4>
  inline void ComputeABASecondOrderDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const Tensor1 & d2ddq_dqdq,
    const Tensor2 & d2ddq_dvdv,
    const Tensor3 & d2ddq_dqdv,
    const Tensor4 & d2ddq_dtaudq)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(tau.size(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dqdq.dimension(0), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dqdq.dimension(1), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dqdq.dimension(2), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dvdv.dimension(0), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dvdv.dimension(1), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dvdv.dimension(2), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dqdv.dimension(0), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dqdv.dimension(1), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dqdv.dimension(2), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dtaudq.dimension(0), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dtaudq.dimension(1), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(d2ddq_dtaudq.dimension(2), model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    assert(model.check(MimicChecker()) && "Function does not support mimic joints");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Data::MatrixXs MatrixXs;
    typedef typename Data::VectorXs VectorXs;
    typedef typename Data::Matrix6x Matrix6x;
    typedef typename Data::Tensor3x Tensor3x;
    typedef typename Model::JointIndex JointIndex;

    const Eigen::Index nv = model.nv;
    const Eigen::Index nv2 = nv * nv;

    // 1) First-order ABA derivatives: fills data.ddq, data.ddq_dq, data.ddq_dv
    //    and (upper triangle of) data.Minv.
    computeABADerivatives(model, data, q.derived(), v.derived(), tau.derived());
    // Symmetrize Minv: computeABADerivatives only fills the upper triangle
    data.Minv.template triangularView<Eigen::StrictlyLower>() =
      data.Minv.transpose().template triangularView<Eigen::StrictlyLower>();

    // 2) Second-order RNEA derivatives at (q, v, ddq). d2tau_dadq holds the
    //    pages of dM/dq.
    ComputeRNEASecondOrderDerivatives(
      model, data, q.derived(), v.derived(), data.ddq, data.d2tau_dqdq, data.d2tau_dvdv,
      data.d2tau_dqdv, data.d2tau_dadq);

    // 3) Inner-Term: for each column w of (ddq_dq, ddq_dv, Minv), compute
    //    (dM/dq) * column and store as page w of the corresponding tensor.
    //
    // The per-column forward sweep below is the acceleration-only pass; it
    // reuses the q-dependent kinematics (data.oMi, data.J, data.joints[i])
    // already populated by computeABADerivatives and SO RNEA, and the leaf
    // body inertias cached in oYcrb_leaf right below.
    typedef ComputeABASecondOrderDerivativesInnerForwardStep<
      Scalar, Options, JointCollectionTpl, VectorXs>
      InnerPass1;
    typedef ComputeABASecondOrderDerivativesInnerBackwardStep<
      Scalar, Options, JointCollectionTpl, Matrix6x, MatrixXs>
      InnerPass2;

    // Cache the world-frame leaf body inertia once. SO RNEA leaves
    // data.oYcrb in its composited state, and the inner-term backward sweep
    // mutates data.oYcrb again per call, so we restore the leaf values at
    // the start of each per-column forward pass.
    typename InnerPass1::InertiaVector oYcrb_leaf((std::size_t)model.njoints);
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      oYcrb_leaf[i] = data.oMi[i].act(model.inertias[i]);

    // TODO: preallocate these scratch buffers (three nv-cubed tensors, two
    // 6-by-nv matrices, one nv-by-nv matrix, plus the two nv-by-4*nv^2
    // matrices in step 4) as Data fields to avoid the per-call allocations
    // (relevant for trajectory-optimization loops that call this per knot
    // point).
    Tensor3x prodq(nv, nv, nv), prodv(nv, nv, nv), prodqdd(nv, nv, nv);
    Matrix6x Ftmp1 = Matrix6x::Zero(6, nv);
    Matrix6x Ftmp3 = Matrix6x::Zero(6, nv);
    MatrixXs inner_out(nv, nv);

    auto fill_inner = [&](const auto & input_columns, Tensor3x & out_tensor) {
      for (Eigen::Index w = 0; w < nv; ++w)
      {
        VectorXs a_input = input_columns.col(w);
        for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
          InnerPass1::run(
            model.joints[i], data.joints[i],
            typename InnerPass1::ArgsType(model, data, a_input, oYcrb_leaf));

        Ftmp1.setZero();
        Ftmp3.setZero();
        inner_out.setZero();
        for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
          InnerPass2::run(
            model.joints[i], typename InnerPass2::ArgsType(model, data, Ftmp1, Ftmp3, inner_out));

        Eigen::Map<MatrixXs>(out_tensor.data() + w * nv2, nv, nv) = inner_out;
      }
    };

    fill_inner(data.ddq_dq, prodq);
    fill_inner(data.ddq_dv, prodv);
    fill_inner(data.Minv, prodqdd);

    // 4) Outer-Term: build term_in (nv x 4*nv*nv) and take a single dense
    //    -M^{-1} * term_in product. Singh, Russell & Wensing (2023),
    //    Fig. 14b: at practical robotics sizes the BLAS-3 GEMM beats the
    //    AZA recursion on the Outer-Term.
    MatrixXs term_in(nv, 4 * nv2);
    for (Eigen::Index u = 0; u < nv; ++u)
    {
      for (Eigen::Index i = 0; i < nv; ++i)
      {
        for (Eigen::Index n = 0; n < nv; ++n)
        {
          term_in(n, (4 * u + 0) * nv + i) =
            data.d2tau_dqdq(n, i, u) + prodq(n, u, i) + prodq(n, i, u);
          term_in(n, (4 * u + 1) * nv + i) = data.d2tau_dvdv(n, i, u);
          term_in(n, (4 * u + 2) * nv + i) = data.d2tau_dqdv(n, i, u) + prodv(n, i, u);
          term_in(n, (4 * u + 3) * nv + i) = prodqdd(n, u, i);
        }
      }
    }

    MatrixXs term_out(nv, 4 * nv2);
    term_out.noalias() = -data.Minv * term_in;

    // 5) Scatter term_out into the four output tensors.
    for (Eigen::Index u = 0; u < nv; ++u)
    {
      for (Eigen::Index i = 0; i < nv; ++i)
      {
        for (Eigen::Index n = 0; n < nv; ++n)
        {
          const_cast<Tensor1 &>(d2ddq_dqdq)(n, i, u) = term_out(n, (4 * u + 0) * nv + i);
          const_cast<Tensor2 &>(d2ddq_dvdv)(n, i, u) = term_out(n, (4 * u + 1) * nv + i);
          const_cast<Tensor3 &>(d2ddq_dqdv)(n, i, u) = term_out(n, (4 * u + 2) * nv + i);
          const_cast<Tensor4 &>(d2ddq_dtaudq)(n, i, u) = term_out(n, (4 * u + 3) * nv + i);
        }
      }
    }
  }

} // namespace pinocchio
