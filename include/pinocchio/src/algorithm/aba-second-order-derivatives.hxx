//
// Copyright (c) 2018-2024 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/algorithm/aba-second-order-derivatives.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/algorithm/aba-second-order-derivatives.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  // Forward sweep of the IDFOZA pass (Singh, Russell & Wensing, IEEE T-RO 2024, Eq. (48)) used
  // by ComputeABASecondOrderDerivatives. It propagates an arbitrary joint acceleration vector `a`
  // with zero joint velocity and zero gravity, so that the companion backward sweep returns
  // d/dq ( M(q) a ). Only acceleration-dependent quantities are computed:
  //   - data.oa[i]: spatial acceleration of body i, without gravity,
  //   - ddJ_cols = data.oa[parent] x J_cols,
  //   - data.of[i] = data.oinertias[i] * data.oa[i].
  // The q-dependent quantities (data.oMi, data.J, data.joints[i], data.oinertias and the
  // composite inertias data.oYcrb) must be up-to-date, which is the case after
  // computeABADerivatives and ComputeRNEASecondOrderDerivatives.
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

    typedef boost::fusion::vector<const Model &, Data &, const InputAccType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<InputAccType> & a)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      Motion & oa = data.oa[i];

      if (parent > 0)
        oa = data.oa[parent];
      else
        oa.setZero(); // zero gravity: the result is (dM/dq) a, without the gravity contribution

      auto J_cols = jmodel.jointCols(data.J);
      auto ddJ_cols = jmodel.jointCols(data.ddJ);

      motionSet::motionAction(oa, J_cols, ddJ_cols);
      oa += data.oMi[i].act(jdata.S() * jmodel.jointVelocitySelector(a));

      data.of[i] = data.oinertias[i] * oa;
    }
  };

  // Backward sweep companion of ComputeABASecondOrderDerivativesInnerForwardStep. It fills
  // dMa_dq(:, k) = (dM/dq_k) a, using data.Ftmp1 and data.Ftmp2 as workspace and the composite
  // rigid body inertias data.oYcrb left by ComputeRNEASecondOrderDerivatives.
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename OutputMatrixType>
  struct ComputeABASecondOrderDerivativesInnerBackwardStep
  : public fusion::JointUnaryVisitorBase<ComputeABASecondOrderDerivativesInnerBackwardStep<
      Scalar,
      Options,
      JointCollectionTpl,
      OutputMatrixType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &, OutputMatrixType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      const Model & model,
      Data & data,
      OutputMatrixType & dMa_dq)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      auto J_cols = jmodel.jointCols(data.J);
      auto ddJ_cols = jmodel.jointCols(data.ddJ);
      auto Ftmp1_cols = jmodel.jointCols(data.Ftmp1);
      auto Ftmp2_cols = jmodel.jointCols(data.Ftmp2);

      const Eigen::Index joint_idx = (Eigen::Index)jmodel.idx_v();
      const Eigen::Index joint_dofs = (Eigen::Index)jmodel.nv();
      const Eigen::Index subtree_dofs = (Eigen::Index)data.nvSubtree[i];
      const Eigen::Index successor_idx = joint_idx + joint_dofs;
      const Eigen::Index successor_dofs = subtree_dofs - joint_dofs;

      const typename Data::Inertia & oYcrb = data.oYcrb[i];
      motionSet::inertiaAction(oYcrb, J_cols, Ftmp1_cols);
      motionSet::inertiaAction<ADDTO>(oYcrb, ddJ_cols, Ftmp2_cols);
      motionSet::act<ADDTO>(J_cols, data.of[i], Ftmp2_cols);

      if (successor_dofs > 0)
        dMa_dq.block(joint_idx, successor_idx, joint_dofs, successor_dofs).noalias() =
          J_cols.transpose() * data.Ftmp2.middleCols(successor_idx, successor_dofs);

      dMa_dq.block(joint_idx, joint_idx, subtree_dofs, joint_dofs).noalias() =
        data.Ftmp1.middleCols(joint_idx, subtree_dofs).transpose() * ddJ_cols;

      if (parent > 0)
        data.of[parent] += data.of[i];
    }
  };

  namespace internal
  {
    // Computes dMa_dq(:, k) = (dM/dq_k) a for a given joint acceleration vector a, with one
    // forward and one backward sweep (IDFOZA, Singh, Russell & Wensing, IEEE T-RO 2024,
    // Eq. (48)).
    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename InputAccType,
      typename OutputMatrixType>
    inline void computeABASecondOrderDerivativesInnerTerm(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<InputAccType> & a,
      const Eigen::MatrixBase<OutputMatrixType> & dMa_dq)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::JointIndex JointIndex;

      typedef ComputeABASecondOrderDerivativesInnerForwardStep<
        Scalar, Options, JointCollectionTpl, InputAccType>
        Pass1;
      for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        Pass1::run(
          model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, a.derived()));
      }

      OutputMatrixType & dMa_dq_ = dMa_dq.const_cast_derived();
      dMa_dq_.setZero();
      data.Ftmp2.setZero();

      typedef ComputeABASecondOrderDerivativesInnerBackwardStep<
        Scalar, Options, JointCollectionTpl, OutputMatrixType>
        Pass2;
      for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
      {
        Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data, dMa_dq_));
      }
    }

    // Applies the Outer-Term of Eq. (42) (Singh, Russell & Wensing, IEEE T-RO 2024) page by page:
    // tensor(:, :, k) <- -Minv * tensor(:, :, k). Each page is staged in page_in and page_out (of
    // size nv x nv), so that the dense products do not trigger any dynamic allocation.
    template<typename MinvType, typename TensorType, typename MatrixType>
    inline void applyMinusMinvToTensorPages(
      const Eigen::MatrixBase<MinvType> & Minv,
      TensorType & tensor,
      MatrixType & page_in,
      MatrixType & page_out)
    {
      const Eigen::Index nv = Minv.rows();
      for (Eigen::Index k = 0; k < nv; ++k)
      {
        for (Eigen::Index j = 0; j < nv; ++j)
          for (Eigen::Index i = 0; i < nv; ++i)
            page_in(i, j) = tensor(i, j, k);

        page_out.noalias() = -Minv * page_in;

        for (Eigen::Index j = 0; j < nv; ++j)
          for (Eigen::Index i = 0; i < nv; ++i)
            tensor(i, j, k) = page_out(i, j);
      }
    }
  } // namespace internal

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
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      data.allocation == DataAllocationOption::ALL,
      "data was created with DataAllocationOption::NO_TENSORS: data.d2tau_dqdq, data.d2tau_dvdv, "
      "data.d2tau_dqdv and data.d2tau_dadq, used as workspace, are not allocated");
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

    Tensor1 & d2ddq_dqdq_ = const_cast<Tensor1 &>(d2ddq_dqdq);
    Tensor2 & d2ddq_dvdv_ = const_cast<Tensor2 &>(d2ddq_dvdv);
    Tensor3 & d2ddq_dqdv_ = const_cast<Tensor3 &>(d2ddq_dqdv);
    Tensor4 & d2ddq_dtaudq_ = const_cast<Tensor4 &>(d2ddq_dtaudq);

    const Eigen::Index nv = model.nv;

    // 1) First-order derivatives of the forward dynamics: fills data.ddq, data.ddq_dq,
    //    data.ddq_dv and data.Minv (both triangles).
    computeABADerivatives(model, data, q.derived(), v.derived(), tau.derived());

    // 2) Second-order derivatives of the inverse dynamics, evaluated at (q, v, ddq). Their output
    //    tensors must be zero-initialized, as they are only written where structurally non-zero.
    data.d2tau_dqdq.setZero();
    data.d2tau_dvdv.setZero();
    data.d2tau_dqdv.setZero();
    data.d2tau_dadq.setZero();
    ComputeRNEASecondOrderDerivatives(
      model, data, q.derived(), v.derived(), data.ddq, data.d2tau_dqdq, data.d2tau_dvdv,
      data.d2tau_dqdv, data.d2tau_dadq);

    // 3) Inner-Term of Eq. (42) in Singh, Russell & Wensing, IEEE T-RO 2024, assembled in place
    //    in the output tensors:
    //      d2ddq_dqdq   <- d2tau_dqdq + (dM/dq ddq_dq) + (dM/dq ddq_dq)^R
    //      d2ddq_dvdv   <- d2tau_dvdv
    //      d2ddq_dqdv   <- d2tau_dqdv + (dM/dq ddq_dv)^R                 (Eq. (45))
    //      d2ddq_dtaudq <- (dM/dq Minv)
    //    where (dM/dq X)(:, j, k) = (dM/dq_k) X(:, j) and ^R swaps the 2nd and 3rd dimensions.
    //    Page j of (dM/dq X)^R, i.e. d/dq ( M(q) X(:, j) ), is given by one IDFOZA pass.
    for (Eigen::Index k = 0; k < nv; ++k)
      for (Eigen::Index j = 0; j < nv; ++j)
        for (Eigen::Index i = 0; i < nv; ++i)
        {
          d2ddq_dqdq_(i, j, k) = data.d2tau_dqdq(i, j, k);
          d2ddq_dvdv_(i, j, k) = data.d2tau_dvdv(i, j, k);
        }

    typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & dMa_dq = data.MXtmp1;
    for (Eigen::Index w = 0; w < nv; ++w)
    {
      internal::computeABASecondOrderDerivativesInnerTerm(model, data, data.ddq_dq.col(w), dMa_dq);
      for (Eigen::Index k = 0; k < nv; ++k)
        for (Eigen::Index i = 0; i < nv; ++i)
        {
          d2ddq_dqdq_(i, k, w) += dMa_dq(i, k);
          d2ddq_dqdq_(i, w, k) += dMa_dq(i, k);
        }

      internal::computeABASecondOrderDerivativesInnerTerm(model, data, data.ddq_dv.col(w), dMa_dq);
      for (Eigen::Index k = 0; k < nv; ++k)
        for (Eigen::Index i = 0; i < nv; ++i)
          d2ddq_dqdv_(i, k, w) = data.d2tau_dqdv(i, k, w) + dMa_dq(i, k);

      internal::computeABASecondOrderDerivativesInnerTerm(model, data, data.Minv.col(w), dMa_dq);
      for (Eigen::Index k = 0; k < nv; ++k)
        for (Eigen::Index i = 0; i < nv; ++i)
          d2ddq_dtaudq_(i, w, k) = dMa_dq(i, k);
    }

    // 4) Outer-Term of Eq. (42): multiply every page of the Inner-Term by -Minv. As shown in
    //    Fig. 9(b) of the same paper, the dense product beats the AZA recursion for practical
    //    robotics model sizes.
    internal::applyMinusMinvToTensorPages(data.Minv, d2ddq_dqdq_, data.MXtmp1, data.MXtmp2);
    internal::applyMinusMinvToTensorPages(data.Minv, d2ddq_dvdv_, data.MXtmp1, data.MXtmp2);
    internal::applyMinusMinvToTensorPages(data.Minv, d2ddq_dqdv_, data.MXtmp1, data.MXtmp2);
    internal::applyMinusMinvToTensorPages(data.Minv, d2ddq_dtaudq_, data.MXtmp1, data.MXtmp2);
  }

} // namespace pinocchio
