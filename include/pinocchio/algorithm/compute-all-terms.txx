//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_compute_all_terms_txx__
#define __pinocchio_algorithm_compute_all_terms_txx__

namespace pinocchio
{
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeAllTerms<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
  } // namespace impl
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_compute_all_terms_txx__
