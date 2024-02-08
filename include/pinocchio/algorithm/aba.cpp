//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/aba.hpp"

namespace pinocchio {
namespace impl {

  template const context::VectorXs & aba
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

  template const context::VectorXs & aba
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const container::aligned_vector<ForceTpl<context::Scalar, context::Options> > &);

  namespace minimal {
    template const context::VectorXs & aba
      <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    template const context::VectorXs & aba
      <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,  const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const container::aligned_vector<ForceTpl<context::Scalar, context::Options> > &);
  } // namespace minimal

  template const context::RowMatrixXs & computeMinverse
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
} // namespace impl

  template const context::RowMatrixXs & computeMinverse
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
} // namespace pinocchio
