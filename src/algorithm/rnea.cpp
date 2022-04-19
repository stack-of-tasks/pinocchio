//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/rnea.hpp"

namespace pinocchio {
namespace impl {
  template const context::VectorXs & rnea
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

  template const context::VectorXs & rnea
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, context::Force>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const container::aligned_vector<context::Force> &);

  template const context::VectorXs & nonLinearEffects
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

  template const context::VectorXs & computeGeneralizedGravity
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

  template const context::VectorXs & computeStaticTorque
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const container::aligned_vector<context::Force> &);

  template const context::MatrixXs & computeCoriolisMatrix
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
}  // namespace impl
  template const context::MatrixXs & getCoriolisMatrix
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
} // namespace pinocchio
