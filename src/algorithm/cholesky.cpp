//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/cholesky.hpp"

namespace pinocchio {
    namespace cholesky {

    template const context::MatrixXs & decompose
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, context::Data &);

    template context::MatrixXs & solve
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs Mv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & Mv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & UDUtv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & Uv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & Utv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & Uiv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & Utiv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template context::MatrixXs & computeMinv
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>
    (const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);
    } // namespace cholesky
} // namespace pinocchio
