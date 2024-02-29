//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"

namespace pinocchio {

  template void integrate
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void integrate
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void interpolate
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &, const Eigen::MatrixBase<context::VectorXs> &);

  template void interpolate
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &, const Eigen::MatrixBase<context::VectorXs> &);

  template void difference
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void difference
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &,const Eigen::MatrixBase<context::VectorXs> &);

  template void squaredDistance
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void squaredDistance
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void randomConfiguration
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void randomConfiguration
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void neutral
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &);

  template void neutral
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &);

  template void dIntegrate
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition, const AssignmentOperatorType);

  template void dIntegrate
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition);

  template void dIntegrate
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition, const AssignmentOperatorType);

  template void dIntegrateTransport
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition);

  template void dIntegrateTransport
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition);

  template void dIntegrateTransport
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition);

  template void dDifference
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition);

  template void dDifference
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const ArgumentPosition);

  template context::Scalar squaredDistanceSum
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::Scalar squaredDistanceSum
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::Scalar distance
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::Scalar distance
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void normalize
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &);

  template void normalize
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &);

#ifndef PINOCCHIO_SKIP_CASADI_UNSUPPORTED

  template bool isNormalized
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &);

  template bool isNormalized
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &);

  template bool isSameConfiguration
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &);

  template bool isSameConfiguration
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &);

  template void integrateCoeffWiseJacobian
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &);

  template void integrateCoeffWiseJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::MatrixXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &);

#endif // PINOCCHIO_SKIP_CASADI_UNSUPPORTED

  template context::VectorXs integrate
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs integrate
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs interpolate
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &);

  template context::VectorXs interpolate
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::Scalar &);

  template context::VectorXs difference
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs difference
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs squaredDistance
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs squaredDistance
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs randomConfiguration
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs randomConfiguration
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const context::Model &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template context::VectorXs randomConfiguration
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &);

  template context::VectorXs randomConfiguration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &);

  template context::VectorXs neutral
    <LieGroupMap, context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &);

  template context::VectorXs neutral
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &);
} // namespace pinocchio 
