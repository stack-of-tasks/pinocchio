//
// Copyright (c) 2026 INRIA
//

#pragma once

#define PINOCCHIO_PYTHON_SCALAR_TYPE_DEFAULT double

// #define PINOCCHIO_PYTHON_CONTEXT_FILE_DEFAULT "pinocchio/bindings/python-nb/context/default.hpp"

// #ifndef PINOCCHIO_PYTHON_CONTEXT_FILE
//   #define PINOCCHIO_PYTHON_CONTEXT_FILE PINOCCHIO_PYTHON_CONTEXT_FILE_DEFAULT
// #endif

// #include PINOCCHIO_PYTHON_CONTEXT_FILE

#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/pool/fwd.hpp"
#include "pinocchio/constraints/fwd.hpp"

#include "./macros.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using Scalar = PINOCCHIO_PYTHON_SCALAR_TYPE_DEFAULT;
static constexpr int Options = 0;

// Eigen
using VectorXs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using MatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options>;
using RowMatrixXs =
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Options>;
using ConstVectorRef = Eigen::Ref<const VectorXs>;

// Spatial
using SE3 = SE3Tpl<Scalar, Options>;
using Motion = MotionTpl<Scalar, Options>;
using Force = ForceTpl<Scalar, Options>;
using Inertia = InertiaTpl<Scalar, Options>;
using PseudoInertia = PseudoInertiaTpl<Scalar, Options>;
using LogCholeskyParameters = LogCholeskyParametersTpl<Scalar, Options>;

// Multibody
using Model = ModelTpl<Scalar, Options>;
using Data = DataTpl<Scalar, Options>;
using ModelPool = ModelPoolTpl<Scalar, Options>;
using GeometryPool = GeometryPoolTpl<Scalar, Options>;

// Joints
using JointModel = JointModelTpl<Scalar, Options>;
using JointData = JointDataTpl<Scalar, Options>;

// Constraints
using BaumgarteCorrectorParameters = BaumgarteCorrectorParametersTpl<Scalar>;
using RigidConstraintModel = RigidConstraintModelTpl<Scalar, Options>;
using RigidConstraintData = RigidConstraintDataTpl<Scalar, Options>;
using RigidConstraintModelVector = std::vector<RigidConstraintModel>;
using RigidConstraintDataVector = std::vector<RigidConstraintData>;
using PointContactConstraintModel = PointContactConstraintModelTpl<Scalar, Options>;
using PointContactConstraintData = PointContactConstraintDataTpl<Scalar, Options>;
using PointAnchorConstraintModel = PointAnchorConstraintModelTpl<Scalar, Options>;
using PointAnchorConstraintData = PointAnchorConstraintDataTpl<Scalar, Options>;
using FrameAnchorConstraintModel = FrameAnchorConstraintModelTpl<Scalar, Options>;
using FrameAnchorConstraintData = FrameAnchorConstraintDataTpl<Scalar, Options>;
using JointFrictionConstraintModel = JointFrictionConstraintModelTpl<Scalar, Options>;
using JointFrictionConstraintData = JointFrictionConstraintDataTpl<Scalar, Options>;
using JointLimitConstraintModel = JointLimitConstraintModelTpl<Scalar, Options>;
using JointLimitConstraintData = JointLimitConstraintDataTpl<Scalar, Options>;
using ConstraintModel = ConstraintModelTpl<Scalar, Options>;
using ConstraintData = ConstraintDataTpl<Scalar, Options>;
using ConstraintModelVector = std::vector<ConstraintModel>;
using ConstraintDataVector = std::vector<ConstraintData>;

// Algorithm
using ProximalSettings = ProximalSettingsTpl<Scalar>;
using ConstraintCholeskyDecomposition = ConstraintCholeskyDecompositionTpl<Scalar, Options>;
PINOCCHIO_PYTHON_NAMESPACE_END
