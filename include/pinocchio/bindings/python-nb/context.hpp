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

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/pool/fwd.hpp"

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

// Multibody
using Model = ModelTpl<Scalar, Options>;
using Data = DataTpl<Scalar, Options>;
using ModelPool = ModelPoolTpl<Scalar, Options>;
using GeometryPool = GeometryPoolTpl<Scalar, Options>;

// Joints
using JointModel = JointModelTpl<Scalar, Options>;
using JointData = JointDataTpl<Scalar, Options>;
PINOCCHIO_PYTHON_NAMESPACE_END
