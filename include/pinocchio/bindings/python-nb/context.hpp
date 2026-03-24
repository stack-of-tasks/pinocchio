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

#define PINOCCHIO_PYTHON_NAMESPACE_BEGIN                                                           \
  namespace pinocchio::python_nb                                                                   \
  {
#define PINOCCHIO_PYTHON_NAMESPACE_END } // pinocchio::python_nb

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using Scalar = PINOCCHIO_PYTHON_SCALAR_TYPE_DEFAULT;
static constexpr int Options = 0;

// Spatial
using SE3 = SE3Tpl<Scalar, Options>;
using Motion = MotionTpl<Scalar, Options>;
using Force = ForceTpl<Scalar, Options>;
using Inertia = Inertia;

// Multibody

// Joints
using JointModel = JointModelTpl<Scalar, Options>;
PINOCCHIO_PYTHON_NAMESPACE_END
