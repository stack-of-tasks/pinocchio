//
// Copyright (c) INRIA 2026
//
#pragma once

// IWYU pragma: begin_keep
#include <limits>
#include <ostream>
#include <string>
#include <vector>
#include <type_traits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/type_traits.hpp>
#include <boost/blank.hpp>
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/container/generation/make_vector.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/variant.hpp>

#include "pinocchio/context.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/eigen-common.hpp"

#include "pinocchio/utils/static-if.hpp"

#include "pinocchio/math.hpp"
#include "pinocchio/spatial.hpp"

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/src/multibody/joint-motion-subspace-base.hxx"
#include "pinocchio/src/multibody/joint-motion-subspace-generic.hxx"

#include "pinocchio/serialization/fwd.hpp"
// IWYU pragma: end_keep

// IWYU pragma: begin_exports
#include "pinocchio/src/multibody/joint/fwd.hxx"
#include "pinocchio/src/multibody/joint/joint-model-base.hxx"
#include "pinocchio/src/multibody/joint/joint-data-base.hxx"
#include "pinocchio/src/multibody/joint/joint-common-operations.hxx"

#include "pinocchio/src/multibody/visitor/fusion.hxx"
#include "pinocchio/src/multibody/visitor/joint-unary-visitor.hxx"
#include "pinocchio/src/multibody/visitor/joint-binary-visitor.hxx"

#include "pinocchio/src/multibody/joint/joint-revolute.hxx"
#include "pinocchio/src/multibody/joint/joint-revolute-unaligned.hxx"
#include "pinocchio/src/multibody/joint/joint-revolute-unbounded.hxx"
#include "pinocchio/src/multibody/joint/joint-revolute-unbounded-unaligned.hxx"

#include "pinocchio/src/multibody/joint/joint-translation.hxx"

#include "pinocchio/src/multibody/joint/joint-prismatic.hxx"
#include "pinocchio/src/multibody/joint/joint-prismatic-unaligned.hxx"

#include "pinocchio/src/multibody/joint/joint-planar.hxx"

#include "pinocchio/src/multibody/joint/joint-ellipsoid.hxx"

#include "pinocchio/src/multibody/joint/joint-helical.hxx"
#include "pinocchio/src/multibody/joint/joint-helical-unaligned.hxx"

#include "pinocchio/src/multibody/joint/joint-spherical.hxx"
#include "pinocchio/src/multibody/joint/joint-spherical-ZYX.hxx"

#include "pinocchio/src/multibody/joint/joint-universal.hxx"

#include "pinocchio/src/multibody/joint/joint-free-flyer.hxx"

#include "pinocchio/src/multibody/joint/joint-basic-visitors.hxx"

#include "pinocchio/src/multibody/joint/joint-composite.hxx"

#include "pinocchio/src/multibody/joint/joint-mimic.hxx"

#include "pinocchio/src/multibody/joint/spline-utils.hxx"
#include "pinocchio/src/multibody/joint/joint-spline.hxx"

#include "pinocchio/src/multibody/joint/joint-collection.hxx"

#include "pinocchio/src/multibody/joint/joint-generic.hxx"
// IWYU pragma: end_exports
