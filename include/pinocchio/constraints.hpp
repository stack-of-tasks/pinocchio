//
// Copyright (c) INRIA 2026
//

#pragma once

// IWYU pragma: begin_keep
#include <cassert>
#include <cstddef>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <limits>
#include <utility>
#include <vector>
#include <type_traits>
#include <algorithm>
#include <list>

#include <boost/blank.hpp>
#include <boost/core/ref.hpp>
#include <boost/fusion/container/generation/make_vector.hpp>
#include <boost/fusion/container/vector.hpp>
#include <boost/variant.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/contains.hpp>

#include <Eigen/Core>

#include "pinocchio/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/unsupported.hpp"
#include "pinocchio/eigen-common.hpp"
#include "pinocchio/context.hpp"

#include "pinocchio/utils/check.hpp"
#include "pinocchio/utils/static-if.hpp"
#include "pinocchio/utils/reference.hpp"
#include "pinocchio/utils/std-vector.hpp"

#include "pinocchio/common.hpp"

#include "pinocchio/container/eigen-storage.hpp"
#include "pinocchio/container/matrix-stack.hpp"

#include "pinocchio/math.hpp"
#include "pinocchio/spatial.hpp"
#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/serialization/serializable.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/check-model.hpp"
// IWYU pragma: end_keep

// IWYU pragma: begin_exports
#include "pinocchio/src/constraints/fwd.hxx"

#include "pinocchio/src/constraints/sets/set-base.hxx"
#include "pinocchio/src/constraints/sets/box-set.hxx"

#include "pinocchio/src/constraints/sets/cone-base.hxx"
#include "pinocchio/src/constraints/sets/coulomb-friction-cone.hxx"
#include "pinocchio/src/constraints/sets/zero-cone.hxx"
#include "pinocchio/src/constraints/sets/zero-cone-jordan-operation.hxx"
#include "pinocchio/src/constraints/sets/second-order-cone-jordan-operation.hxx"
#include "pinocchio/src/constraints/sets/orthant-cone.hxx"
#include "pinocchio/src/constraints/sets/orthant-cone-jordan-operation.hxx"
#include "pinocchio/src/constraints/sets/full-space-cone.hxx"

#include "pinocchio/src/constraints/baumgarte-corrector-parameters.hxx"

#include "pinocchio/src/constraints/blank-constraint.hxx"
#include "pinocchio/src/constraints/constraint-model-common-parameters.hxx"

#include "pinocchio/src/constraints/constraint-model-base.hxx"
#include "pinocchio/src/constraints/constraint-data-base.hxx"

#include "pinocchio/src/constraints/visitors/constraint-model-visitor.hxx"

#include "pinocchio/src/constraints/constraint-data-generic.hxx"
#include "pinocchio/src/constraints/constraint-model-generic.hxx"

#include "pinocchio/src/constraints/kinematics-constraint-model-base.hxx"
#include "pinocchio/src/constraints/binary-kinematics-constraint-model-base.hxx"

#include "pinocchio/src/constraints/point-constraint-data-base.hxx"
#include "pinocchio/src/constraints/point-constraint-model-base.hxx"
#include "pinocchio/src/constraints/point-anchor-constraint.hxx"
#include "pinocchio/src/constraints/point-contact-constraint.hxx"
#include "pinocchio/src/constraints/constant-length-constraint.hxx"

#include "pinocchio/src/constraints/frame-constraint-data-base.hxx"
#include "pinocchio/src/constraints/frame-constraint-model-base.hxx"
#include "pinocchio/src/constraints/frame-anchor-constraint.hxx"

#include "pinocchio/src/constraints/jointwise-constraint-model-base.hxx"
#include "pinocchio/src/constraints/joint-friction-constraint.hxx"
#include "pinocchio/src/constraints/joint-limit-constraint.hxx"

#include "pinocchio/src/constraints/constraint-collection-default.hxx"

#include "pinocchio/src/constraints/constraint-ordering.hxx"
#include "pinocchio/src/constraints/contact-info.hxx"
#include "pinocchio/src/constraints/rigid-constraint-conversion.hxx"

#include "pinocchio/src/constraints/utils.hxx"
// IWYU pragma: end_exports
