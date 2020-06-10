//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_fwd_hpp__
#define __pinocchio_fwd_hpp__

// Forward declaration of the main pinocchio namespace
namespace pinocchio {

  /// \brief Argument position.
  ///        Used as template parameter to refer to an argument.
  enum ArgumentPosition
  {
    ARG0 = 0,
    ARG1 = 1,
    ARG2 = 2,
    ARG3 = 3,
    ARG4 = 4
  };
  
  enum AssignmentOperatorType
  {
    SETTO,
    ADDTO,
    RMTO
  };
}

#include "pinocchio/macros.hpp"
#include "pinocchio/deprecation.hpp"
#include "pinocchio/warning.hpp"
#include "pinocchio/config.hpp"

#include "pinocchio/utils/helpers.hpp"
#include "pinocchio/utils/cast.hpp"

#include "pinocchio/container/boost-container-limits.hpp"

#include <Eigen/Core>

#include "pinocchio/eigen-macros.hpp"

#include "pinocchio/core/binary-op.hpp"
#include "pinocchio/core/unary-op.hpp"

#include <cstddef> // std::size_t

#include "pinocchio/traits.hpp"

#endif // #ifndef __pinocchio_fwd_hpp__
