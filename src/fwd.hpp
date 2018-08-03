//
// Copyright (c) 2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_fwd_hpp__
#define __se3_fwd_hpp__


#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT

  #ifdef PINOCCHIO_CPPAD_REQUIRES_MATRIX_BASE_PLUGIN
    #define EIGEN_MATRIXBASE_PLUGIN <cppad/example/cppad_eigen.hpp>
  #endif

  #include <Eigen/Core>
  #include <cppad/cppad.hpp>

#endif

#endif // #ifndef __se3_fwd_hpp__
