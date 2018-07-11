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

#ifndef __pinocchio_unit_test_utils_macros_hpp__
#define __pinocchio_unit_test_utils_macros_hpp__

#ifdef __clang__
  #include <boost/variant.hpp> // to avoid some warning with clang
#endif

//#ifdef __clang__
//  #define BEGIN_UNIT_TEST_SECTION \
//  _Pragma("clang diagnostic push") \
//  _Pragma("clang diagnostic ignored \"-Wc99-extensions\" ")
//#else
//  #define BEGIN_UNIT_TEST_SECTION
//#endif
//
//#ifdef __clang__
//  #define END_UNIT_TEST_SECTION _Pragma("clang diagnostic pop")
//#else
//  #define END_UNIT_TEST_SECTION
//#endif

#endif // ifndef __pinocchio_unit_test_utils_macros_hpp__
