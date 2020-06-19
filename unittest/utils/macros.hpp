//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_unit_test_utils_macros_hpp__
#define __pinocchio_unit_test_utils_macros_hpp__

#ifdef __clang__
  #include <boost/variant.hpp> // to avoid some warning with clang
#endif

//#ifdef __clang__
//  #define BEGIN_UNIT_TEST_SECTION _Pragma("clang diagnostic push") _Pragma("clang diagnostic ignored \"-Wc99-extensions\" ")
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
