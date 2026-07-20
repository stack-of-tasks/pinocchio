//
// Copyright (c) 2018 CNRS
//

#pragma once

#ifdef __clang__
  #include <boost/variant.hpp> // to avoid some warning with clang
#endif

// #ifdef __clang__
//   #define BEGIN_UNIT_TEST_SECTION _Pragma("clang diagnostic push") _Pragma("clang diagnostic
//   ignored \"-Wc99-extensions\" ")
// #else
//   #define BEGIN_UNIT_TEST_SECTION
// #endif
//
// #ifdef __clang__
//   #define END_UNIT_TEST_SECTION _Pragma("clang diagnostic pop")
// #else
//   #define END_UNIT_TEST_SECTION
// #endif
