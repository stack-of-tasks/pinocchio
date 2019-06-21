//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_utils_string_generator_hpp__
#define __pinocchio_utils_string_generator_hpp__

#include <string>
#include <cstdlib>

namespace pinocchio
{

  ///
  /// \brief Generate a random string composed of alphanumeric symbols of a given length.
  ///
  /// \input len The length of the output string.
  ///
  /// \returns a random string composed of alphanumeric symbols.
  ///
  inline std::string randomStringGenerator(const int len)
  {
    std::string res;
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i=0; i<len;++i)
      res += alphanum[((size_t)std::rand() % (sizeof(alphanum) - 1))];
    return res;
  }
}

#endif // __pinocchio_utils_string_generator_hpp__
