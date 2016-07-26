//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_string_generator_hpp__
#define __se3_string_generator_hpp__

#include <string>
#include <cstdlib>

namespace se3
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

#endif // __se3_string_generator_hpp__
