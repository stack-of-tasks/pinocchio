//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_utils_shared_ptr_hpp__
#define __pinocchio_utils_shared_ptr_hpp__

#include <memory>

namespace pinocchio
{

  /// \brief Compares two std::shared_ptr
  ///
  template<typename T>
  bool compare_shared_ptr(const std::shared_ptr<T> & ptr1, const std::shared_ptr<T> & ptr2)
  {
    if (ptr1 == ptr2)
      return true;
    if (ptr1 && ptr2)
      return *ptr1.get() == *ptr2.get();
    return false;
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_utils_shared_ptr_hpp__
