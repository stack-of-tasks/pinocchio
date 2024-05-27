//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_utils_check_hpp__
#define __pinocchio_utils_check_hpp__

#include <boost/type_traits/is_floating_point.hpp>

namespace pinocchio
{

  template<
    typename Scalar,
    bool default_value = true,
    bool is_real_valued = boost::is_floating_point<Scalar>::value>
  struct check_expression_if_real_valued
  {
    static bool run(const void *)
    {
      return default_value;
    }
  };

  template<typename Scalar, bool default_value>
  struct check_expression_if_real_valued<Scalar, default_value, true>
  {
    static bool run(const void * expression_ptr)
    {
      return *static_cast<const bool *>(expression_ptr);
    }
  };

  template<typename Scalar, typename Any>
  bool check_expression_if_real(const Any & expression)
  {
    return check_expression_if_real_valued<Scalar>::run(static_cast<const void *>(&expression));
  }

  template<typename Scalar, bool default_value, typename Any>
  bool check_expression_if_real(const Any & expression)
  {
    return check_expression_if_real_valued<Scalar, default_value>::run(
      static_cast<const void *>(&expression));
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_utils_check_hpp__
