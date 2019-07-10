//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_core_binary_op_hpp__
#define __pinocchio_core_binary_op_hpp__

namespace pinocchio
{
  ///
  /// \brief Forward declaration of the multiplication operation return type.
  ///        Should be overloaded, otherwise it will procude a compilation error.
  ///
  template<typename Lhs, typename Rhs>
  struct MultiplicationOp;
  
  namespace impl
  {
    template<typename Lhs, typename Rhs>
    struct LhsMultiplicationOp;
  }
}

#endif // ifndef __pinocchio_core_binary_op_hpp__
