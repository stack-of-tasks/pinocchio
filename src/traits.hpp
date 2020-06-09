#ifndef __pinocchio_traits_hpp__
#define __pinocchio_traits_hpp__

namespace pinocchio
{
  ///
  /// \brief Common traits structure to fully define base classes for CRTP.
  ///
  template<class C> struct traits {};
  
  namespace internal
  {
    template<typename T> struct traits {};
  }
  
  ///
  /// \brief Type of the cast of a class C templated by Scalar and Options, to a new NewScalar type.
  ///        This class should be specialized for each types.
  ///
  template<typename NewScalar, class C> struct CastType;

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

  
  
  /// \brief Return type undefined
  ///        This is an helper structure to help internal diagnosis.
  struct ReturnTypeNotDefined;
}

#endif
