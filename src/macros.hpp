//
// Copyright (c) 2017-2018 CNRS INRIA
//

#ifndef __pinocchio_macros_hpp__
#define __pinocchio_macros_hpp__

#if __cplusplus >= 201103L
  #define PINOCCHIO_WITH_CXX11_SUPPORT
#endif

#define PINOCCHIO_STRING_LITERAL(string) #string

/// \remarks The following two macros should be adapted for WIN32
#define PINOCCHIO_PRAGMA_MESSAGE(the_message) PINOCCHIO_STRING_LITERAL(message(the_message))
#define PINOCCHIO_PRAGMA_MESSAGE_CALL(the_message) _Pragma(PINOCCHIO_PRAGMA_MESSAGE(the_message))

/// \brief Macro to check the current Pinocchio version against a version provided by x.y.z
#define PINOCCHIO_VERSION_AT_LEAST(x,y,z) \
          (PINOCCHIO_MAJOR_VERSION>x || (PINOCCHIO_MAJOR_VERSION>=x && \
          (PINOCCHIO_MINOR_VERSION>y || (PINOCCHIO_MINOR_VERSION>=y && \
          PINOCCHIO_PATCH_VERSION>=z))))

// This macro can be used to prevent from macro expansion, similarly to EIGEN_NOT_A_MACRO
#define PINOCCHIO_NOT_A_MACRO

namespace pinocchio
{
  namespace helper
  {
    template<typename T> struct argument_type;
    template<typename T, typename U> struct argument_type<T(U)> { typedef U type; };
  }
}

/// \brief Empty macro argument
#define PINOCCHIO_MACRO_EMPTY_ARG

/// \brief Helper to declare that a parameter is unused
#define PINOCCHIO_UNUSED_VARIABLE(var) (void)(var)

/// Ensure that a matrix (or vector) is of correct size (compile-time and run-time assertion)
#define PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(type,M,nrows,ncols)              \
  EIGEN_STATIC_ASSERT(   (type::RowsAtCompileTime == Eigen::Dynamic || type::RowsAtCompileTime == nrows) \
                      && (type::ColsAtCompileTime == Eigen::Dynamic || type::ColsAtCompileTime == ncols),\
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);    \
  assert(M.rows()==nrows && M.cols()==ncols);

/// Static assertion.
/// \param condition a boolean convertible expression
/// \param msg a valid C++ variable name.
#define PINOCCHIO_STATIC_ASSERT(condition,msg)                                 \
  { int msg[(condition) ? 1 : -1]; /*avoid unused-variable warning*/ (void) msg; }

namespace pinocchio
{
  namespace helper
  {
    template<typename D, template<typename> class TypeAccess>
    struct handle_return_type_without_typename
    {
      typedef typename TypeAccess< typename argument_type<void(D)>::type >::type type;
    };
  }
}

#endif // ifndef __pinocchio_macros_hpp__
