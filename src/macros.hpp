//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_macros_hpp__
#define __pinocchio_macros_hpp__

#if __cplusplus >= 201103L
  #define PINOCCHIO_WITH_CXX11_SUPPORT
#endif

#if __cplusplus >= 201403L
  #define PINOCCHIO_WITH_CXX14_SUPPORT
#endif

#if __cplusplus >= 201703L
  #define PINOCCHIO_WITH_CXX17_SUPPORT
#endif

#define PINOCCHIO_STRING_LITERAL(string) #string

#include <exception>

/// \brief Macro to check an assert-like condition and throw a runtime error exception (with a message) if violated.
#define PINOCCHIO_ASSERT_THROW_AT_RUNTIME(cond) if (!(cond)) { throw std::runtime_error("Wrong argument size"); }
#define PINOCCHIO_ASSERT_THROW_AT_RUNTIME_WITH_MESSAGE(cond, message) if (!(cond)) { throw std::runtime_error(message); }

// For more details, visit https://stackoverflow.com/questions/171435/portability-of-warning-preprocessor-directive
#if defined(__GNUC__) || defined(__clang__)
  #define PINOCCHIO_PRAGMA(x) _Pragma(#x)
  #define PINOCCHIO_PRAGMA_MESSAGE(the_message) PINOCCHIO_PRAGMA(GCC message #the_message)
  #define PINOCCHIO_PRAGMA_WARNING(the_message) PINOCCHIO_PRAGMA(GCC warning #the_message)
  #define PINOCCHIO_PRAGMA_DEPRECATED(the_message) PINOCCHIO_PRAGMA_WARNING(Deprecated: #the_message)
  #define PINOCCHIO_PRAGMA_DEPRECATED_HEADER(old_header,new_header) \
          PINOCCHIO_PRAGMA_WARNING(Deprecated header file: #old_header has been replaced by #new_header.\n Please use #new_header instead of #old_header.)
#endif

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
  PINOCCHIO_ASSERT_THROW_AT_RUNTIME(M.rows()==nrows && M.cols()==ncols);

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
