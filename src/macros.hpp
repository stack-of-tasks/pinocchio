//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_macros_hpp__
#define __pinocchio_macros_hpp__

// On Windows, __cplusplus is not necessarily set to the C++ version being used.
// See https://docs.microsoft.com/fr-fr/cpp/build/reference/zc-cplusplus?view=vs-2019 for further information.

#if (__cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703))
  #define PINOCCHIO_WITH_CXX17_SUPPORT
#endif

#if (__cplusplus >= 201403L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201403))
  #define PINOCCHIO_WITH_CXX14_SUPPORT
#endif

#if (__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
  #define PINOCCHIO_WITH_CXX11_SUPPORT
#endif

#define PINOCCHIO_STRING_LITERAL(string) #string

// For more details, visit https://stackoverflow.com/questions/171435/portability-of-warning-preprocessor-directive
#if defined(__GNUC__) || defined(__clang__)
  #define PINOCCHIO_PRAGMA(x) _Pragma(#x)
  #define PINOCCHIO_PRAGMA_MESSAGE(the_message) PINOCCHIO_PRAGMA(GCC message #the_message)
  #define PINOCCHIO_PRAGMA_WARNING(the_message) PINOCCHIO_PRAGMA(GCC warning #the_message)
  #define PINOCCHIO_PRAGMA_DEPRECATED(the_message) PINOCCHIO_PRAGMA_WARNING(Deprecated: #the_message)
  #define PINOCCHIO_PRAGMA_DEPRECATED_HEADER(old_header,new_header) \
          PINOCCHIO_PRAGMA_WARNING(Deprecated header file: #old_header has been replaced by #new_header.\n Please use #new_header instead of #old_header.)
#endif

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

// Handle explicitely the GCC borring warning: 'anonymous variadic macros were introduced in C++11'
#include <exception>
#include <stdexcept>

#if defined(__GNUC__)
  #pragma GCC system_header
#endif

#if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wvariadic-macros"
#endif

/// \brief Generic macro to throw an exception in Pinocchio if the condition is not met with a given input message.
#if !defined(PINOCCHIO_NO_THROW)
  #define PINOCCHIO_THROW(condition,exception_type,message) \
    if (!(condition)) { throw exception_type(PINOCCHIO_STRING_LITERAL(message)); }
#else
  #define PINOCCHIO_THROW(condition,exception_type,message)
#endif

#define _PINOCCHIO_GET_OVERRIDE_FOR_CHECK_INPUT_ARGUMENT(_1, _2, MACRO_NAME, ...) MACRO_NAME

#define _PINOCCHIO_CHECK_INPUT_ARGUMENT_2(condition, message) \
  PINOCCHIO_THROW(condition,std::invalid_argument,PINOCCHIO_STRING_LITERAL(message))

#define _PINOCCHIO_CHECK_INPUT_ARGUMENT_1(condition) \
  _PINOCCHIO_CHECK_INPUT_ARGUMENT_2(condition,\
                                    "The following check on the input argument has failed: "#condition)

#define _PINOCCHIO_CHECK_INPUT_ARGUMENT_0

/// \brief Macro to check an assert-like condition and throw a std::invalid_argument exception (with a message) if violated.
#define PINOCCHIO_CHECK_INPUT_ARGUMENT(...) \
  _PINOCCHIO_GET_OVERRIDE_FOR_CHECK_INPUT_ARGUMENT(__VA_ARGS__,_PINOCCHIO_CHECK_INPUT_ARGUMENT_2,\
  _PINOCCHIO_CHECK_INPUT_ARGUMENT_1,_PINOCCHIO_CHECK_INPUT_ARGUMENT_0)(__VA_ARGS__)

#define _PINOCCHIO_GET_OVERRIDE_FOR_CHECK_ARGUMENT_SIZE(_1, _2, _3, MACRO_NAME, ...) MACRO_NAME

#define _PINOCCHIO_CHECK_ARGUMENT_SIZE_5(size, expected_size, size_literal, expected_size_literal, message) \
  if (size != expected_size) { \
    std::ostringstream oss; \
    oss << "wrong argument size: expected " << expected_size << ", got " << size << std::endl; \
    oss << "hint: "; \
    if(!std::string(message).empty()) \
      oss << message << std::endl; \
    else \
      oss << size_literal << " is different from " << expected_size_literal << std::endl; \
    PINOCCHIO_THROW(true, std::invalid_argument, oss.str()); \
  }

#define _PINOCCHIO_CHECK_ARGUMENT_SIZE_3(size, expected_size, message) \
  _PINOCCHIO_CHECK_ARGUMENT_SIZE_5(size, expected_size, PINOCCHIO_STRING_LITERAL(size), PINOCCHIO_STRING_LITERAL(expected_size), PINOCCHIO_STRING_LITERAL(message))

#define _PINOCCHIO_CHECK_ARGUMENT_SIZE_2(size, expected_size) \
  _PINOCCHIO_CHECK_ARGUMENT_SIZE_5(size, expected_size, PINOCCHIO_STRING_LITERAL(size), PINOCCHIO_STRING_LITERAL(expected_size), "")

#define _PINOCCHIO_CHECK_ARGUMENT_SIZE_1

/// \brief Macro to check if the size of an element is equal to the expected size.
#define PINOCCHIO_CHECK_ARGUMENT_SIZE(...) \
  _PINOCCHIO_GET_OVERRIDE_FOR_CHECK_ARGUMENT_SIZE(__VA_ARGS__,_PINOCCHIO_CHECK_ARGUMENT_SIZE_3, \
  _PINOCCHIO_CHECK_ARGUMENT_SIZE_2, _PINOCCHIO_CHECK_ARGUMENT_SIZE_1)(__VA_ARGS__)

#if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic pop
#endif

#endif // ifndef __pinocchio_macros_hpp__
