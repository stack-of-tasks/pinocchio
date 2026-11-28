//
// Copyright (c) 2025-2026 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/utils/eigen-helpers.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/utils/eigen-helpers.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace internal
  {

    namespace helper
    {
      template<class T>
      struct is_eigen_noalias : std::false_type
      {
      };

      template<typename ExpressionType, template<typename> class StorageBase>
      struct is_eigen_noalias<Eigen::NoAlias<ExpressionType, StorageBase>> : std::true_type
      {
      };

      template<class T>
      inline constexpr bool is_eigen_noalias_v = is_eigen_noalias<std::decay_t<T>>::value;

      template<class T>
      struct is_eigen_product : std::false_type
      {
      };

      template<typename Lhs, typename Rhs, int Option>
      struct is_eigen_product<Eigen::Product<Lhs, Rhs, Option>> : std::true_type
      {
      };

      template<class T>
      inline constexpr bool is_eigen_product_v = is_eigen_product<std::decay_t<T>>::value;

      template<typename T>
      struct remove_eigen_noalias
      {
        typedef T type;
        static T & get(T & t)
        {
          return t;
        }
        static const T & get(const T & t)
        {
          return t;
        }
      };

      template<typename ExpressionType, template<typename> class StorageBase>
      struct remove_eigen_noalias<Eigen::NoAlias<ExpressionType, StorageBase>>
      {
        typedef ExpressionType type;
        static ExpressionType & get(Eigen::NoAlias<ExpressionType, StorageBase> & t)
        {
          return t.expression();
        }
        static const ExpressionType & get(const Eigen::NoAlias<ExpressionType, StorageBase> & t)
        {
          return t.expression();
        }
      };

      template<typename T, typename = void>
      inline constexpr bool has_fixed_rows_v = false;

      template<typename T>
      inline constexpr bool has_fixed_rows_v<T, std::void_t<decltype(T::RowsAtCompileTime)>> =
        (T::RowsAtCompileTime != Eigen::Dynamic);

      template<typename T, typename = void>
      inline constexpr bool has_fixed_cols_v = false;

      template<typename T>
      inline constexpr bool has_fixed_cols_v<T, std::void_t<decltype(T::ColsAtCompileTime)>> =
        (T::ColsAtCompileTime != Eigen::Dynamic);

      template<typename T, typename = void>
      inline constexpr bool has_fixed_size_v = false;

      template<typename T>
      inline constexpr bool has_fixed_size_v<
        T,
        std::void_t<decltype(T::RowsAtCompileTime), decltype(T::ColsAtCompileTime)>> =
        has_fixed_rows_v<T> && has_fixed_cols_v<T>;

      template<typename T>
      struct is_eigen_map : std::false_type
      {
      };

      template<typename P, int Opt, typename Stride>
      struct is_eigen_map<Eigen::Map<P, Opt, Stride>> : std::true_type
      {
      };

      template<typename P, int Opt, typename Stride>
      struct is_eigen_map<Eigen::Map<const P, Opt, Stride>> : std::true_type
      {
      };

      template<typename T>
      inline constexpr bool is_eigen_map_v = is_eigen_map<std::remove_cv_t<T>>::value;

      template<typename T>
      struct is_eigen_matrix : std::false_type
      {
      };

      // detect Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>
      template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
      struct is_eigen_matrix<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
      : std::true_type
      {
      };

      // also handle const-qualified matrices
      template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
      struct is_eigen_matrix<const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
      : std::true_type
      {
      };

      // handy variable template
      template<typename T>
      inline constexpr bool is_eigen_matrix_v = is_eigen_matrix<std::remove_cv_t<T>>::value;

    } // namespace helper

    template<typename D1, typename D2>
    bool compare_maps(const Eigen::MatrixBase<D1> & map1, const Eigen::MatrixBase<D2> & map2)
    {
      if ((map1.rows() != map2.rows()) || (map1.cols() != map2.cols()))
        return false;
      if (map1 != map2)
        return false;
      return true;
    }

  } // namespace internal
} // namespace pinocchio
