//
// Copyright (c) 2017-2018 CNRS
//

#ifndef __pinocchio_utils_eigen_fix_hpp__
#define __pinocchio_utils_eigen_fix_hpp__

#if EIGEN_VERSION_AT_LEAST(3,2,90) && !EIGEN_VERSION_AT_LEAST(3,3,0)
namespace pinocchio
{
  namespace internal
  {
    /// \brief Fix issue concerning 3.2.90 and more versions of Eigen that do not define size_of_xpr_at_compile_time structure.
    template<typename XprType> struct size_of_xpr_at_compile_time
    {
      enum { ret = Eigen::internal::size_at_compile_time<Eigen::internal::traits<XprType>::RowsAtCompileTime,Eigen::internal::traits<XprType>::ColsAtCompileTime>::ret };
    };
  }
}
#endif

namespace Eigen
{
  namespace internal
  {
    namespace fix
    {
      /* plain_matrix_type_row_major : same as plain_matrix_type but guaranteed to be row-major
       */
      template<typename T> struct plain_matrix_type_row_major
      {
        enum { Rows = traits<T>::RowsAtCompileTime,
               Cols = traits<T>::ColsAtCompileTime,
               MaxRows = traits<T>::MaxRowsAtCompileTime,
               MaxCols = traits<T>::MaxColsAtCompileTime
        };
        typedef Matrix<typename traits<T>::Scalar,
                      Rows,
                      Cols,
                      (MaxCols==1&&MaxRows!=1) ? ColMajor : RowMajor,
                      MaxRows,
                      MaxCols
                > type;
      };
    }
  }
} // namespace fix

#endif // ifndef __pinocchio_utils_eigen_fix_hpp__

