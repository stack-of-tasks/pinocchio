//
// Copyright (c) 2015-2018 CNRS INRIA
//

#ifndef __pinocchio_skew_hpp__
#define __pinocchio_skew_hpp__

#include "pinocchio/macros.hpp"

namespace pinocchio
{
  
  ///
  /// \brief Computes the skew representation of a given 3d vector,
  ///        i.e. the antisymmetric matrix representation of the cross product operator (\f$ [v]_{\times} x = v \times x \f$)
  ///
  /// \param[in]  v a vector of dimension 3.
  /// \param[out] M the skew matrix representation of dimension 3x3.
  ///
  template <typename Vector3, typename Matrix3>
  inline void skew(const Eigen::MatrixBase<Vector3> & v,
                   const Eigen::MatrixBase<Matrix3> & M)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
    
    Matrix3 & M_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,M);
    typedef typename Matrix3::RealScalar Scalar;
    
    M_(0,0) = Scalar(0);  M_(0,1) = -v[2];      M_(0,2) = v[1];
    M_(1,0) = v[2];       M_(1,1) = Scalar(0);  M_(1,2) = -v[0];
    M_(2,0) = -v[1];      M_(2,1) = v[0];       M_(2,2) = Scalar(0);
  }
  
  ///
  /// \brief Computes the skew representation of a given 3D vector,
  ///        i.e. the antisymmetric matrix representation of the cross product operator.
  ///
  /// \param[in] v a vector of dimension 3.
  ///
  /// \return The skew matrix representation of v.
  ///
  template <typename D>
  inline Eigen::Matrix<typename D::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(D)::Options>
  skew(const Eigen::MatrixBase<D> & v)
  {
    Eigen::Matrix<typename D::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(D)::Options> M;
    skew(v,M);
    return M;
  }
  
  ///
  /// \brief Add skew matrix represented by a 3d vector to a given matrix,
  ///        i.e. add the antisymmetric matrix representation of the cross product operator (\f$ [v]_{\times} x = v \times x \f$)
  ///
  /// \param[in]  v a vector of dimension 3.
  /// \param[out] M the 3x3 matrix to which the skew matrix is added.
  ///
  template <typename Vector3Like, typename Matrix3Like>
  inline void addSkew(const Eigen::MatrixBase<Vector3Like> & v,
                      const Eigen::MatrixBase<Matrix3Like> & M)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like,M,3,3);
    
    Matrix3Like & M_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,M);
    
                          M_(0,1) -= v[2];      M_(0,2) += v[1];
    M_(1,0) += v[2];                            M_(1,2) -= v[0];
    M_(2,0) -= v[1];      M_(2,1) += v[0];                     ;
  }
  
  ///
  /// \brief Inverse of skew operator. From a given skew-symmetric matrix M
  ///        of dimension 3x3, it extracts the supporting vector, i.e. the entries of M.
  ///        Mathematically speacking, it computes \f$ v \f$ such that \f$ M x = v \times x \f$.
  ///
  /// \param[in]  M a 3x3 skew symmetric matrix.
  /// \param[out] v the 3d vector representation of M.
  ///
  template <typename Matrix3, typename Vector3>
  inline void unSkew(const Eigen::MatrixBase<Matrix3> & M,
                     const Eigen::MatrixBase<Vector3> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
    assert((M + M.transpose()).isMuchSmallerThan(M));
    
    Vector3 & v_ = PINOCCHIO_EIGEN_CONST_CAST(Vector3,v);
    typedef typename Vector3::RealScalar Scalar;
    
    v_[0] = Scalar(0.5) * (M(2,1) - M(1,2));
    v_[1] = Scalar(0.5) * (M(0,2) - M(2,0));
    v_[2] = Scalar(0.5) * (M(1,0) - M(0,1));
  }
  
  ///
  /// \brief Inverse of skew operator. From a given skew-symmetric matrix M
  ///        of dimension 3x3, it extracts the supporting vector, i.e. the entries of M.
  ///        Mathematically speacking, it computes \f$ v \f$ such that \f$ M x = v \times x \f$.
  ///
  /// \param[in] M a 3x3 matrix.
  ///
  /// \return The vector entries of the skew-symmetric matrix.
  ///
  template <typename Matrix3>
  inline Eigen::Matrix<typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3)::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3)::Options>
  unSkew(const Eigen::MatrixBase<Matrix3> & M)
  {
    Eigen::Matrix<typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3)::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3)::Options> v;
    unSkew(M,v);
    return v;
  }

  ///
  /// \brief Computes the skew representation of a given 3d vector multiplied by a given scalar.
  ///        i.e. the antisymmetric matrix representation of the cross product operator (\f$ [\alpha v]_{\times} x = \alpha v \times x \f$)
  ///
  /// \param[in]  alpha a real scalar.
  /// \param[in]  v a vector of dimension 3.
  /// \param[out] M the skew matrix representation of dimension 3x3.
  ///
  template <typename Scalar, typename Vector3, typename Matrix3>
  void alphaSkew(const Scalar alpha,
                 const Eigen::MatrixBase<Vector3> & v,
                 const Eigen::MatrixBase<Matrix3> & M)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
    
    Matrix3 & M_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,M);
    typedef typename Matrix3::RealScalar RealScalar;
    
    M_(0,0) = RealScalar(0);  M_(0,1) = -v[2] * alpha;  M_(0,2) = v[1] * alpha;
    M_(1,0) = -M_(0,1);       M_(1,1) = RealScalar(0);  M_(1,2) = -v[0] * alpha;
    M_(2,0) = -M_(0,2);       M_(2,1) = -M_(1,2);       M_(2,2) = RealScalar(0);
  }
  
  ///
  /// \brief Computes the skew representation of a given 3d vector multiplied by a given scalar.
  ///        i.e. the antisymmetric matrix representation of the cross product operator (\f$ [\alpha v]_{\times} x = \alpha v \times x \f$)
  ///
  /// \param[in]  alpha a real scalar.
  /// \param[in]  v a vector of dimension 3.
  ///
  /// \returns the skew matrix representation of \f$ \alpha v \f$.
  ///
  template <typename Scalar, typename Vector3>
  inline Eigen::Matrix<typename Vector3::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3)::Options>
  alphaSkew(const Scalar alpha,
            const Eigen::MatrixBase<Vector3> & v)
  {
    Eigen::Matrix<typename Vector3::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3)::Options> M;
    alphaSkew(alpha,v,M);
    return M;
  }
  
  ///
  /// \brief Computes the square cross product linear operator C(u,v) such that for any vector w, \f$ u \times ( v \times w ) = C(u,v) w \f$.
  ///
  /// \param[in]  u a 3 dimensional vector.
  /// \param[in]  v a 3 dimensional vector.
  /// \param[out] C the skew square matrix representation of dimension 3x3.
  ///
  template <typename V1, typename V2, typename Matrix3>
  inline void skewSquare(const Eigen::MatrixBase<V1> & u,
                         const Eigen::MatrixBase<V2> & v,
                         const Eigen::MatrixBase<Matrix3> & C)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V1,3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V2,3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
    
    Matrix3 & C_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,C);
    typedef typename Matrix3::RealScalar Scalar;

    C_.noalias() = v*u.transpose();
    const Scalar udotv(u.dot(v));
    C_.diagonal().array() -= udotv;
  }
  
  ///
  /// \brief Computes the square cross product linear operator C(u,v) such that for any vector w, \f$ u \times ( v \times w ) = C(u,v) w \f$.
  ///
  /// \param[in] u A 3 dimensional vector.
  /// \param[in] v A 3 dimensional vector.
  ///
  /// \return The square cross product matrix skew[u] * skew[v].
  ///
  template <typename V1, typename V2>
  inline Eigen::Matrix<typename V1::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(V1)::Options>
  skewSquare(const Eigen::MatrixBase<V1> & u,
             const Eigen::MatrixBase<V2> & v)
  {
    
    Eigen::Matrix<typename V1::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(V1)::Options> M;
    skewSquare(u,v,M);
    return M;
  }
  
  ///
  /// \brief Applies the cross product onto the columns of M.
  ///
  /// \param[in] v      a vector of dimension 3.
  /// \param[in] Min    a 3 rows matrix.
  /// \param[out] Mout  a 3 rows matrix.
  ///
  /// \return the results of \f$ Mout = [v]_{\times} Min \f$.
  ///
  template <typename Vector3, typename Matrix3xIn, typename Matrix3xOut>
  inline void cross(const Eigen::MatrixBase<Vector3> & v,
                    const Eigen::MatrixBase<Matrix3xIn> & Min,
                    const Eigen::MatrixBase<Matrix3xOut> & Mout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
    EIGEN_STATIC_ASSERT(Matrix3xIn::RowsAtCompileTime==3,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Matrix3xOut::RowsAtCompileTime==3,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    
    Matrix3xOut & Mout_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut,Mout);
    
    Mout_.row(0) = v[1]*Min.row(2) - v[2]*Min.row(1);
    Mout_.row(1) = v[2]*Min.row(0) - v[0]*Min.row(2);
    Mout_.row(2) = v[0]*Min.row(1) - v[1]*Min.row(0);
  }

  ///
  /// \brief Applies the cross product onto the columns of M.
  ///
  /// \param[in] v a vector of dimension 3.
  /// \param[in] M a 3 rows matrix.
  ///
  /// \return the results of \f$ [v]_{\times} M \f$.
  ///
  template <typename Vector3, typename Matrix3x>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3x)
  cross(const Eigen::MatrixBase<Vector3> & v,
        const Eigen::MatrixBase<Matrix3x> & M)
  {
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3x) res(3,M.cols());
    cross(v,M,res);
    return res;
  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_skew_hpp__
