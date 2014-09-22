#ifndef __se3_skew_hpp__
#define __se3_skew_hpp__
namespace se3
{
  template <typename D>
  inline Eigen::Matrix<typename D::Scalar,3,3,D::Options>
  skew(const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    Eigen::Matrix<typename D::Scalar,3,3,D::Options> m;
    m(0,0) =  0   ;  m(0,1) = -v[2];   m(0,2) =  v[1];
    m(1,0) =  v[2];  m(1,1) =  0   ;   m(1,2) = -v[0];
    m(2,0) = -v[1];  m(2,1) =  v[0];   m(2,2) =  0   ;
    return m;
  }

  template <typename V,typename M>
  inline Eigen::Matrix<typename M::Scalar,3,M::ColsAtCompileTime,M::Options>
  cross(const Eigen::MatrixBase<V> & v,
	const Eigen::MatrixBase<M> & m)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V,3);

    Eigen::Matrix<typename M::Scalar,3,M::ColsAtCompileTime,M::Options> res (3,m.cols());
    res.row(0) = v[1]*m.row(2) - v[2]*m.row(1);
    res.row(1) = v[2]*m.row(0) - v[0]*m.row(2);
    res.row(2) = v[0]*m.row(1) - v[1]*m.row(0);
    return res;
  }
} // namespace se3
#endif // ifndef __se3_skew_hpp__
