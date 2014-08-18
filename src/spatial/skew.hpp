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
} // namespace se3
