//
// Copyright (c) 2014-2019 CNRS INRIA
//

#ifndef __pinocchio_symmetric3_hpp__
#define __pinocchio_symmetric3_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/math/matrix.hpp"

namespace pinocchio
{

  template<typename _Scalar, int _Options>
  class Symmetric3Tpl
  {
  public:
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,2,2,Options> Matrix2;
    typedef Eigen::Matrix<Scalar,3,2,Options> Matrix32;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:    
    Symmetric3Tpl(): m_data() {}

    template<typename Sc,int Opt>
    explicit Symmetric3Tpl(const Eigen::Matrix<Sc,3,3,Opt> & I)
    {
      assert( (I-I.transpose()).isMuchSmallerThan(I) );
      m_data(0) = I(0,0);
      m_data(1) = I(1,0); m_data(2) = I(1,1);
      m_data(3) = I(2,0); m_data(4) = I(2,1); m_data(5) = I(2,2);
    }
    
    explicit Symmetric3Tpl(const Vector6 & I) : m_data(I) {}
    
    Symmetric3Tpl(const Scalar & a0, const Scalar & a1, const Scalar & a2,
		  const Scalar & a3, const Scalar & a4, const Scalar & a5)
    { m_data << a0,a1,a2,a3,a4,a5; }

    static Symmetric3Tpl Zero()     { return Symmetric3Tpl(Vector6::Zero());  }
    void setZero() { m_data.setZero(); }
    
    static Symmetric3Tpl Random()   { return RandomPositive();  }
    void setRandom()
    {
      Scalar
      a = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      b = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      c = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      d = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      e = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      f = Scalar(std::rand())/RAND_MAX*2.0-1.0;
      
      m_data << a, b, c, d, e, f;
    }
    
    static Symmetric3Tpl Identity() { return Symmetric3Tpl(1, 0, 1, 0, 0, 1);  }
    void setIdentity() { m_data << 1, 0, 1, 0, 0, 1; }

    /* Required by Inertia::operator== */
    bool operator==(const Symmetric3Tpl & other) const
    { return m_data == other.m_data; }
    
    bool operator!=(const Symmetric3Tpl & other) const
    { return !(*this == other); }
    
    bool isApprox(const Symmetric3Tpl & other,
                  const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return m_data.isApprox(other.m_data,prec); }
    
    bool isZero(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return m_data.isZero(prec); }
    
    void fill(const Scalar value) { m_data.fill(value); }
    
    struct SkewSquare
    {
      const Vector3 & v;
      SkewSquare( const Vector3 & v ) : v(v) {}
      operator Symmetric3Tpl () const 
      {
        const Scalar & x = v[0], & y = v[1], & z = v[2];
        return Symmetric3Tpl( -y*y-z*z,
                             x*y    ,  -x*x-z*z,
                             x*z    ,   y*z    ,  -x*x-y*y );
      }
    }; // struct SkewSquare
    
    Symmetric3Tpl operator- (const SkewSquare & v) const
    {
      const Scalar & x = v.v[0], & y = v.v[1], & z = v.v[2];
      return Symmetric3Tpl(m_data[0]+y*y+z*z,
                           m_data[1]-x*y,m_data[2]+x*x+z*z,
                           m_data[3]-x*z,m_data[4]-y*z,m_data[5]+x*x+y*y);
    }
    
    Symmetric3Tpl& operator-= (const SkewSquare & v)
    {
      const Scalar & x = v.v[0], & y = v.v[1], & z = v.v[2];
      m_data[0]+=y*y+z*z;
      m_data[1]-=x*y; m_data[2]+=x*x+z*z;
      m_data[3]-=x*z; m_data[4]-=y*z; m_data[5]+=x*x+y*y;
      return *this;
    }
    
    template<typename D>
    friend Matrix3 operator- (const Symmetric3Tpl & S, const Eigen::MatrixBase <D> & M)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);
      Matrix3 result (S.matrix());
      result -= M;
      
      return result;
    }

    struct AlphaSkewSquare
    {
      const Scalar & m;
      const Vector3 & v;
      
      AlphaSkewSquare(const Scalar & m, const SkewSquare & v) : m(m),v(v.v) {}
      AlphaSkewSquare(const Scalar & m, const Vector3 & v) : m(m),v(v) {}
      
      operator Symmetric3Tpl () const 
      {
        const Scalar & x = v[0], & y = v[1], & z = v[2];
        return Symmetric3Tpl(-m*(y*y+z*z),
                             m* x*y,-m*(x*x+z*z),
                             m* x*z,m* y*z,-m*(x*x+y*y));
      }
    };
    
    friend AlphaSkewSquare operator* (const Scalar & m, const SkewSquare & sk)
    { return AlphaSkewSquare(m,sk); }
    
    Symmetric3Tpl operator- (const AlphaSkewSquare & v) const
    {
      const Scalar & x = v.v[0], & y = v.v[1], & z = v.v[2];
      return Symmetric3Tpl(m_data[0]+v.m*(y*y+z*z),
                           m_data[1]-v.m* x*y, m_data[2]+v.m*(x*x+z*z),
                           m_data[3]-v.m* x*z, m_data[4]-v.m* y*z,
                           m_data[5]+v.m*(x*x+y*y));
    }
    
    Symmetric3Tpl& operator-= (const AlphaSkewSquare & v)
    {
      const Scalar & x = v.v[0], & y = v.v[1], & z = v.v[2];
      m_data[0]+=v.m*(y*y+z*z);
      m_data[1]-=v.m* x*y; m_data[2]+=v.m*(x*x+z*z);
      m_data[3]-=v.m* x*z; m_data[4]-=v.m* y*z; m_data[5]+=v.m*(x*x+y*y);
      return *this;
    }

    const Vector6 & data () const {return m_data;}
    Vector6 & data () {return m_data;}
    
    // static Symmetric3Tpl SkewSq( const Vector3 & v )
    // { 
    //   const Scalar & x = v[0], & y = v[1], & z = v[2];
    //   return Symmetric3Tpl(-y*y-z*z,
    // 			    x*y, -x*x-z*z,
    // 			    x*z, y*z, -x*x-y*y );
    // }

    /* Shoot a positive definite matrix. */
    static Symmetric3Tpl RandomPositive() 
    { 
      Scalar
      a = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      b = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      c = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      d = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      e = Scalar(std::rand())/RAND_MAX*2.0-1.0,
      f = Scalar(std::rand())/RAND_MAX*2.0-1.0;
      return Symmetric3Tpl(a*a+b*b+d*d,
			   a*b+b*c+d*e, b*b+c*c+e*e,
			   a*d+b*e+d*f, b*d+c*e+e*f,  d*d+e*e+f*f );
    }
    
    Matrix3 matrix() const
    {
      Matrix3 res;
      res(0,0) = m_data(0); res(0,1) = m_data(1); res(0,2) = m_data(3);
      res(1,0) = m_data(1); res(1,1) = m_data(2); res(1,2) = m_data(4);
      res(2,0) = m_data(3); res(2,1) = m_data(4); res(2,2) = m_data(5);
      return res;
    }
    operator Matrix3 () const { return matrix(); }
    
    Scalar vtiv (const Vector3 & v) const
    {
      const Scalar & x = v[0];
      const Scalar & y = v[1];
      const Scalar & z = v[2];
      
      const Scalar xx = x*x;
      const Scalar xy = x*y;
      const Scalar xz = x*z;
      const Scalar yy = y*y;
      const Scalar yz = y*z;
      const Scalar zz = z*z;
      
      return m_data(0)*xx + m_data(2)*yy + m_data(5)*zz + 2.*(m_data(1)*xy + m_data(3)*xz + m_data(4)*yz);
    }
    
    ///
    /// \brief Performs the operation \f$ M = [v]_{\times} S_{3} \f$.
    ///        This operation is equivalent to applying the cross product of v on each column of S.
    ///
    /// \tparam Vector3, Matrix3
    ///
    /// \param[in]  v  a vector of dimension 3.
    /// \param[in]  S3 a symmetric matrix of dimension 3x3.
    /// \param[out] M  an output matrix of dimension 3x3.
    ///
    template<typename Vector3, typename Matrix3>
    static void vxs(const Eigen::MatrixBase<Vector3> & v,
                    const Symmetric3Tpl & S3,
                    const Eigen::MatrixBase<Matrix3> & M)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
      
      const Scalar & a = S3.data()[0];
      const Scalar & b = S3.data()[1];
      const Scalar & c = S3.data()[2];
      const Scalar & d = S3.data()[3];
      const Scalar & e = S3.data()[4];
      const Scalar & f = S3.data()[5];
      
      
      const typename Vector3::RealScalar & v0 = v[0];
      const typename Vector3::RealScalar & v1 = v[1];
      const typename Vector3::RealScalar & v2 = v[2];
      
      Matrix3 & M_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,M);
      M_(0,0) = d * v1 - b * v2;
      M_(1,0) = a * v2 - d * v0;
      M_(2,0) = b * v0 - a * v1;
      
      M_(0,1) = e * v1 - c * v2;
      M_(1,1) = b * v2 - e * v0;
      M_(2,1) = c * v0 - b * v1;
      
      M_(0,2) = f * v1 - e * v2;
      M_(1,2) = d * v2 - f * v0;
      M_(2,2) = e * v0 - d * v1;
    }
    
    ///
    /// \brief Performs the operation \f$ [v]_{\times} S \f$.
    ///        This operation is equivalent to applying the cross product of v on each column of S.
    ///
    /// \tparam Vector3
    ///
    /// \param[in]  v  a vector of dimension 3.
    ///
    /// \returns the result \f$ [v]_{\times} S \f$.
    ///
    template<typename Vector3>
    Matrix3 vxs(const Eigen::MatrixBase<Vector3> & v) const
    {
      Matrix3 M;
      vxs(v,*this,M);
      return M;
    }
    
    ///
    /// \brief Performs the operation \f$ M = S_{3} [v]_{\times} \f$.
    ///
    /// \tparam Vector3, Matrix3
    ///
    /// \param[in]  v  a vector of dimension 3.
    /// \param[in]  S3 a symmetric matrix of dimension 3x3.
    /// \param[out] M  an output matrix of dimension 3x3.
    ///
    template<typename Vector3, typename Matrix3>
    static void svx(const Eigen::MatrixBase<Vector3> & v,
                    const Symmetric3Tpl & S3,
                    const Eigen::MatrixBase<Matrix3> & M)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
      
      const Scalar & a = S3.data()[0];
      const Scalar & b = S3.data()[1];
      const Scalar & c = S3.data()[2];
      const Scalar & d = S3.data()[3];
      const Scalar & e = S3.data()[4];
      const Scalar & f = S3.data()[5];
      
      const typename Vector3::RealScalar & v0 = v[0];
      const typename Vector3::RealScalar & v1 = v[1];
      const typename Vector3::RealScalar & v2 = v[2];
      
      Matrix3 & M_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,M);
      M_(0,0) = b * v2 - d * v1;
      M_(1,0) = c * v2 - e * v1;
      M_(2,0) = e * v2 - f * v1;
      
      M_(0,1) = d * v0 - a * v2;
      M_(1,1) = e * v0 - b * v2;
      M_(2,1) = f * v0 - d * v2;
      
      M_(0,2) = a * v1 - b * v0;
      M_(1,2) = b * v1 - c * v0;
      M_(2,2) = d * v1 - e * v0;
    }
    
    /// \brief Performs the operation \f$ M = S_{3} [v]_{\times} \f$.
    ///
    /// \tparam Vector3
    ///
    /// \param[in]  v  a vector of dimension 3.
    ///
    /// \returns the result \f$ S [v]_{\times} \f$.
    ///
    template<typename Vector3>
    Matrix3 svx(const Eigen::MatrixBase<Vector3> & v) const
    {
      Matrix3 M;
      svx(v,*this,M);
      return M;
    }

    Symmetric3Tpl operator+(const Symmetric3Tpl & s2) const
    {
      return Symmetric3Tpl((m_data+s2.m_data).eval());
    }

    Symmetric3Tpl & operator+=(const Symmetric3Tpl & s2)
    {
      m_data += s2.m_data; return *this;
    }

    template<typename V3in, typename V3out>
    static void rhsMult(const Symmetric3Tpl & S3,
                        const Eigen::MatrixBase<V3in> & vin,
                        const Eigen::MatrixBase<V3out> & vout)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(V3in,Vector3);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(V3out,Vector3);
      
      V3out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3out,vout);
      
      vout_[0] = S3.m_data(0) * vin[0] + S3.m_data(1) * vin[1] + S3.m_data(3) * vin[2];
      vout_[1] = S3.m_data(1) * vin[0] + S3.m_data(2) * vin[1] + S3.m_data(4) * vin[2];
      vout_[2] = S3.m_data(3) * vin[0] + S3.m_data(4) * vin[1] + S3.m_data(5) * vin[2];
    }

    template<typename V3>
    Vector3 operator*(const Eigen::MatrixBase<V3> & v) const
    {
      Vector3 res;
      rhsMult(*this,v,res);
      return res;
    }
    
    // Matrix3 operator*(const Matrix3 &a) const
    // {
    //   Matrix3 r;
    //   for(unsigned int i=0; i<3; ++i)
    //     {
    //       r(0,i) = m_data(0) * a(0,i) + m_data(1) * a(1,i) + m_data(3) * a(2,i);
    //       r(1,i) = m_data(1) * a(0,i) + m_data(2) * a(1,i) + m_data(4) * a(2,i);
    //       r(2,i) = m_data(3) * a(0,i) + m_data(4) * a(1,i) + m_data(5) * a(2,i);
    //     }
    //   return r;
    // }

    const Scalar& operator()(const int &i,const int &j) const
    {
      return ((i!=2)&&(j!=2)) ? m_data[i+j] : m_data[i+j+1];
    }

    Symmetric3Tpl operator-(const Matrix3 &S) const
    {
      assert( (S-S.transpose()).isMuchSmallerThan(S) );
      return Symmetric3Tpl( m_data(0)-S(0,0),
			    m_data(1)-S(1,0), m_data(2)-S(1,1),
			    m_data(3)-S(2,0), m_data(4)-S(2,1), m_data(5)-S(2,2) );
    }

    Symmetric3Tpl operator+(const Matrix3 &S) const
    {
      assert( (S-S.transpose()).isMuchSmallerThan(S) );
      return Symmetric3Tpl( m_data(0)+S(0,0),
			    m_data(1)+S(1,0), m_data(2)+S(1,1),
			    m_data(3)+S(2,0), m_data(4)+S(2,1), m_data(5)+S(2,2) );
    }

    /* --- Symmetric R*S*R' and R'*S*R products --- */
  public: //private:
    
    /** \brief Computes L for a symmetric matrix A.
     */
    Matrix32  decomposeltI() const
    {
      Matrix32 L;
      L << 
      m_data(0) - m_data(5),    m_data(1),
      m_data(1),              m_data(2) - m_data(5),
      2*m_data(3),            m_data(4) + m_data(4);
      return L;
    }

    /* R*S*R' */
    template<typename D>
    Symmetric3Tpl rotate(const Eigen::MatrixBase<D> & R) const
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);
      assert(isUnitary(R.transpose()*R) && "R is not a Unitary matrix");

      Symmetric3Tpl Sres;
      
      // 4 a
      const Matrix32 L( decomposeltI() );
      
      // Y = R' L   ===> (12 m + 8 a)
      const Matrix2 Y( R.template block<2,3>(1,0) * L );
	
      // Sres= Y R  ===> (16 m + 8a)
      Sres.m_data(1) = Y(0,0)*R(0,0) + Y(0,1)*R(0,1);
      Sres.m_data(2) = Y(0,0)*R(1,0) + Y(0,1)*R(1,1);
      Sres.m_data(3) = Y(1,0)*R(0,0) + Y(1,1)*R(0,1);
      Sres.m_data(4) = Y(1,0)*R(1,0) + Y(1,1)*R(1,1);
      Sres.m_data(5) = Y(1,0)*R(2,0) + Y(1,1)*R(2,1);

      // r=R' v ( 6m + 3a)
      const Vector3 r(-R(0,0)*m_data(4) + R(0,1)*m_data(3),
                      -R(1,0)*m_data(4) + R(1,1)*m_data(3),
                      -R(2,0)*m_data(4) + R(2,1)*m_data(3));

      // Sres_11 (3a)
      Sres.m_data(0) = L(0,0) + L(1,1) - Sres.m_data(2) - Sres.m_data(5);
	
      // Sres + D + (Ev)x ( 9a)
      Sres.m_data(0) += m_data(5);
      Sres.m_data(1) += r(2); Sres.m_data(2)+= m_data(5);
      Sres.m_data(3) +=-r(1); Sres.m_data(4)+= r(0); Sres.m_data(5) += m_data(5);

      return Sres;
    }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    Symmetric3Tpl<NewScalar,Options> cast() const
    {
      return Symmetric3Tpl<NewScalar,Options>(m_data.template cast<NewScalar>());
    }
    
    // TODO: adjust code
//    bool isValid() const
//    {
//      return
//         m_data(0) >= Scalar(0)
//      && m_data(2) >= Scalar(0)
//      && m_data(5) >= Scalar(0);
//    }

  protected:
    Vector6 m_data;
    
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_symmetric3_hpp__

