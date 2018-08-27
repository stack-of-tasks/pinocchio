//
// Copyright (c) 2014-2017 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//
// This file is originally copied from metapod/tools/spatial/lti.hh.
// Authors: Olivier Stasse (LAAS, CNRS) and Sébastien Barthélémy (Aldebaran Robotics)
// The file was modified in pinocchio by Nicolas Mansard (LAAS, CNRS)
//
// metapod is free software, distributed under the terms of the GNU Lesser
// General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef __se3_symmetric3_hpp__
#define __se3_symmetric3_hpp__

#include "pinocchio/macros.hpp"

namespace se3
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
    Symmetric3Tpl(): data_() {}
    
//    template<typename D>
//    explicit Symmetric3Tpl(const Eigen::MatrixBase<D> & I)
//    {
//      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);
//      assert( (I-I.transpose()).isMuchSmallerThan(I) );
//      data_(0) = I(0,0);
//      data_(1) = I(1,0); data_(2) = I(1,1);
//      data_(3) = I(2,0); data_(4) = I(2,1); data_(5) = I(2,2);
//    }
    template<typename Sc,int N,int Opt>
    explicit Symmetric3Tpl(const Eigen::Matrix<Sc,N,N,Opt> & I)
    {
      EIGEN_STATIC_ASSERT(N==3,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
      assert( (I-I.transpose()).isMuchSmallerThan(I) );
      data_(0) = I(0,0);
      data_(1) = I(1,0); data_(2) = I(1,1);
      data_(3) = I(2,0); data_(4) = I(2,1); data_(5) = I(2,2);
    }
    
    explicit Symmetric3Tpl(const Vector6 & I) : data_(I) {}
    
    Symmetric3Tpl(const Scalar & a0, const Scalar & a1, const Scalar & a2,
		  const Scalar & a3, const Scalar & a4, const Scalar & a5)
    { data_ << a0,a1,a2,a3,a4,a5; }

    static Symmetric3Tpl Zero()     { return Symmetric3Tpl(Vector6::Zero());  }
    void setZero() { data_.setZero(); }
    
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
      
      data_ << a, b, c, d, e, f;
    }
    
    static Symmetric3Tpl Identity() { return Symmetric3Tpl(1, 0, 1, 0, 0, 1);  }
    void setIdentity() { data_ << 1, 0, 1, 0, 0, 1; }

    /* Requiered by Inertia::operator== */
    bool operator== (const Symmetric3Tpl & S2) const { return data_ == S2.data_; }
    
    bool isApprox(const Symmetric3Tpl & other,
                  const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return data_.isApprox(other.data_,prec); }
    
    void fill(const Scalar value) { data_.fill(value); }
    
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
      return Symmetric3Tpl(data_[0]+y*y+z*z,
                           data_[1]-x*y,data_[2]+x*x+z*z,
                           data_[3]-x*z,data_[4]-y*z,data_[5]+x*x+y*y);
    }
    
    Symmetric3Tpl& operator-= (const SkewSquare & v)
    {
      const Scalar & x = v.v[0], & y = v.v[1], & z = v.v[2];
      data_[0]+=y*y+z*z;
      data_[1]-=x*y; data_[2]+=x*x+z*z;
      data_[3]-=x*z; data_[4]-=y*z; data_[5]+=x*x+y*y;
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
      return Symmetric3Tpl(data_[0]+v.m*(y*y+z*z),
                           data_[1]-v.m* x*y, data_[2]+v.m*(x*x+z*z),
                           data_[3]-v.m* x*z, data_[4]-v.m* y*z,
                           data_[5]+v.m*(x*x+y*y));
    }
    
    Symmetric3Tpl& operator-= (const AlphaSkewSquare & v)
    {
      const Scalar & x = v.v[0], & y = v.v[1], & z = v.v[2];
      data_[0]+=v.m*(y*y+z*z);
      data_[1]-=v.m* x*y; data_[2]+=v.m*(x*x+z*z);
      data_[3]-=v.m* x*z; data_[4]-=v.m* y*z; data_[5]+=v.m*(x*x+y*y);
      return *this;
    }

    const Vector6 & data () const {return data_;}
    Vector6 & data () {return data_;}
    
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
      res(0,0) = data_(0); res(0,1) = data_(1); res(0,2) = data_(3);
      res(1,0) = data_(1); res(1,1) = data_(2); res(1,2) = data_(4);
      res(2,0) = data_(3); res(2,1) = data_(4); res(2,2) = data_(5);
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
      
      return data_(0)*xx + data_(2)*yy + data_(5)*zz + 2.*(data_(1)*xy + data_(3)*xz + data_(4)*yz);
    }
    
    ///
    /// \brief Performs the operation \f$ M = [v]_{\cross} S_{3} \f$.
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
      
      Matrix3 & M_ = const_cast<Eigen::MatrixBase<Matrix3> &>(M).derived();
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
    /// \brief Performs the operation \f$ [v]_{\cross} S \f$.
    ///        This operation is equivalent to applying the cross product of v on each column of S.
    ///
    /// \tparam Vector3
    ///
    /// \param[in]  v  a vector of dimension 3.
    ///
    /// \returns the result \f$ [v]_{\cross} S \f$.
    ///
    template<typename Vector3>
    Matrix3 vxs(const Eigen::MatrixBase<Vector3> & v) const
    {
      Matrix3 M;
      vxs(v,*this,M);
      return M;
    }
    
    ///
    /// \brief Performs the operation \f$ M = S_{3} [v]_{\cross \f$.
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
      
      Matrix3 & M_ = const_cast<Eigen::MatrixBase<Matrix3> &>(M).derived();
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
    
    /// \brief Performs the operation \f$ M = S_{3} [v]_{\cross \f$.
    ///
    /// \tparam Vector3
    ///
    /// \param[in]  v  a vector of dimension 3.
    ///
    /// \returns the result \f$ S [v]_{\cross} \f$.
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
      return Symmetric3Tpl((data_+s2.data_).eval());
    }

    Symmetric3Tpl & operator+=(const Symmetric3Tpl & s2)
    {
      data_ += s2.data_; return *this;
    }

    template<typename V3in, typename V3out>
    static void rhsMult(const Symmetric3Tpl & S3,
                        const Eigen::MatrixBase<V3in> & vin,
                        const Eigen::MatrixBase<V3out> & vout)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(V3in,Vector3);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(V3out,Vector3);
      
      Eigen::MatrixBase<V3out> & vout_ = const_cast<Eigen::MatrixBase<V3out>&>(vout);
      
      vout_[0] = S3.data_(0) * vin[0] + S3.data_(1) * vin[1] + S3.data_(3) * vin[2];
      vout_[1] = S3.data_(1) * vin[0] + S3.data_(2) * vin[1] + S3.data_(4) * vin[2];
      vout_[2] = S3.data_(3) * vin[0] + S3.data_(4) * vin[1] + S3.data_(5) * vin[2];
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
    //       r(0,i) = data_(0) * a(0,i) + data_(1) * a(1,i) + data_(3) * a(2,i);
    //       r(1,i) = data_(1) * a(0,i) + data_(2) * a(1,i) + data_(4) * a(2,i);
    //       r(2,i) = data_(3) * a(0,i) + data_(4) * a(1,i) + data_(5) * a(2,i);
    //     }
    //   return r;
    // }

    const Scalar& operator()(const int &i,const int &j) const
    {
      return ((i!=2)&&(j!=2)) ? data_[i+j] : data_[i+j+1];
    }

    Symmetric3Tpl operator-(const Matrix3 &S) const
    {
      assert( (S-S.transpose()).isMuchSmallerThan(S) );
      return Symmetric3Tpl( data_(0)-S(0,0),
			    data_(1)-S(1,0), data_(2)-S(1,1),
			    data_(3)-S(2,0), data_(4)-S(2,1), data_(5)-S(2,2) );
    }

    Symmetric3Tpl operator+(const Matrix3 &S) const
    {
      assert( (S-S.transpose()).isMuchSmallerThan(S) );
      return Symmetric3Tpl( data_(0)+S(0,0),
			    data_(1)+S(1,0), data_(2)+S(1,1),
			    data_(3)+S(2,0), data_(4)+S(2,1), data_(5)+S(2,2) );
    }

    /* --- Symmetric R*S*R' and R'*S*R products --- */
  public: //private:
    
    /** \brief Computes L for a symmetric matrix A.
     */
    Matrix32  decomposeltI() const
    {
      Matrix32 L;
      L << 
	data_(0) - data_(5),    data_(1),
	data_(1),              data_(2) - data_(5),
	2*data_(3),            data_(4) + data_(4);
      return L;
    }

    /* R*S*R' */
    template<typename D>
    Symmetric3Tpl rotate(const Eigen::MatrixBase<D> & R) const
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);
      assert( (R.transpose()*R).isApprox(Matrix3::Identity()) );

      Symmetric3Tpl Sres;
      
      // 4 a
      const Matrix32 L( decomposeltI() );
      
      // Y = R' L   ===> (12 m + 8 a)
      const Matrix2 Y( R.template block<2,3>(1,0) * L );
	
      // Sres= Y R  ===> (16 m + 8a)
      Sres.data_(1) = Y(0,0)*R(0,0) + Y(0,1)*R(0,1);
      Sres.data_(2) = Y(0,0)*R(1,0) + Y(0,1)*R(1,1);
      Sres.data_(3) = Y(1,0)*R(0,0) + Y(1,1)*R(0,1);
      Sres.data_(4) = Y(1,0)*R(1,0) + Y(1,1)*R(1,1);
      Sres.data_(5) = Y(1,0)*R(2,0) + Y(1,1)*R(2,1);

      // r=R' v ( 6m + 3a)
      const Vector3 r(-R(0,0)*data_(4) + R(0,1)*data_(3),
                      -R(1,0)*data_(4) + R(1,1)*data_(3),
                      -R(2,0)*data_(4) + R(2,1)*data_(3));

      // Sres_11 (3a)
      Sres.data_(0) = L(0,0) + L(1,1) - Sres.data_(2) - Sres.data_(5);
	
      // Sres + D + (Ev)x ( 9a)
      Sres.data_(0) += data_(5); 
      Sres.data_(1) += r(2); Sres.data_(2)+= data_(5);
      Sres.data_(3) +=-r(1); Sres.data_(4)+= r(0); Sres.data_(5) += data_(5);

      return Sres;
    }

  protected:
    Vector6 data_;
    
  };

} // namespace se3

#endif // ifndef __se3_symmetric3_hpp__

