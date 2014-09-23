/* Copyright LAAS-CNRS, 2014
 *
 * This file is originally copied from metapod/tools/spatial/lti.hh.
 * Authors: Olivier Stasse (LAAS, CNRS) and Sébastien Barthélémy (Aldebaran Robotics)
 * The file was modified in pinocchio by Nicolas Mansard (LAAS, CNRS)
 *
 * metapod is free software, distributed under the terms of the GNU Lesser
 * General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __se3__symmetric3_hpp__
#define __se3__symmetric3_hpp__

#include <ostream>

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

  public:    
    Symmetric3Tpl(): data() {}
    template<typename Sc,int N,int Opt>
    explicit Symmetric3Tpl(const Eigen::Matrix<Sc,N,N,Opt> & I)
    { 
      assert( (I.rows()==3)&&(I.cols()==3) );
      assert( (I-I.transpose()).isMuchSmallerThan(I) );
      data(0) = I(0,0);
      data(1) = I(1,0); data(2) = I(1,1);
      data(3) = I(2,0); data(4) = I(2,1); data(5) = I(2,2);
    }
    explicit Symmetric3Tpl(const Eigen::MatrixBase<Matrix3> &I) 
    {
      assert( (I-I.transpose()).isMuchSmallerThan(I) );
      data(0) = I(0,0);
      data(1) = I(1,0); data(2) = I(1,1);
      data(3) = I(2,0); data(4) = I(2,1); data(5) = I(2,2);
    }
    explicit Symmetric3Tpl(const Vector6 &I) : data(I) {}
    Symmetric3Tpl(const double & a0,const double & a1,const double & a2,
		  const double & a3,const double & a4,const double & a5)
    { data << a0,a1,a2,a3,a4,a5; }

    static Symmetric3Tpl Zero()     { return Symmetric3Tpl(Vector6::Zero()  );  }
    static Symmetric3Tpl Random()   { return Symmetric3Tpl(Vector6::Random().eval());  }
    static Symmetric3Tpl Identity() { return Symmetric3Tpl( 1, 0, 1, 0, 0, 1);  }

    /* Requiered by Inertia::operator== */
    bool operator== (const Symmetric3Tpl & S2 ) { return data == S2.data; }
    
    struct SkewSquare
    {
      const Vector3 & v;
      SkewSquare( const Vector3 & v ) : v(v) {}
      operator Symmetric3Tpl () const 
      {
	const double & x = v[0], & y = v[1], & z = v[2]; 
	return Symmetric3Tpl( -y*y-z*z,
			      x*y    ,  -x*x-z*z, 
			      x*z    ,   y*z    ,  -x*x-y*y );
      }
    }; // struct SkewSquare 
    Symmetric3Tpl operator- (const SkewSquare & v) const
    {
      const double & x = v.v[0], & y = v.v[1], & z = v.v[2]; 
      return Symmetric3Tpl( data[0]+y*y+z*z,
			    data[1]-x*y    ,  data[2]+x*x+z*z, 
			    data[3]-x*z    ,  data[4]-y*z    ,  data[5]+x*x+y*y );
    }
    Symmetric3Tpl& operator-= (const SkewSquare & v)
    {
      const double & x = v.v[0], & y = v.v[1], & z = v.v[2]; 
      data[0]+=y*y+z*z;
      data[1]-=x*y    ;  data[2]+=x*x+z*z; 
      data[3]-=x*z    ;  data[4]-=y*z    ;  data[5]+=x*x+y*y;
      return *this;
    }

    struct AlphaSkewSquare
    {
      const double & m;  const Vector3 & v;
      AlphaSkewSquare( const double & m, const SkewSquare & v ) : m(m),v(v.v) {}
      operator Symmetric3Tpl () const 
      {
	const double & x = v[0], & y = v[1], & z = v[2]; 
	return Symmetric3Tpl( -m*(y*y+z*z),
			       m* x*y     ,  -m*(x*x+z*z), 
			       m* x*z     ,   m* y*z     ,  -m*(x*x+y*y) );
      }
    };
    friend AlphaSkewSquare operator* (const double & m, const SkewSquare & sk )
    { return AlphaSkewSquare(m,sk); }
    Symmetric3Tpl operator- (const AlphaSkewSquare & v) const
    {
      const double & x = v.v[0], & y = v.v[1], & z = v.v[2]; 
      return Symmetric3Tpl( data[0]+v.m*(y*y+z*z),
			    data[1]-v.m* x*y     ,  data[2]+v.m*(x*x+z*z), 
			    data[3]-v.m* x*z     ,  data[4]-v.m* y*z     ,  data[5]+v.m*(x*x+y*y) );
    }
    Symmetric3Tpl& operator-= (const AlphaSkewSquare & v)
    {
      const double & x = v.v[0], & y = v.v[1], & z = v.v[2]; 
      data[0]+=v.m*(y*y+z*z);
      data[1]-=v.m* x*y     ;  data[2]+=v.m*(x*x+z*z); 
      data[3]-=v.m* x*z     ;  data[4]-=v.m* y*z     ;  data[5]+=v.m*(x*x+y*y);
      return *this;
    }


    // static Symmetric3Tpl SkewSq( const Vector3 & v )
    // { 
    //   const double & x = v[0], & y = v[1], & z = v[2]; 
    //   return Symmetric3Tpl( -y*y-z*z,
    // 			    x*y    ,  -x*x-z*z, 
    // 			    x*z    ,   y*z    ,  -x*x-y*y );
    // }

    /* Shoot a positive definite matrix. */
    static Symmetric3Tpl RandomPositive() 
    { 
      double 
	a = double(std::rand())/RAND_MAX*2.0-1.0,
	b = double(std::rand())/RAND_MAX*2.0-1.0,
	c = double(std::rand())/RAND_MAX*2.0-1.0,
	d = double(std::rand())/RAND_MAX*2.0-1.0,
	e = double(std::rand())/RAND_MAX*2.0-1.0,
	f = double(std::rand())/RAND_MAX*2.0-1.0;
      return Symmetric3Tpl(a*a+b*b+d*d,
			   a*b+b*c+d*e, b*b+c*c+e*e,
			   a*d+b*e+d*f, b*d+c*e+e*f,  d*d+e*e+f*f );
    }

    Matrix3 matrix() const
    {
      Matrix3 res;
      res(0,0) = data(0); res(0,1) = data(1); res(0,2) = data(3);
      res(1,0) = data(1); res(1,1) = data(2); res(1,2) = data(4);
      res(2,0) = data(3); res(2,1) = data(4); res(2,2) = data(5);
      return res;
    }
    operator Matrix3 () const { return matrix(); }

    Symmetric3Tpl operator+(const Symmetric3Tpl & s2) const
    {
      return Symmetric3Tpl((data+s2.data).eval());
    }

    Symmetric3Tpl & operator+=(const Symmetric3Tpl & s2)
    {
      data += s2.data; return *this;
    }

    Vector3 operator*(const Vector3 &v) const
    {
      return Vector3(
		     data(0) * v(0) + data(1) * v(1) + data(3) * v(2),
		     data(1) * v(0) + data(2) * v(1) + data(4) * v(2),
		     data(3) * v(0) + data(4) * v(1) + data(5) * v(2)
		     );		     
    }

    // Matrix3 operator*(const Matrix3 &a) const
    // {
    //   Matrix3 r;
    //   for(unsigned int i=0; i<3; ++i)
    //     {
    //       r(0,i) = data(0) * a(0,i) + data(1) * a(1,i) + data(3) * a(2,i);
    //       r(1,i) = data(1) * a(0,i) + data(2) * a(1,i) + data(4) * a(2,i);
    //       r(2,i) = data(3) * a(0,i) + data(4) * a(1,i) + data(5) * a(2,i);
    //     }
    //   return r;
    // }

    const Scalar& operator()(const int &i,const int &j) const
    {
      return ((i!=2)&&(j!=2)) ? data[i+j] : data[i+j+1];
    }

    Symmetric3Tpl operator-(const Matrix3 &S) const
    {
      assert( (S-S.transpose()).isMuchSmallerThan(S) );
      return Symmetric3Tpl( data(0)-S(0,0),
			    data(1)-S(1,0), data(2)-S(1,1),
			    data(3)-S(2,0), data(4)-S(2,1), data(5)-S(2,2) );
    }

    Symmetric3Tpl operator+(const Matrix3 &S) const
    {
      assert( (S-S.transpose()).isMuchSmallerThan(S) );
      return Symmetric3Tpl( data(0)+S(0,0),
			    data(1)+S(1,0), data(2)+S(1,1),
			    data(3)+S(2,0), data(4)+S(2,1), data(5)+S(2,2) );
    }

    /* --- Symmetric R*S*R' and R'*S*R products --- */
  public: //private:
    
    /** \brief Computes L for a symmetric matrix A.
     */
    Matrix32  decomposeltI() const
    {
      Matrix32 L;
      L << 
	data(0) - data(5),    data(1),
	data(1),              data(2) - data(5),
	2*data(3),            data(4) + data(4);
      return L;
    }

    /* R*S*R' */
    template<typename D>
    Symmetric3Tpl rotate(const Eigen::MatrixBase<D> & R) const
    {
      assert( (R.cols()==3) && (R.rows()==3) );
      assert( (R.transpose()*R).isApprox(Matrix3::Identity()) );

      Symmetric3Tpl Sres;
      
      // 4 a
      const Matrix32 & L = decomposeltI();
      
      // Y = R' L   ===> (12 m + 8 a)
      const Matrix2 & Y = R.template block<2,3>(1,0) * L;
	
      // Sres= Y R  ===> (16 m + 8a)
      Sres.data(1) = Y(0,0)*R(0,0) + Y(0,1)*R(0,1);
      Sres.data(2) = Y(0,0)*R(1,0) + Y(0,1)*R(1,1);
      Sres.data(3) = Y(1,0)*R(0,0) + Y(1,1)*R(0,1);
      Sres.data(4) = Y(1,0)*R(1,0) + Y(1,1)*R(1,1);
      Sres.data(5) = Y(1,0)*R(2,0) + Y(1,1)*R(2,1);

      // r=R' v ( 6m + 3a)
      const Vector3 r( -R(0,0)*data(4) + R(0,1)*data(3),
		       -R(1,0)*data(4) + R(1,1)*data(3),
		       -R(2,0)*data(4) + R(2,1)*data(3) );

      // Sres_11 (3a)
      Sres.data(0) = L(0,0) + L(1,1) - Sres.data(2) - Sres.data(5);
	
      // Sres + D + (Ev)x ( 9a)
      Sres.data(0) += data(5); 
      Sres.data(1) += r(2);    Sres.data(2)+= data(5); 
      Sres.data(3) +=-r(1);    Sres.data(4)+=    r(0); Sres.data(5) += data(5);

      return Sres;
    }


  public: //todo: make private

    Vector6 data; 
  };

  typedef Symmetric3Tpl<double,0> Symmetric3;

} // namespace se3

#endif // ifndef __se3__symmetric3_hpp__

