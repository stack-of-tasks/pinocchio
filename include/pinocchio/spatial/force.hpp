#ifndef __se3_force_hpp__
#define __se3_force_hpp__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pinocchio/spatial/fwd.hpp"

namespace se3
{
  template<typename _Scalar, int _Options>
  class ForceTpl
  {
  public:
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef SE3Tpl<Scalar,Options> SE3;
    enum { LINEAR = 0, ANGULAR = 3 };

  public:
    // Constructors
    ForceTpl() : m_n(), m_f() {}
    template<typename f3_t,typename n3_t>
    ForceTpl(const Eigen::MatrixBase<f3_t> & f,const Eigen::MatrixBase<n3_t> & n)
      : m_n(n), m_f(f) {}
    template<typename f6>
    explicit ForceTpl(const Eigen::MatrixBase<f6> & f)
      : m_n(f.template segment<3>(ANGULAR))
      , m_f(f.template segment<3>(LINEAR)) 
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(f6);
      assert( f.size() == 6 );
    }
    template<typename S2,int O2>
    explicit ForceTpl(const ForceTpl<S2,O2> & clone)
      : m_n(clone.angular()), m_f(clone.linear()) {}

    // initializers
    static ForceTpl Zero() { return ForceTpl(Vector3::Zero(), Vector3::Zero()); }
    static ForceTpl Random() { return ForceTpl(Vector3::Random(), Vector3::Random()); }

    Vector6 toVector() const
    {
      Vector6 f;
      f.template segment<3>(ANGULAR) = m_n;
      f.template segment<3>(LINEAR)  = m_f;
      return f;
    }
    operator Vector6 () const { return toVector(); }

    // Getters
    const Vector3 & linear() const { return m_f; }
    const Vector3 & angular() const { return m_n; }
    void linear(const Vector3 & f) { m_f = f; }
    void angular(const Vector3 & n) { m_n = n; }

    // Arithmetic operators
    template<typename S2, int O2>
    ForceTpl & operator= (const ForceTpl<S2,O2> & other)
    {
      m_n = other.angular ();
      m_f = other.linear ();
      return *this;
    }
    
    template<typename F6>
    ForceTpl & operator=(const Eigen::MatrixBase<F6> & phi)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(F6); assert(phi.size() == 6);
      m_n = phi.template segment<3>(ANGULAR);
      m_f = phi.template segment<3>(LINEAR);
      return *this;
    }

    ForceTpl operator+(const ForceTpl & phi) const
    {
      return ForceTpl(m_f+phi.m_f,m_n+phi.m_n);
    }

    ForceTpl& operator+= (const ForceTpl & phi)
    {
      m_f += phi.m_f;
      m_n += phi.m_n;
      return *this;
    }

    ForceTpl operator-() const
    {
      return ForceTpl(-m_f, -m_n);
    }

    ForceTpl operator-(const ForceTpl & phi) const
    {
      return ForceTpl(m_f-phi.m_f,m_n-phi.m_n);
    }

    // ForceTpl operator*(Scalar a) const
    // {
    //   return ForceTpl(m_n*a, m_f*a);
    // }

    // friend ForceTpl operator*(Scalar a, const ForceTpl & phi)
    // {
    //   return ForceTpl(phi.n()*a, phi.f()*a);
    // }


    /// af = aXb.act(bf)
    ForceTpl se3Action(const SE3 & m) const
    {
      Vector3 Rf = static_cast<Vector3>(m.rotation()*linear());
      return ForceTpl(Rf,m.translation().cross(Rf)+m.rotation()*angular());
    }
    /// bf = aXb.actInv(af)
    ForceTpl se3ActionInverse(const SE3 & m) const
    {
      return ForceTpl(m.rotation().transpose()*linear(),
		      m.rotation().transpose()*(angular() - m.translation().cross(linear())) );
    }

    friend std::ostream & operator << (std::ostream & os, const ForceTpl & phi)
    {
      os
	<< "f =\n" << phi.linear() << std::endl
	<< "tau =\n" << phi.angular() << std::endl;
      return os;
    }

  public: //
  private:
    Vector3 m_n;
    Vector3 m_f;
  };
 
  typedef ForceTpl<double> Force;

} // namespace se3

#endif // ifndef __se3_force_hpp__

