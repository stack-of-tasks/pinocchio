#ifndef __se3_inertia_hpp__
#define __se3_inertia_hpp__

#include "pinocchio/spatial/symmetric3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"


namespace se3
{
  template<typename _Scalar, int _Options>
  class InertiaTpl
  {
  public:
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef InertiaTpl<Scalar,Options> Inertia;
    enum { LINEAR = 0, ANGULAR = 3 };
    typedef Symmetric3Tpl<Scalar,Options> Symmetric3;
    
  public:
    // Constructors
    InertiaTpl() : m(), c(), I() {}
    InertiaTpl(Scalar m_, 
	       const Vector3 &c_, 
	       const Matrix3 &I_)
      : m(m_),
	c(c_),
	I(I_)  {}
    InertiaTpl(Scalar _m, 
	       const Vector3 &_c, 
	       const Symmetric3 &_I)
      : m(_m),
	c(_c),
	I(_I)   { }
    InertiaTpl(const InertiaTpl & clone)  // Clone constructor for std::vector 
      : m(clone.m),
	c(clone.c),
	I(clone.I)    {}
    InertiaTpl& operator= (const InertiaTpl& clone) // Copy operator for std::vector 
    {
      m=clone.m; c=clone.c; I=clone.I;
      return *this;
    }
    /* Requiered by std::vector boost::python bindings. */
    bool operator==( const InertiaTpl& Y2 ) 
    { return (m==Y2.m) && (c==Y2.c) && (I==Y2.I); }
    template<typename S2,int O2>
    InertiaTpl( const InertiaTpl<S2,O2> & clone )
      : m(clone.mass()),
	c(clone.lever()),
	I(clone.inertia().matrix())    {}

    // Initializers
    static Inertia Zero() 
    {
      return InertiaTpl(0., 
			Vector3::Zero(), 
			Symmetric3::Zero());
    }
    static Inertia Identity() 
    {
      return InertiaTpl(1., 
			Vector3::Zero(), 
			Symmetric3::Identity());
    }
    static Inertia Random()
    {
      /* We have to shoot "I" definite positive and not only symmetric. */
      return InertiaTpl(Eigen::internal::random<Scalar>()+1,
			Vector3::Random(),
			Symmetric3::RandomPositive());
    }

    // Getters
    Scalar             mass()    const { return m; }
    const Vector3 &    lever()   const { return c; }
    const Symmetric3 & inertia() const { return I; }

    Matrix6 matrix() const
    {
      Matrix6 M;
      M.template block<3,3>(LINEAR, LINEAR ) =  m*Matrix3::Identity();
      M.template block<3,3>(LINEAR, ANGULAR) = -m*skew(c);
      M.template block<3,3>(ANGULAR,LINEAR ) =  m*skew(c);
      M.template block<3,3>(ANGULAR,ANGULAR) =  (Matrix3)(I - m*typename Symmetric3::SkewSquare(c));
      return M;
    }
    operator Matrix6 () const { return matrix(); }

    // Arithmetic operators
    friend Inertia operator+(const InertiaTpl &Ya, const InertiaTpl &Yb)
    {
      /* Y_{a+b} = ( m_a+m_b,
       *             (m_a*c_a + m_b*c_b ) / (m_a + m_b),
       *             I_a + I_b - (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
       */

      const double & mab = Ya.m+Yb.m;
      const Vector3 & AB = (Ya.c-Yb.c).eval();
      return InertiaTpl( mab,
			 (Ya.m*Ya.c+Yb.m*Yb.c)/mab,
			 Ya.I+Yb.I - (Ya.m*Yb.m/mab)* typename Symmetric3::SkewSquare(AB));
    }

    Inertia& operator+=(const InertiaTpl &Yb)
    {
      const Inertia& Ya = *this;
      const double & mab = Ya.m+Yb.m;
      const Vector3 & AB = (Ya.c-Yb.c).eval();
      c *= m; c += Yb.m*Yb.c; c /= mab;
      I += Yb.I; I -= (Ya.m*Yb.m/mab)* typename Symmetric3::SkewSquare(AB);
      m  = mab;
      return *this;
    }

    Force operator*(const Motion &v) const 
    {
      Vector3 mcxw = m*c.cross(v.angular());
      return Force( m*v.linear()-mcxw,
		    m*c.cross(v.linear()) + I*v.angular() - c.cross(mcxw) );
    }

    /// aI = aXb.act(bI)
    Inertia se3Action(const SE3 & M) const
    {
      /* The multiplication RIR' has a particular form that could be used, however it
       * does not seems to be more efficient, see http://stackoverflow.com/questions/
       * 13215467/eigen-best-way-to-evaluate-asa-transpose-and-store-the-result-in-a-symmetric .*/
      return Inertia( m,
		      M.translation()+M.rotation()*c,
		      I.rotate(M.rotation()) );
    }
    /// bI = aXb.actInv(aI)
    Inertia se3ActionInverse(const SE3 & M) const
    {
      return Inertia( m,
		      M.rotation().transpose()*(c-M.translation()),
		      I.rotate(M.rotation().transpose()) );
    }

    //
    Force vxiv( const Motion& v ) const 
    {
      const Vector3 & mcxw = m*c.cross(v.angular());
      const Vector3 & mv_mcxw = m*v.linear()-mcxw;
      return Force( v.angular().cross(mv_mcxw),
		    v.angular().cross(c.cross(mv_mcxw)+I*v.angular())-v.linear().cross(mcxw) );
    }

    friend std::ostream & operator<< (std::ostream &os, const Inertia &I)
    {
      os << "m =" << I.m << ";\n"
	 << "c = [\n" << I.c.transpose() << "]';\n"
	 << "I = [\n" << (Matrix3)I.I << "];";
      return os;
    }

  public:
  private:
    Scalar m;
    Vector3 c;
    Symmetric3 I;
  };

  typedef InertiaTpl<double,0> Inertia;
    
} // namespace se3

#endif // ifndef __se3_inertia_hpp__
