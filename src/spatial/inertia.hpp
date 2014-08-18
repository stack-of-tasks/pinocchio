#ifndef __se3_inertia_hpp__
#define __se3_inertia_hpp__

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
    typedef Eigen::SelfAdjointView<Matrix3,Eigen::Upper> Symmetric3;
    
  public:
    // Constructors
    InertiaTpl() : m(), c(), dense_I(), I(dense_I) {}
    InertiaTpl(Scalar m_, 
	       const Vector3 &c_, 
	       const Matrix3 &I_)
      : m(m_),
	c(c_),
	dense_I(I_),
	I(dense_I)  {}
    InertiaTpl(Scalar _m, 
	       const Vector3 &_c, 
	       const Symmetric3 &_I)
      : m(_m),
	c(_c),
	dense_I(),
	I(dense_I)   { I = _I; }
    InertiaTpl(const InertiaTpl & clone)  // Clone constructor for std::vector 
      : m(clone.m),
	c(clone.c),
	dense_I(clone.dense_I),
	I(dense_I)    {}
    InertiaTpl& operator= (const InertiaTpl& clone) // Copy operator for std::vector 
    {
      m=clone.m; c=clone.c; dense_I=clone.dense_I;    
      return *this;
    }

    // Initializers
    static Inertia Zero() 
    {
      return InertiaTpl(0., 
			Vector3::Zero(), 
			Matrix3::Zero());
    }
    static Inertia Identity() 
    {
      return InertiaTpl(1., 
			Vector3::Zero(), 
			Matrix3::Identity());
    }
    static Inertia Random()
    {
      return InertiaTpl(Eigen::internal::random<Scalar>(),
			Vector3::Random(),
			Matrix3::Random().template selfadjointView<Eigen::Upper>());
    }

    // Getters
    Scalar             mass()    const { return m; }
    const Vector3 &    lever()   const { return c; }
    const Symmetric3 & inertia() const { return I; }

    Matrix6 toMatrix() const
    {
      Matrix6 M;
      M.template block<3,3>(LINEAR, LINEAR ) =  m*Matrix3::Identity();
      M.template block<3,3>(LINEAR, ANGULAR) = -m*skew(c);
      M.template block<3,3>(ANGULAR,LINEAR ) =  m*skew(c);
      M.template block<3,3>(ANGULAR,ANGULAR) =  (Matrix3)I - m*skew(c)*skew(c);
      return M;
    }
    operator Matrix6 () const { return toMatrix(); }

    // Arithmetic operators
    Inertia operator+(const InertiaTpl &other) const
    {
      return InertiaTpl(m+other.m, c+other.c, I+other.I);
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
		      M.rotation()*I*M.rotation().transpose());
    }
    /// bI = aXb.actInv(aI)
    Inertia se3ActionInverse(const SE3 & M) const
    {
      return Inertia( m,
		      M.rotation().transpose()*(c-M.translation()),
		      M.rotation().transpose()*I*M.rotation());
    }

    //
    Force vxiv( const Motion& v ) const 
    {
      Vector3 mcxw = m*c.cross(v.angular());
      Vector3 mv_mcxw = m*v.linear()-mcxw;
      return Force( v.angular().cross(mv_mcxw),
		    v.angular().cross(c.cross(mv_mcxw)+I*v.angular())-v.linear().cross(mcxw) );
    }

    friend std::ostream & operator<< (std::ostream &os, const Inertia &I)
    {
      os << "m =\n" << I.m << "\n"
	 << "c =\n" << I.c << "\n"
	 << "I =\n" << (Matrix3)I.I;
      return os;
    }

  public:
  private:
    Scalar m;
    Vector3 c;
    Matrix3 dense_I;
    Symmetric3 I;
  };


  typedef InertiaTpl<double,0> Inertia;
    
} // namespace se3

#endif // ifndef __se3_inertia_hpp__
