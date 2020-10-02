//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_inertia_hpp__
#define __pinocchio_inertia_hpp__

#include <iostream>

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/spatial/symmetric3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{

  template< class Derived>
  class InertiaBase
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);

  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t & derived() const { return *static_cast<const Derived_t*>(this); }

    Scalar           mass()    const { return static_cast<const Derived_t*>(this)->mass(); }
    Scalar &         mass() { return static_cast<const Derived_t*>(this)->mass(); }
    const Vector3 &    lever()   const { return static_cast<const Derived_t*>(this)->lever(); }
    Vector3 &          lever() { return static_cast<const Derived_t*>(this)->lever(); }
    const Symmetric3 & inertia() const { return static_cast<const Derived_t*>(this)->inertia(); }
    Symmetric3 &       inertia() { return static_cast<const Derived_t*>(this)->inertia(); }

    Matrix6 matrix() const { return derived().matrix_impl(); }
    operator Matrix6 () const { return matrix(); }

    Derived_t& operator= (const Derived_t& clone){return derived().__equl__(clone);}
    bool operator==(const Derived_t & other) const {return derived().isEqual(other);}
    bool operator!=(const Derived_t & other) const { return !(*this == other); }
    
    Derived_t& operator+= (const Derived_t & Yb) { return derived().__pequ__(Yb); }
    Derived_t operator+(const Derived_t & Yb) const { return derived().__plus__(Yb); }
    
    template<typename MotionDerived>
    ForceTpl<typename traits<MotionDerived>::Scalar,traits<MotionDerived>::Options>
    operator*(const MotionDense<MotionDerived> & v) const
    { return derived().__mult__(v); }

    Scalar vtiv(const Motion & v) const { return derived().vtiv_impl(v); }
    Matrix6 variation(const Motion & v) const { return derived().variation_impl(v); }
    
    /// \brief Time variation operator.
    ///        It computes the time derivative of an inertia I corresponding to the formula \f$ \dot{I} = v \times^{*} I \f$.
    ///
    /// \param[in] v The spatial velocity of the frame supporting the inertia.
    /// \param[in] I The spatial inertia in motion.
    /// \param[out] Iout The time derivative of the inertia I.
    ///
    template<typename M6>
    static void vxi(const Motion & v, const Derived & I, const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M6, Matrix6);
      Derived::vxi_impl(v,I,Iout);
    }
    
    Matrix6 vxi(const Motion & v) const
    {
      Matrix6 Iout;
      vxi(v,derived(),Iout);
      return Iout;
    }
    
    /// \brief Time variation operator.
    ///        It computes the time derivative of an inertia I corresponding to the formula \f$ \dot{I} = v \times^{*} I \f$.
    ///
    /// \param[in] v The spatial velocity of the frame supporting the inertia.
    /// \param[in] I The spatial inertia in motion.
    /// \param[out] Iout The time derivative of the inertia I.
    ///
    template<typename M6>
    static void ivx(const Motion & v, const Derived & I, const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M6, Matrix6);
      Derived::ivx_impl(v,I,Iout);
    }
    
    Matrix6 ivx(const Motion & v) const
    {
      Matrix6 Iout;
      ivx(v,derived(),Iout);
      return Iout;
    }

    void setZero() { derived().setZero(); }
    void setIdentity() { derived().setIdentity(); }
    void setRandom() { derived().setRandom(); }
    
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec); }
    
    bool isZero(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isZero_impl(prec); }

    /// aI = aXb.act(bI)
    Derived_t se3Action(const SE3 & M) const { return derived().se3Action_impl(M); }

    /// bI = aXb.actInv(aI)
    Derived_t se3ActionInverse(const SE3 & M) const { return derived().se3ActionInverse_impl(M); }

    void disp(std::ostream & os) const { static_cast<const Derived_t*>(this)->disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os,const InertiaBase<Derived_t> & X)
    { 
      X.disp(os);
      return os;
    }

  }; // class InertiaBase


  template<typename T, int U>
  struct traits< InertiaTpl<T, U> >
  {
    typedef T Scalar;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef Matrix6 ActionMatrix_t;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Eigen::Quaternion<T,U> Quaternion_t;
    typedef SE3Tpl<T,U> SE3;
    typedef ForceTpl<T,U> Force;
    typedef MotionTpl<T,U> Motion;
    typedef Symmetric3Tpl<T,U> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits InertiaTpl

  template<typename _Scalar, int _Options>
  class InertiaTpl : public InertiaBase< InertiaTpl< _Scalar, _Options > >
  {
  public:
    friend class InertiaBase< InertiaTpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(InertiaTpl);
    enum { Options = _Options };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename Symmetric3::AlphaSkewSquare AlphaSkewSquare;

    typedef typename Eigen::Matrix<_Scalar, 10, 1, _Options> Vector10;

  public:
    // Constructors
    InertiaTpl()
    {}

    InertiaTpl(const Scalar mass, const Vector3 & com, const Matrix3 & rotational_inertia)
    : m_mass(mass), m_com(com), m_inertia(rotational_inertia)
    {}
    
    InertiaTpl(const Matrix6 & I6)
    {
      assert((I6 - I6.transpose()).isMuchSmallerThan(I6));
      mass() = I6(LINEAR, LINEAR);
      const Matrix3 & mc_cross = I6.template block <3,3>(ANGULAR,LINEAR);
      lever() = unSkew(mc_cross);
      lever() /= mass();
      
      Matrix3 I3 (mc_cross * mc_cross);
      I3 /= mass();
      I3 += I6.template block<3,3>(ANGULAR,ANGULAR);
      const Symmetric3 S3(I3);
      inertia() = S3;
    }

    InertiaTpl(Scalar mass, const Vector3 & com, const Symmetric3 & rotational_inertia)
    : m_mass(mass), m_com(com), m_inertia(rotational_inertia)
    {}
    
    InertiaTpl(const InertiaTpl & clone)  // Copy constructor
    : m_mass(clone.mass()), m_com(clone.lever()), m_inertia(clone.inertia())
    {}

    InertiaTpl& operator=(const InertiaTpl & clone)  // Copy assignment operator
    {
      m_mass = clone.mass();
      m_com = clone.lever();
      m_inertia = clone.inertia();
      return *this;
    }

    template<int O2>
    InertiaTpl(const InertiaTpl<Scalar,O2> & clone)
    : m_mass(clone.mass())
    , m_com(clone.lever())
    , m_inertia(clone.inertia().matrix())
    {}

    // Initializers
    static InertiaTpl Zero() 
    {
      return InertiaTpl(Scalar(0),
                        Vector3::Zero(), 
                        Symmetric3::Zero());
    }
    
    void setZero() { mass() = Scalar(0); lever().setZero(); inertia().setZero(); }

    static InertiaTpl Identity() 
    {
      return InertiaTpl(Scalar(1),
                        Vector3::Zero(), 
                        Symmetric3::Identity());
    }
    
    void setIdentity ()
    {
      mass() = Scalar(1); lever().setZero(); inertia().setIdentity();
    }

    static InertiaTpl Random()
    {
        // We have to shoot "I" definite positive and not only symmetric.
      return InertiaTpl(Eigen::internal::random<Scalar>()+1,
                        Vector3::Random(),
                        Symmetric3::RandomPositive());
    }
    
    static InertiaTpl FromSphere(const Scalar m, const Scalar radius)
    {
      return FromEllipsoid(m,radius,radius,radius);
    }

    static InertiaTpl FromEllipsoid(const Scalar m, const Scalar x,
                                    const Scalar y, const Scalar z)
    {
      Scalar a = m * (y*y + z*z) / Scalar(5);
      Scalar b = m * (x*x + z*z) / Scalar(5);
      Scalar c = m * (y*y + x*x) / Scalar(5);
      return InertiaTpl(m, Vector3::Zero(), Symmetric3(a,         Scalar(0), b,
                                                       Scalar(0), Scalar(0), c));
    }

    static InertiaTpl FromCylinder(const Scalar m, const Scalar r, const Scalar l)
    {
      Scalar a = m * (r*r / Scalar(4) + l*l / Scalar(12));
      Scalar c = m * (r*r / Scalar(2));
      return InertiaTpl(m, Vector3::Zero(), Symmetric3(a,         Scalar(0), a,
                                                       Scalar(0), Scalar(0), c));
    }

    static InertiaTpl FromBox(const Scalar m, const Scalar x,
                              const Scalar y, const Scalar z)
    {
      Scalar a = m * (y*y + z*z) / Scalar(12);
      Scalar b = m * (x*x + z*z) / Scalar(12);
      Scalar c = m * (y*y + x*x) / Scalar(12);
      return InertiaTpl(m, Vector3::Zero(), Symmetric3(a,         Scalar(0), b,
                                                       Scalar(0), Scalar(0), c));
    }

    void setRandom()
    {
      mass() = static_cast<Scalar>(std::rand())/static_cast<Scalar>(RAND_MAX);
      lever().setRandom(); inertia().setRandom();
    }

    Matrix6 matrix_impl() const
    {
      Matrix6 M;
      
      M.template block<3,3>(LINEAR, LINEAR ).setZero();
      M.template block<3,3>(LINEAR, LINEAR ).diagonal().fill (mass());
      M.template block<3,3>(ANGULAR,LINEAR ) = alphaSkew(mass(),lever());
      M.template block<3,3>(LINEAR, ANGULAR) = -M.template block<3,3>(ANGULAR, LINEAR);
      M.template block<3,3>(ANGULAR,ANGULAR) = (inertia() - AlphaSkewSquare(mass(),lever())).matrix();

      return M;
    }

    /** Returns the representation of the matrix as a vector of dynamic parameters.
    * The parameters are given as
    * \f$ v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T \f$
    * where \f$ c \f$ is the center of mass, \f$ I = I_C + mS^T(c)S(c) \f$ and \f$ I_C \f$ has its origin at the barycenter
    * and \f$ S(c) \f$ is the the skew matrix representation of the cross product operator.
    */
    Vector10 toDynamicParameters() const
    {
      Vector10 v;

      v[0] = mass();
      v.template segment<3>(1) = mass() * lever();
      v.template segment<6>(4) = (inertia() - AlphaSkewSquare(mass(),lever())).data();

      return v;
    }

    /** Builds and inertia matrix from a vector of dynamic parameters.
    *
    * @param[in] params The dynamic parameters.
    *
    * The parameters are given as
    * \f$ v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T \f$
    * where \f$ I = I_C + mS^T(c)S(c) \f$ and \f$ I_C \f$ has its origin at the barycenter.
    */
    template<typename Vector10Like>
    static InertiaTpl FromDynamicParameters(const Eigen::MatrixBase<Vector10Like> & params)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector10Like, params, 10, 1);

      const Scalar & mass = params[0];
      Vector3 lever = params.template segment<3>(1);
      lever /= mass;

      return InertiaTpl(mass, lever, Symmetric3(params.template segment<6>(4)) + AlphaSkewSquare(mass,lever));
    }

    // Arithmetic operators
    InertiaTpl & __equl__(const InertiaTpl & clone)
    {
      mass() = clone.mass(); lever() = clone.lever(); inertia() = clone.inertia();
      return *this;
    }

    // Required by std::vector boost::python bindings.
    bool isEqual( const InertiaTpl& Y2 ) const
    { 
      return (mass()==Y2.mass()) && (lever()==Y2.lever()) && (inertia()==Y2.inertia());
    }
    
    bool isApprox_impl(const InertiaTpl & other,
                       const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      using math::fabs;
      return fabs(static_cast<Scalar>(mass() - other.mass())) <= prec
      && lever().isApprox(other.lever(),prec)
      && inertia().isApprox(other.inertia(),prec);
    }
    
    bool isZero_impl(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      using math::fabs;
      return fabs(mass()) <= prec
          && lever().isZero(prec)
          && inertia().isZero(prec);
    }
    
    InertiaTpl __plus__(const InertiaTpl & Yb) const
    {
      /* Y_{a+b} = ( m_a+m_b,
       *             (m_a*c_a + m_b*c_b ) / (m_a + m_b),
       *             I_a + I_b - (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
       */

      const Scalar & mab = mass()+Yb.mass();
      const Scalar mab_inv = Scalar(1)/mab;
      const Vector3 & AB = (lever()-Yb.lever()).eval();
      return InertiaTpl(mab,
                        (mass()*lever()+Yb.mass()*Yb.lever())*mab_inv,
                        inertia()+Yb.inertia() - (mass()*Yb.mass()*mab_inv)* typename Symmetric3::SkewSquare(AB));
    }

    InertiaTpl& __pequ__(const InertiaTpl & Yb)
    {
      const InertiaTpl& Ya = *this;
      const Scalar & mab = Ya.mass()+Yb.mass();
      const Scalar mab_inv = Scalar(1)/mab;
      const Vector3 & AB = (Ya.lever()-Yb.lever()).eval();
      lever() *= (mass()*mab_inv); lever() += (Yb.mass()*mab_inv)*Yb.lever(); //c *= mab_inv;
      inertia() += Yb.inertia(); inertia() -= (Ya.mass()*Yb.mass()*mab_inv)* typename Symmetric3::SkewSquare(AB);
      mass() = mab;
      return *this;
    }

    template<typename MotionDerived>
    ForceTpl<typename traits<MotionDerived>::Scalar,traits<MotionDerived>::Options>
    __mult__(const MotionDense<MotionDerived> & v) const
    {
      typedef ForceTpl<typename traits<MotionDerived>::Scalar,traits<MotionDerived>::Options> ReturnType;
      ReturnType f;
      __mult__(v,f);
      return f;
    }
    
    template<typename MotionDerived, typename ForceDerived>
    void __mult__(const MotionDense<MotionDerived> & v, ForceDense<ForceDerived> & f) const
    {
      f.linear().noalias() = mass()*(v.linear() - lever().cross(v.angular()));
      Symmetric3::rhsMult(inertia(),v.angular(),f.angular());
      f.angular() += lever().cross(f.linear());
//      f.angular().noalias() = c.cross(f.linear()) + I*v.angular();
    }
    
    Scalar vtiv_impl(const Motion & v) const
    {
      const Vector3 cxw (lever().cross(v.angular()));
      Scalar res = mass() * (v.linear().squaredNorm() - Scalar(2)*v.linear().dot(cxw));
      const Vector3 mcxcxw (-mass()*lever().cross(cxw));
      res += v.angular().dot(mcxcxw);
      res += inertia().vtiv(v.angular());
      
      return res;
    }
    
    Matrix6 variation(const Motion & v) const
    {
      Matrix6 res;
      const Motion mv(v*mass());
      
      res.template block<3,3>(LINEAR,ANGULAR) = -skew(mv.linear()) - skewSquare(mv.angular(),lever()) + skewSquare(lever(),mv.angular());
      res.template block<3,3>(ANGULAR,LINEAR) = res.template block<3,3>(LINEAR,ANGULAR).transpose();
      
//      res.template block<3,3>(LINEAR,LINEAR) = mv.linear()*c.transpose(); // use as temporary variable
//      res.template block<3,3>(ANGULAR,ANGULAR) = res.template block<3,3>(LINEAR,LINEAR) - res.template block<3,3>(LINEAR,LINEAR).transpose();
      res.template block<3,3>(ANGULAR,ANGULAR) = -skewSquare(mv.linear(),lever()) - skewSquare(lever(),mv.linear());
      
      res.template block<3,3>(LINEAR,LINEAR) = (inertia() - AlphaSkewSquare(mass(),lever())).matrix();
      
      res.template block<3,3>(ANGULAR,ANGULAR) -= res.template block<3,3>(LINEAR,LINEAR) * skew(v.angular());
      res.template block<3,3>(ANGULAR,ANGULAR) += cross(v.angular(),res.template block<3,3>(LINEAR,LINEAR));
      
      res.template block<3,3>(LINEAR,LINEAR).setZero();
      return res;
    }
    
    template<typename M6>
    static void vxi_impl(const Motion & v,
                         const InertiaTpl & I,
                         const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6,6,6);
      M6 & Iout_ = PINOCCHIO_EIGEN_CONST_CAST(M6,Iout);

      // Block 1,1
      alphaSkew(I.mass(),v.angular(),Iout_.template block<3,3>(LINEAR,LINEAR));
//      Iout_.template block<3,3>(LINEAR,LINEAR) = alphaSkew(I.mass(),v.angular());
      const Vector3 mc(I.mass()*I.lever());

      // Block 1,2
      skewSquare(-v.angular(),mc,Iout_.template block<3,3>(LINEAR,ANGULAR));
      

      //// Block 2,1
      alphaSkew(I.mass(),v.linear(),Iout_.template block<3,3>(ANGULAR,LINEAR));
      Iout_.template block<3,3>(ANGULAR,LINEAR) -= Iout_.template block<3,3>(LINEAR,ANGULAR);

      //// Block 2,2
      skewSquare(-v.linear(),mc,Iout_.template block<3,3>(ANGULAR,ANGULAR));

      // TODO: I do not why, but depending on the CPU, these three lines can give
      // wrong output.
      //      typename Symmetric3::AlphaSkewSquare mcxcx(I.mass(),I.lever());
      //      const Symmetric3 I_mcxcx(I.inertia() - mcxcx);
      //      Iout_.template block<3,3>(ANGULAR,ANGULAR) += I_mcxcx.vxs(v.angular());
      Symmetric3 mcxcx(typename Symmetric3::AlphaSkewSquare(I.mass(),I.lever()));
      Iout_.template block<3,3>(ANGULAR,ANGULAR) += I.inertia().vxs(v.angular());
      Iout_.template block<3,3>(ANGULAR,ANGULAR) -= mcxcx.vxs(v.angular());
      
    }
    
    template<typename M6>
    static void ivx_impl(const Motion & v,
                         const InertiaTpl & I,
                         const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6,6,6);
      M6 & Iout_ = PINOCCHIO_EIGEN_CONST_CAST(M6,Iout);
      
      // Block 1,1
      alphaSkew(I.mass(),v.angular(),Iout_.template block<3,3>(LINEAR,LINEAR));
      
      // Block 2,1
      const Vector3 mc(I.mass()*I.lever());
      skewSquare(mc,v.angular(),Iout_.template block<3,3>(ANGULAR,LINEAR));
      
      // Block 1,2
      alphaSkew(I.mass(),v.linear(),Iout_.template block<3,3>(LINEAR,ANGULAR));
      
      // Block 2,2
      cross(-I.lever(),Iout_.template block<3,3>(ANGULAR,LINEAR),Iout_.template block<3,3>(ANGULAR,ANGULAR));
      Iout_.template block<3,3>(ANGULAR,ANGULAR) += I.inertia().svx(v.angular());
      for(int k = 0; k < 3; ++k)
        Iout_.template block<3,3>(ANGULAR,ANGULAR).col(k) += I.lever().cross(Iout_.template block<3,3>(LINEAR,ANGULAR).col(k));

      // Block 1,2
      Iout_.template block<3,3>(LINEAR,ANGULAR) -= Iout_.template block<3,3>(ANGULAR,LINEAR);
      
    }

    // Getters
    Scalar           mass()    const { return m_mass; }
    const Vector3 &    lever()   const { return m_com; }
    const Symmetric3 & inertia() const { return m_inertia; }
    
    Scalar &   mass()    { return m_mass; }
    Vector3 &    lever()   { return m_com; }
    Symmetric3 & inertia() { return m_inertia; }

    /// aI = aXb.act(bI)
    InertiaTpl se3Action_impl(const SE3 & M) const
    {
      /* The multiplication RIR' has a particular form that could be used, however it
       * does not seems to be more efficient, see http://stackoverflow.m_comom/questions/
       * 13215467/eigen-best-way-to-evaluate-asa-transpose-and-store-the-result-in-a-symmetric .*/
       return InertiaTpl(mass(),
                         M.translation()+M.rotation()*lever(),
                         inertia().rotate(M.rotation()));
     }

    ///bI = aXb.actInv(aI)
    InertiaTpl se3ActionInverse_impl(const SE3 & M) const
    {
      return InertiaTpl(mass(),
                        M.rotation().transpose()*(lever()-M.translation()),
                        inertia().rotate(M.rotation().transpose()) );
    }

    Force vxiv( const Motion& v ) const 
    {
      const Vector3 & mcxw = mass()*lever().cross(v.angular());
      const Vector3 & mv_mcxw = mass()*v.linear()-mcxw;
      return Force( v.angular().cross(mv_mcxw),
                    v.angular().cross(lever().cross(mv_mcxw)+inertia()*v.angular())-v.linear().cross(mcxw) );
    }

    void disp_impl(std::ostream & os) const
    {
      os
      << "  m = " << mass() << "\n"
      << "  c = " << lever().transpose() << "\n"
      << "  I = \n" << inertia().matrix() << "";
    }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    InertiaTpl<NewScalar,Options> cast() const
    {
      return InertiaTpl<NewScalar,Options>(static_cast<NewScalar>(mass()),
                                           lever().template cast<NewScalar>(),
                                           inertia().template cast<NewScalar>());
    }

  protected:
    Scalar m_mass;
    Vector3 m_com;
    Symmetric3 m_inertia;
    
  }; // class InertiaTpl
    
} // namespace pinocchio

#endif // ifndef __pinocchio_inertia_hpp__
