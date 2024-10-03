//
// Copyright (c) 2015-2021 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_spatial_inertia_hpp__
#define __pinocchio_spatial_inertia_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/spatial/symmetric3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{
  template<class Derived>
  struct InertiaBase : NumericalBase<Derived>
  {
    SPATIAL_TYPEDEF_TEMPLATE(Derived);

    Derived & derived()
    {
      return *static_cast<Derived *>(this);
    }
    const Derived & derived() const
    {
      return *static_cast<const Derived *>(this);
    }

    Derived & const_cast_derived() const
    {
      return *const_cast<Derived *>(&derived());
    }

    Scalar mass() const
    {
      return static_cast<const Derived *>(this)->mass();
    }
    Scalar & mass()
    {
      return static_cast<const Derived *>(this)->mass();
    }
    const Vector3 & lever() const
    {
      return static_cast<const Derived *>(this)->lever();
    }
    Vector3 & lever()
    {
      return static_cast<const Derived *>(this)->lever();
    }
    const Symmetric3 & inertia() const
    {
      return static_cast<const Derived *>(this)->inertia();
    }
    Symmetric3 & inertia()
    {
      return static_cast<const Derived *>(this)->inertia();
    }

    template<typename Matrix6Like>
    void matrix(const Eigen::MatrixBase<Matrix6Like> & mat) const
    {
      derived().matrix_impl(mat);
    }
    Matrix6 matrix() const
    {
      return derived().matrix_impl();
    }
    operator Matrix6() const
    {
      return matrix();
    }

    template<typename Matrix6Like>
    void inverse(const Eigen::MatrixBase<Matrix6Like> & mat) const
    {
      derived().inverse_impl(mat);
    }
    Matrix6 inverse() const
    {
      return derived().inverse_impl();
    }

    Derived & operator=(const Derived & clone)
    {
      return derived().__equl__(clone);
    }
    bool operator==(const Derived & other) const
    {
      return derived().isEqual(other);
    }
    bool operator!=(const Derived & other) const
    {
      return !(*this == other);
    }

    Derived & operator+=(const Derived & Yb)
    {
      return derived().__pequ__(Yb);
    }
    Derived operator+(const Derived & Yb) const
    {
      return derived().__plus__(Yb);
    }
    Derived & operator-=(const Derived & Yb)
    {
      return derived().__mequ__(Yb);
    }
    Derived operator-(const Derived & Yb) const
    {
      return derived().__minus__(Yb);
    }

    template<typename MotionDerived>
    ForceTpl<typename traits<MotionDerived>::Scalar, traits<MotionDerived>::Options>
    operator*(const MotionDense<MotionDerived> & v) const
    {
      return derived().__mult__(v);
    }

    template<typename MotionDerived>
    Scalar vtiv(const MotionDense<MotionDerived> & v) const
    {
      return derived().vtiv_impl(v);
    }

    template<typename MotionDerived>
    Matrix6 variation(const MotionDense<MotionDerived> & v) const
    {
      return derived().variation_impl(v);
    }

    /// \brief Time variation operator.
    ///        It computes the time derivative of an inertia I corresponding to the formula \f$
    ///        \dot{I} = v \times^{*} I \f$.
    ///
    /// \param[in] v The spatial velocity of the frame supporting the inertia.
    /// \param[in] I The spatial inertia in motion.
    /// \param[out] Iout The time derivative of the inertia I.
    ///
    template<typename MotionDerived, typename M6>
    static void
    vxi(const MotionDense<MotionDerived> & v, const Derived & I, const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M6, Matrix6);
      Derived::vxi_impl(v, I, Iout);
    }

    template<typename MotionDerived>
    Matrix6 vxi(const MotionDense<MotionDerived> & v) const
    {
      Matrix6 Iout;
      vxi(v, derived(), Iout);
      return Iout;
    }

    /// \brief Time variation operator.
    ///        It computes the time derivative of an inertia I corresponding to the formula \f$
    ///        \dot{I} = v \times^{*} I \f$.
    ///
    /// \param[in] v The spatial velocity of the frame supporting the inertia.
    /// \param[in] I The spatial inertia in motion.
    /// \param[out] Iout The time derivative of the inertia I.
    ///
    template<typename MotionDerived, typename M6>
    static void
    ivx(const MotionDense<MotionDerived> & v, const Derived & I, const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M6, Matrix6);
      Derived::ivx_impl(v, I, Iout);
    }

    template<typename MotionDerived>
    Matrix6 ivx(const MotionDense<MotionDerived> & v) const
    {
      Matrix6 Iout;
      ivx(v, derived(), Iout);
      return Iout;
    }

    void setZero()
    {
      derived().setZero();
    }
    void setIdentity()
    {
      derived().setIdentity();
    }
    void setRandom()
    {
      derived().setRandom();
    }

    bool isApprox(
      const Derived & other,
      const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return derived().isApprox_impl(other, prec);
    }

    bool isZero(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return derived().isZero_impl(prec);
    }

    /// aI = aXb.act(bI)
    template<typename S2, int O2>
    Derived se3Action(const SE3Tpl<S2, O2> & M) const
    {
      return derived().se3Action_impl(M);
    }

    /// bI = aXb.actInv(aI)
    template<typename S2, int O2>
    Derived se3ActionInverse(const SE3Tpl<S2, O2> & M) const
    {
      return derived().se3ActionInverse_impl(M);
    }

    void disp(std::ostream & os) const
    {
      static_cast<const Derived *>(this)->disp_impl(os);
    }
    friend std::ostream & operator<<(std::ostream & os, const InertiaBase<Derived> & X)
    {
      X.disp(os);
      return os;
    }
  };

  // class InertiaBase
  template<typename T, int U>
  struct traits<InertiaTpl<T, U>>
  {
    typedef T Scalar;
    typedef Eigen::Matrix<T, 3, 1, U> Vector3;
    typedef Eigen::Matrix<T, 4, 1, U> Vector4;
    typedef Eigen::Matrix<T, 6, 1, U> Vector6;
    typedef Eigen::Matrix<T, 3, 3, U> Matrix3;
    typedef Eigen::Matrix<T, 4, 4, U> Matrix4;
    typedef Eigen::Matrix<T, 6, 6, U> Matrix6;
    typedef Eigen::Matrix<T, 10, 10, U> Matrix10;
    typedef Matrix6 ActionMatrix_t;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Eigen::Quaternion<T, U> Quaternion_t;
    typedef SE3Tpl<T, U> SE3;
    typedef ForceTpl<T, U> Force;
    typedef MotionTpl<T, U> Motion;
    typedef Symmetric3Tpl<T, U> Symmetric3;
    typedef PseudoInertiaTpl<T, U> PseudoInertia;
    typedef LogCholeskyParametersTpl<T, U> LogCholeskyParameters;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits InertiaTpl

  template<typename _Scalar, int _Options>
  struct InertiaTpl : public InertiaBase<InertiaTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SPATIAL_TYPEDEF_TEMPLATE(InertiaTpl);
    enum
    {
      Options = _Options
    };

    typedef typename Symmetric3::AlphaSkewSquare AlphaSkewSquare;
    typedef Eigen::Matrix<Scalar, 10, 1, Options> Vector10;
    typedef Eigen::Matrix<Scalar, 10, 10, Options> Matrix10;
    typedef PseudoInertiaTpl<Scalar, Options> PseudoInertia;
    typedef LogCholeskyParametersTpl<Scalar, Options> LogCholeskyParameters;

    // Constructors
    InertiaTpl()
    {
    }

    InertiaTpl(const Scalar & mass, const Vector3 & com, const Matrix3 & rotational_inertia)
    : m_mass(mass)
    , m_com(com)
    , m_inertia(rotational_inertia)
    {
    }

    explicit InertiaTpl(const Matrix6 & I6)
    {
      assert(check_expression_if_real<Scalar>(pinocchio::isZero(I6 - I6.transpose())));
      mass() = I6(LINEAR, LINEAR);
      const typename Matrix6::template ConstFixedBlockXpr<3, 3>::Type mc_cross =
        I6.template block<3, 3>(ANGULAR, LINEAR);
      lever() = unSkew(mc_cross);
      lever() /= mass();

      Matrix3 I3(mc_cross * mc_cross);
      I3 /= mass();
      I3 += I6.template block<3, 3>(ANGULAR, ANGULAR);
      const Symmetric3 S3(I3);
      inertia() = S3;
    }

    InertiaTpl(const Scalar & mass, const Vector3 & com, const Symmetric3 & rotational_inertia)
    : m_mass(mass)
    , m_com(com)
    , m_inertia(rotational_inertia)
    {
    }

    InertiaTpl(const InertiaTpl & clone) // Copy constructor
    : m_mass(clone.mass())
    , m_com(clone.lever())
    , m_inertia(clone.inertia())
    {
    }

    InertiaTpl & operator=(const InertiaTpl & clone) // Copy assignment operator
    {
      m_mass = clone.mass();
      m_com = clone.lever();
      m_inertia = clone.inertia();
      return *this;
    }

    template<typename S2, int O2>
    explicit InertiaTpl(const InertiaTpl<S2, O2> & clone)
    {
      *this = clone.template cast<Scalar>();
    }

    // Initializers
    static InertiaTpl Zero()
    {
      return InertiaTpl(Scalar(0), Vector3::Zero(), Symmetric3::Zero());
    }

    void setZero()
    {
      mass() = Scalar(0);
      lever().setZero();
      inertia().setZero();
    }

    static InertiaTpl Identity()
    {
      return InertiaTpl(Scalar(1), Vector3::Zero(), Symmetric3::Identity());
    }

    void setIdentity()
    {
      mass() = Scalar(1);
      lever().setZero();
      inertia().setIdentity();
    }

    static InertiaTpl Random()
    {
      // We have to shoot "I" definite positive and not only symmetric.
      return InertiaTpl(
        Eigen::internal::random<Scalar>() + 1, Vector3::Random(), Symmetric3::RandomPositive());
    }

    ///
    /// \brief Computes the Inertia of a sphere defined by its mass and its radius.
    ///
    /// \param[in] mass of the sphere.
    /// \param[in] radius of the sphere.
    ///
    static InertiaTpl FromSphere(const Scalar mass, const Scalar radius)
    {
      return FromEllipsoid(mass, radius, radius, radius);
    }

    ///
    /// \brief Computes the Inertia of an ellipsoid defined by its mass and main semi-axis
    /// dimensions (x,y,z).
    ///
    /// \param[in] mass of the ellipsoid.
    /// \param[in] x semi-axis dimension along the local X axis.
    /// \param[in] y semi-axis dimension along the local Y axis.
    /// \param[in] z semi-axis dimension along the local Z axis.
    ///
    static InertiaTpl
    FromEllipsoid(const Scalar mass, const Scalar x, const Scalar y, const Scalar z)
    {
      const Scalar a = mass * (y * y + z * z) / Scalar(5);
      const Scalar b = mass * (x * x + z * z) / Scalar(5);
      const Scalar c = mass * (y * y + x * x) / Scalar(5);
      return InertiaTpl(
        mass, Vector3::Zero(), Symmetric3(a, Scalar(0), b, Scalar(0), Scalar(0), c));
    }

    ///
    /// \brief Computes the Inertia of a cylinder defined by its mass, radius and length along the Z
    /// axis.
    ///
    /// \param[in] mass of the cylinder.
    /// \param[in] radius of the cylinder.
    /// \param[in] length of the cylinder.
    ///
    static InertiaTpl FromCylinder(const Scalar mass, const Scalar radius, const Scalar length)
    {
      const Scalar radius_square = radius * radius;
      const Scalar a = mass * (radius_square / Scalar(4) + length * length / Scalar(12));
      const Scalar c = mass * (radius_square / Scalar(2));
      return InertiaTpl(
        mass, Vector3::Zero(), Symmetric3(a, Scalar(0), a, Scalar(0), Scalar(0), c));
    }

    ///
    /// \brief Computes the Inertia of a box defined by its mass and main dimensions (x,y,z).
    ///
    /// \param[in] mass of the box.
    /// \param[in] x dimension along the local X axis.
    /// \param[in] y dimension along the local Y axis.
    /// \param[in] z dimension along the local Z axis.
    ///
    static InertiaTpl FromBox(const Scalar mass, const Scalar x, const Scalar y, const Scalar z)
    {
      const Scalar a = mass * (y * y + z * z) / Scalar(12);
      const Scalar b = mass * (x * x + z * z) / Scalar(12);
      const Scalar c = mass * (y * y + x * x) / Scalar(12);
      return InertiaTpl(
        mass, Vector3::Zero(), Symmetric3(a, Scalar(0), b, Scalar(0), Scalar(0), c));
    }

    ///
    /// \brief Computes the Inertia of a capsule defined by its mass, radius and length along the Z
    /// axis. Assumes a uniform density.
    ///
    /// \param[in] mass of the capsule.
    /// \param[in] radius of the capsule.
    /// \param[in] height of the capsule.
    static InertiaTpl FromCapsule(const Scalar mass, const Scalar radius, const Scalar height)
    {
      const Scalar pi = boost::math::constants::pi<Scalar>();

      // first need to compute mass repartition between cylinder and halfsphere
      const Scalar v_cyl = pi * math::pow(radius, 2) * height;
      const Scalar v_hs = Scalar(2) / Scalar(3) * math::pow(radius, 3) * pi;
      const Scalar total_v = v_cyl + Scalar(2) * v_hs;

      const Scalar m_cyl = mass * (v_cyl / total_v);
      const Scalar m_hs = mass * (v_hs / total_v);

      // Then Distance between halfSphere center and cylindere center.
      const Scalar dist_hc_cyl = height / Scalar(2) + Scalar(0.375) * radius;

      // Computes inertia terms
      const Scalar ix_c =
        m_cyl * (math::pow(height, 2) / Scalar(12) + math::pow(radius, 2) / Scalar(4));
      const Scalar iz_c = m_cyl * math::pow(radius, 2) / Scalar(2);

      // For halfsphere inertia see, "Dynamics: Theory and Applications" McGraw-Hill, New York,
      // 1985, by T.R. Kane and D.A. Levinson, Appendix 23.
      const Scalar ix_hs = m_hs * math::pow(radius, 2) * Scalar(0.259375);
      const Scalar iz_hs = m_hs * math::pow(radius, 2) * Scalar(0.4);

      // Put everything together using the parallel axis theorem
      const Scalar ix = ix_c + Scalar(2) * (ix_hs + m_hs * math::pow(dist_hc_cyl, 2));
      const Scalar iz = iz_c + Scalar(2) * iz_hs;

      return InertiaTpl(
        mass, Vector3::Zero(), Symmetric3(ix, Scalar(0), ix, Scalar(0), Scalar(0), iz));
    }

    void setRandom()
    {
      mass() = static_cast<Scalar>(std::rand()) / static_cast<Scalar>(RAND_MAX);
      lever().setRandom();
      inertia().setRandom();
    }

    template<typename Matrix6Like>
    void matrix_impl(const Eigen::MatrixBase<Matrix6Like> & M_) const
    {
      Matrix6Like & M = M_.const_cast_derived();

      M.template block<3, 3>(LINEAR, LINEAR).setZero();
      M.template block<3, 3>(LINEAR, LINEAR).diagonal().fill(mass());
      M.template block<3, 3>(ANGULAR, LINEAR) = alphaSkew(mass(), lever());
      M.template block<3, 3>(LINEAR, ANGULAR) = -M.template block<3, 3>(ANGULAR, LINEAR);
      M.template block<3, 3>(ANGULAR, ANGULAR) =
        (inertia() - AlphaSkewSquare(mass(), lever())).matrix();
    }

    Matrix6 matrix_impl() const
    {
      Matrix6 M;
      matrix_impl(M);
      return M;
    }

    template<typename Matrix6Like>
    void inverse_impl(const Eigen::MatrixBase<Matrix6Like> & M_) const
    {
      Matrix6Like & M = M_.const_cast_derived();
      inertia().inverse(M.template block<3, 3>(ANGULAR, ANGULAR));

      M.template block<3, 3>(LINEAR, ANGULAR).noalias() =
        -M.template block<3, 3>(ANGULAR, ANGULAR).colwise().cross(lever());
      M.template block<3, 3>(ANGULAR, LINEAR) = M.template block<3, 3>(LINEAR, ANGULAR).transpose();

      const Scalar &cx = lever()[0], cy = lever()[1], cz = lever()[2];

      M.template block<3, 3>(LINEAR, LINEAR).col(0).noalias() =
        cy * M.template block<3, 3>(LINEAR, ANGULAR).col(2)
        - cz * M.template block<3, 3>(LINEAR, ANGULAR).col(1);

      M.template block<3, 3>(LINEAR, LINEAR).col(1).noalias() =
        cz * M.template block<3, 3>(LINEAR, ANGULAR).col(0)
        - cx * M.template block<3, 3>(LINEAR, ANGULAR).col(2);

      M.template block<3, 3>(LINEAR, LINEAR).col(2).noalias() =
        cx * M.template block<3, 3>(LINEAR, ANGULAR).col(1)
        - cy * M.template block<3, 3>(LINEAR, ANGULAR).col(0);

      const Scalar m_inv = Scalar(1) / mass();
      M.template block<3, 3>(LINEAR, LINEAR).diagonal().array() += m_inv;
    }

    Matrix6 inverse_impl() const
    {
      Matrix6 res;
      inverse_impl(res);
      return res;
    }

    /** Returns the representation of the matrix as a vector of dynamic parameters.
     * The parameters are given as
     * \f$ v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T \f$
     * where \f$ c \f$ is the center of mass, \f$ I = I_C + mS^T(c)S(c) \f$ and \f$ I_C \f$ has its
     * origin at the barycenter and \f$ S(c) \f$ is the the skew matrix representation of the cross
     * product operator.
     */
    Vector10 toDynamicParameters() const
    {
      Vector10 v;

      v[0] = mass();
      v.template segment<3>(1).noalias() = mass() * lever();
      v.template segment<6>(4) = (inertia() - AlphaSkewSquare(mass(), lever())).data();

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

      return InertiaTpl(
        mass, lever, Symmetric3(params.template segment<6>(4)) + AlphaSkewSquare(mass, lever));
    }

    /**

     * @brief Create an InertiaTpl object from a PseudoInertia object.
     *
     * @param pseudo_inertia A PseudoInertia object.
     * @return An InertiaTpl object.
     */

    static InertiaTpl FromPseudoInertia(const PseudoInertia & pseudo_inertia)
    {
      return pseudo_inertia.toInertia();
    }

    /**
     * @brief Convert the InertiaTpl object to a 4x4 pseudo inertia matrix.
     *
     * @return A 4x4 pseudo inertia matrix.
     */
    PseudoInertia toPseudoInertia() const
    {
      PseudoInertia pseudo_inertia = PseudoInertia::FromInertia(*this);
      return pseudo_inertia;
    }

    /**
     * @brief Create an InertiaTpl object from log Cholesky parameters.
     *
     * @param log_cholesky A log cholesky parameters object
     *
     * @return An InertiaTpl object.
     */
    static InertiaTpl FromLogCholeskyParameters(const LogCholeskyParameters & log_cholesky)
    {
      Vector10 dynamic_params = log_cholesky.toDynamicParameters();
      return FromDynamicParameters(dynamic_params);
    }

    // Arithmetic operators
    InertiaTpl & __equl__(const InertiaTpl & clone)
    {
      mass() = clone.mass();
      lever() = clone.lever();
      inertia() = clone.inertia();
      return *this;
    }

    // Required by std::vector boost::python bindings.
    bool isEqual(const InertiaTpl & Y2) const
    {
      return (mass() == Y2.mass()) && (lever() == Y2.lever()) && (inertia() == Y2.inertia());
    }

    bool isApprox_impl(
      const InertiaTpl & other,
      const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      using math::fabs;
      return fabs(static_cast<Scalar>(mass() - other.mass())) <= prec
             && lever().isApprox(other.lever(), prec) && inertia().isApprox(other.inertia(), prec);
    }

    bool isZero_impl(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      using math::fabs;
      return fabs(mass()) <= prec && lever().isZero(prec) && inertia().isZero(prec);
    }

    InertiaTpl __plus__(const InertiaTpl & Yb) const
    {
      /* Y_{a+b} = ( m_a+m_b,
       *             (m_a*c_a + m_b*c_b ) / (m_a + m_b),
       *             I_a + I_b - (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
       */

      const Scalar eps = ::Eigen::NumTraits<Scalar>::epsilon();

      const Scalar & mab = mass() + Yb.mass();
      const Scalar mab_inv = Scalar(1) / math::max(mab, eps);
      const Vector3 & AB = (lever() - Yb.lever()).eval();
      return InertiaTpl(
        mab, (mass() * lever() + Yb.mass() * Yb.lever()) * mab_inv,
        inertia() + Yb.inertia()
          - (mass() * Yb.mass() * mab_inv) * typename Symmetric3::SkewSquare(AB));
    }

    InertiaTpl & __pequ__(const InertiaTpl & Yb)
    {
      static const Scalar eps = ::Eigen::NumTraits<Scalar>::epsilon();

      const InertiaTpl & Ya = *this;
      const Scalar & mab = mass() + Yb.mass();
      const Scalar mab_inv = Scalar(1) / math::max(mab, eps);
      const Vector3 & AB = (Ya.lever() - Yb.lever()).eval();
      lever() *= (mass() * mab_inv);
      lever() += (Yb.mass() * mab_inv) * Yb.lever(); // c *= mab_inv;
      inertia() += Yb.inertia();
      inertia() -= (Ya.mass() * Yb.mass() * mab_inv) * typename Symmetric3::SkewSquare(AB);
      mass() = mab;
      return *this;
    }

    InertiaTpl __minus__(const InertiaTpl & Yb) const
    {
      /* Y_{a} = ( m_{a+b}+m_b,
       *             (m_{a+b}*c_{a+b} - m_b*c_b ) / (m_a),
       *             I_{a+b} - I_b + (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
       */

      const Scalar eps = ::Eigen::NumTraits<Scalar>::epsilon();

      const Scalar ma = mass() - Yb.mass();
      assert(check_expression_if_real<Scalar>(ma >= Scalar(0)));

      const Scalar ma_inv = Scalar(1) / math::max(ma, eps);
      const Vector3 c_a((mass() * lever() - Yb.mass() * Yb.lever()) * ma_inv);

      const Vector3 AB = c_a - Yb.lever();

      return InertiaTpl(
        ma, c_a,
        inertia() - Yb.inertia() + (ma * Yb.mass() / mass()) * typename Symmetric3::SkewSquare(AB));
    }

    InertiaTpl & __mequ__(const InertiaTpl & Yb)
    {
      static const Scalar eps = ::Eigen::NumTraits<Scalar>::epsilon();

      const Scalar ma = mass() - Yb.mass();
      assert(check_expression_if_real<Scalar>(ma >= Scalar(0)));

      const Scalar ma_inv = Scalar(1) / math::max(ma, eps);

      lever() *= (mass() * ma_inv);
      lever().noalias() -= (Yb.mass() * ma_inv) * Yb.lever();

      const Vector3 AB = lever() - Yb.lever();
      inertia() -= Yb.inertia();
      inertia() += (ma * Yb.mass() / mass()) * typename Symmetric3::SkewSquare(AB);
      mass() = ma;

      return *this;
    }

    template<typename MotionDerived>
    ForceTpl<typename traits<MotionDerived>::Scalar, traits<MotionDerived>::Options>
    __mult__(const MotionDense<MotionDerived> & v) const
    {
      typedef ForceTpl<typename traits<MotionDerived>::Scalar, traits<MotionDerived>::Options>
        ReturnType;
      ReturnType f;
      __mult__(v, f);
      return f;
    }

    template<typename MotionDerived, typename ForceDerived>
    void __mult__(const MotionDense<MotionDerived> & v, ForceDense<ForceDerived> & f) const
    {
      f.linear().noalias() = mass() * (v.linear() - lever().cross(v.angular()));
      Symmetric3::rhsMult(inertia(), v.angular(), f.angular());
      f.angular() += lever().cross(f.linear());
      //      f.angular().noalias() = c.cross(f.linear()) + I*v.angular();
    }

    template<typename MotionDerived>
    Scalar vtiv_impl(const MotionDense<MotionDerived> & v) const
    {
      const Vector3 cxw(lever().cross(v.angular()));
      Scalar res = mass() * (v.linear().squaredNorm() - Scalar(2) * v.linear().dot(cxw));
      const Vector3 mcxcxw(-mass() * lever().cross(cxw));
      res += v.angular().dot(mcxcxw);
      res += inertia().vtiv(v.angular());

      return res;
    }

    template<typename MotionDerived>
    Matrix6 variation(const MotionDense<MotionDerived> & v) const
    {
      Matrix6 res;
      const Motion mv(v * mass());

      res.template block<3, 3>(LINEAR, ANGULAR) =
        -skew(mv.linear()) - skewSquare(mv.angular(), lever()) + skewSquare(lever(), mv.angular());
      res.template block<3, 3>(ANGULAR, LINEAR) =
        res.template block<3, 3>(LINEAR, ANGULAR).transpose();

      //      res.template block<3,3>(LINEAR,LINEAR) = mv.linear()*c.transpose(); // use as
      //      temporary variable res.template block<3,3>(ANGULAR,ANGULAR) = res.template
      //      block<3,3>(LINEAR,LINEAR) - res.template block<3,3>(LINEAR,LINEAR).transpose();
      res.template block<3, 3>(ANGULAR, ANGULAR) =
        -skewSquare(mv.linear(), lever()) - skewSquare(lever(), mv.linear());

      res.template block<3, 3>(LINEAR, LINEAR) =
        (inertia() - AlphaSkewSquare(mass(), lever())).matrix();

      res.template block<3, 3>(ANGULAR, ANGULAR) -=
        res.template block<3, 3>(LINEAR, LINEAR) * skew(v.angular());
      res.template block<3, 3>(ANGULAR, ANGULAR) +=
        cross(v.angular(), res.template block<3, 3>(LINEAR, LINEAR));

      res.template block<3, 3>(LINEAR, LINEAR).setZero();
      return res;
    }

    template<typename MotionDerived, typename M6>
    static void vxi_impl(
      const MotionDense<MotionDerived> & v,
      const InertiaTpl & I,
      const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6, 6, 6);
      M6 & Iout_ = PINOCCHIO_EIGEN_CONST_CAST(M6, Iout);

      // Block 1,1
      alphaSkew(I.mass(), v.angular(), Iout_.template block<3, 3>(LINEAR, LINEAR));
      //      Iout_.template block<3,3>(LINEAR,LINEAR) = alphaSkew(I.mass(),v.angular());
      const Vector3 mc(I.mass() * I.lever());

      // Block 1,2
      skewSquare(-v.angular(), mc, Iout_.template block<3, 3>(LINEAR, ANGULAR));

      //// Block 2,1
      alphaSkew(I.mass(), v.linear(), Iout_.template block<3, 3>(ANGULAR, LINEAR));
      Iout_.template block<3, 3>(ANGULAR, LINEAR) -= Iout_.template block<3, 3>(LINEAR, ANGULAR);

      //// Block 2,2
      skewSquare(-v.linear(), mc, Iout_.template block<3, 3>(ANGULAR, ANGULAR));

      // TODO: I do not why, but depending on the CPU, these three lines can give
      // wrong output.
      //      typename Symmetric3::AlphaSkewSquare mcxcx(I.mass(),I.lever());
      //      const Symmetric3 I_mcxcx(I.inertia() - mcxcx);
      //      Iout_.template block<3,3>(ANGULAR,ANGULAR) += I_mcxcx.vxs(v.angular());
      Symmetric3 mcxcx(typename Symmetric3::AlphaSkewSquare(I.mass(), I.lever()));
      Iout_.template block<3, 3>(ANGULAR, ANGULAR) += I.inertia().vxs(v.angular());
      Iout_.template block<3, 3>(ANGULAR, ANGULAR) -= mcxcx.vxs(v.angular());
    }

    template<typename MotionDerived, typename M6>
    static void ivx_impl(
      const MotionDense<MotionDerived> & v,
      const InertiaTpl & I,
      const Eigen::MatrixBase<M6> & Iout)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6, 6, 6);
      M6 & Iout_ = PINOCCHIO_EIGEN_CONST_CAST(M6, Iout);

      // Block 1,1
      alphaSkew(I.mass(), v.angular(), Iout_.template block<3, 3>(LINEAR, LINEAR));

      // Block 2,1
      const Vector3 mc(I.mass() * I.lever());
      skewSquare(mc, v.angular(), Iout_.template block<3, 3>(ANGULAR, LINEAR));

      // Block 1,2
      alphaSkew(I.mass(), v.linear(), Iout_.template block<3, 3>(LINEAR, ANGULAR));

      // Block 2,2
      cross(
        -I.lever(), Iout_.template block<3, 3>(ANGULAR, LINEAR),
        Iout_.template block<3, 3>(ANGULAR, ANGULAR));
      Iout_.template block<3, 3>(ANGULAR, ANGULAR) += I.inertia().svx(v.angular());
      for (int k = 0; k < 3; ++k)
        Iout_.template block<3, 3>(ANGULAR, ANGULAR).col(k) +=
          I.lever().cross(Iout_.template block<3, 3>(LINEAR, ANGULAR).col(k));

      // Block 1,2
      Iout_.template block<3, 3>(LINEAR, ANGULAR) -= Iout_.template block<3, 3>(ANGULAR, LINEAR);
    }

    // Getters
    Scalar mass() const
    {
      return m_mass;
    }
    const Vector3 & lever() const
    {
      return m_com;
    }
    const Symmetric3 & inertia() const
    {
      return m_inertia;
    }

    Scalar & mass()
    {
      return m_mass;
    }
    Vector3 & lever()
    {
      return m_com;
    }
    Symmetric3 & inertia()
    {
      return m_inertia;
    }

    /// aI = aXb.act(bI)
    template<typename S2, int O2>
    InertiaTpl se3Action_impl(const SE3Tpl<S2, O2> & M) const
    {
      /* The multiplication RIR' has a particular form that could be used, however it
       * does not seems to be more efficient, see http://stackoverflow.m_comom/questions/
       * 13215467/eigen-best-way-to-evaluate-asa-transpose-and-store-the-result-in-a-symmetric .*/
      return InertiaTpl(
        mass(), M.translation() + M.rotation() * lever(), inertia().rotate(M.rotation()));
    }

    /// bI = aXb.actInv(aI)
    template<typename S2, int O2>
    InertiaTpl se3ActionInverse_impl(const SE3Tpl<S2, O2> & M) const
    {
      return InertiaTpl(
        mass(), M.rotation().transpose() * (lever() - M.translation()),
        inertia().rotate(M.rotation().transpose()));
    }

    template<typename MotionDerived>
    Force vxiv(const MotionDense<MotionDerived> & v) const
    {
      const Vector3 & mcxw = mass() * lever().cross(v.angular());
      const Vector3 & mv_mcxw = mass() * v.linear() - mcxw;
      return Force(
        v.angular().cross(mv_mcxw),
        v.angular().cross(lever().cross(mv_mcxw) + inertia() * v.angular())
          - v.linear().cross(mcxw));
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  m = " << mass() << "\n"
         << "  c = " << lever().transpose() << "\n"
         << "  I = \n"
         << inertia().matrix() << "";
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    InertiaTpl<NewScalar, Options> cast() const
    {
      return InertiaTpl<NewScalar, Options>(
        pinocchio::cast<NewScalar>(mass()), lever().template cast<NewScalar>(),
        inertia().template cast<NewScalar>());
    }

    // TODO: adjust code
    //    /// \brief Check whether *this is a valid inertia, resulting from a positive mass
    //    distribution bool isValid() const
    //    {
    //      return
    //         (m_mass >  Scalar(0) && m_inertia.isValid())
    //      || (m_mass == Scalar(0) && (m_inertia.data().array() == Scalar(0)).all());
    //    }

  protected:
    Scalar m_mass;
    Vector3 m_com;
    Symmetric3 m_inertia;

  }; // class InertiaTpl

  /**
   * @brief A structure representing a pseudo inertia matrix.
   *
   * References:
   * - Wensing, Patrick M., Sangbae Kim, and Jean-Jacques E. Slotine. "Linear matrix inequalities
   * for physically consistent inertial parameter identification: A statistical perspective on the
   * mass distribution." IEEE Robotics and Automation Letters 3.1 (2017): 60-67.
   */
  template<typename _Scalar, int _Options>
  struct PseudoInertiaTpl
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
    };
    typedef Eigen::Matrix<Scalar, 4, 4, Options> Matrix4;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;
    typedef Eigen::Matrix<Scalar, 10, 1, Options> Vector10;
    typedef LogCholeskyParametersTpl<Scalar, Options> LogCholeskyParameters;

    Scalar mass;   ///< Mass of the pseudo inertia
    Vector3 h;     ///< Vector part of the pseudo inertia
    Matrix3 sigma; ///< 3x3 matrix part of the pseudo inertia

    PseudoInertiaTpl(Scalar mass, const Vector3 & h, const Matrix3 & sigma)
    : mass(mass)
    , h(h)
    , sigma(sigma)
    {
    }

    /**
     * @brief Converts the PseudoInertiaTpl object to a 4x4 matrix.
     * @return A 4x4 pseudo inertia matrix.
     */
    Matrix4 toMatrix() const
    {
      Matrix4 pseudo_inertia = Matrix4::Zero();
      pseudo_inertia.template block<3, 3>(0, 0) = sigma;
      pseudo_inertia.template block<3, 1>(0, 3) = h;
      pseudo_inertia.template block<1, 3>(3, 0) = h.transpose();
      pseudo_inertia(3, 3) = mass;
      return pseudo_inertia;
    }

    /**
     * @brief Constructs a PseudoInertiaTpl object from a 4x4 pseudo inertia matrix.
     * @param pseudo_inertia A 4x4 pseudo inertia matrix.
     * @return A PseudoInertiaTpl object.
     */
    static PseudoInertiaTpl FromMatrix(const Matrix4 & pseudo_inertia)
    {
      Scalar mass = pseudo_inertia(3, 3);
      Vector3 h = pseudo_inertia.template block<3, 1>(0, 3);
      Matrix3 sigma = pseudo_inertia.template block<3, 3>(0, 0);
      return PseudoInertiaTpl(mass, h, sigma);
    }

    /**
     * @brief Constructs a PseudoInertiaTpl object from dynamic parameters.
     * @param dynamic_params A 10-dimensional vector of dynamic parameters.
     * @return A PseudoInertiaTpl object.
     */
    template<typename Vector10Like>
    static PseudoInertiaTpl
    FromDynamicParameters(const Eigen::MatrixBase<Vector10Like> & dynamic_params)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector10Like, dynamic_params, 10, 1);
      Scalar mass = dynamic_params[0];
      Vector3 h = dynamic_params.template segment<3>(1);
      Matrix3 I_bar;
      // clang-format off
      I_bar << dynamic_params[4], dynamic_params[5], dynamic_params[7],
               dynamic_params[5], dynamic_params[6], dynamic_params[8],
               dynamic_params[7], dynamic_params[8], dynamic_params[9];
      // clang-format on

      Matrix3 sigma = 0.5 * I_bar.trace() * Matrix3::Identity() - I_bar;
      return PseudoInertiaTpl(mass, h, sigma);
    }

    /**
     * @brief Converts the PseudoInertiaTpl object to dynamic parameters.
     * @return A 10-dimensional vector of dynamic parameters.
     */
    Vector10 toDynamicParameters() const
    {
      Matrix3 I_bar = sigma.trace() * Matrix3::Identity() - sigma;

      Vector10 dynamic_params;
      dynamic_params[0] = mass;
      dynamic_params.template segment<3>(1) = h;
      dynamic_params[4] = I_bar(0, 0);
      dynamic_params[5] = I_bar(0, 1);
      dynamic_params[6] = I_bar(1, 1);
      dynamic_params[7] = I_bar(0, 2);
      dynamic_params[8] = I_bar(1, 2);
      dynamic_params[9] = I_bar(2, 2);

      return dynamic_params;
    }

    /**
     * @brief Constructs a PseudoInertiaTpl object from an InertiaTpl object.
     * @param inertia An InertiaTpl object.
     * @return A PseudoInertiaTpl object.
     */
    static PseudoInertiaTpl FromInertia(const InertiaTpl<Scalar, Options> & inertia)
    {
      Vector10 dynamic_params = inertia.toDynamicParameters();
      return FromDynamicParameters(dynamic_params);
    }

    /**
     * @brief Converts the PseudoInertiaTpl object to an InertiaTpl object.
     * @return An InertiaTpl object.
     */
    InertiaTpl<Scalar, Options> toInertia() const
    {
      Vector10 dynamic_params = toDynamicParameters();
      return InertiaTpl<Scalar, Options>::FromDynamicParameters(dynamic_params);
    }

    /**
     * @brief Constructs a PseudoInertiaTpl object from log Cholesky parameters.
     * @param log_cholesky A 10-dimensional vector of log Cholesky parameters.
     * @return A PseudoInertiaTpl object.
     */
    static PseudoInertiaTpl FromLogCholeskyParameters(const LogCholeskyParameters & log_cholesky)
    {
      Vector10 dynamic_params = log_cholesky.toDynamicParameters();
      return FromDynamicParameters(dynamic_params);
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    PseudoInertiaTpl<NewScalar, Options> cast() const
    {
      return PseudoInertiaTpl<NewScalar, Options>(
        pinocchio::cast<NewScalar>(mass), h.template cast<NewScalar>(),
        sigma.template cast<NewScalar>());
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  m = " << mass << "\n"
         << "  h = " << h.transpose() << "\n"
         << "  sigma = \n"
         << sigma << "";
    }

    friend std::ostream & operator<<(std::ostream & os, const PseudoInertiaTpl & pi)
    {
      pi.disp_impl(os);
      return os;
    }
  };

  /**
   * @brief A structure representing log Cholesky parameters.
   *
   * References:
   * - Rucker, Caleb, and Patrick M. Wensing. "Smooth parameterization of rigid-body inertia."
   * IEEE Robotics and Automation Letters 7.2 (2022): 2771-2778.
   */
  template<typename _Scalar, int _Options>
  struct LogCholeskyParametersTpl
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
    };
    typedef Eigen::Matrix<Scalar, 10, 1, Options> Vector10;
    typedef Eigen::Matrix<Scalar, 10, 10, Options> Matrix10;
    typedef PseudoInertiaTpl<Scalar, Options> PseudoInertia;

    Vector10 parameters; ///< 10-dimensional vector of log Cholesky parameters
    /**
     * @brief Constructor for LogCholeskyParametersTpl.
     * @param log_cholesky A 10-dimensional vector of log Cholesky parameters.
     */
    explicit LogCholeskyParametersTpl(const Vector10 & log_cholesky)
    : parameters(log_cholesky)
    {
    }
    /**
     * @brief Converts the LogCholeskyParametersTpl object to dynamic parameters.
     * @return A 10-dimensional vector of dynamic parameters.
     */
    Vector10 toDynamicParameters() const
    {
      using math::exp;
      using math::pow;

      // clang-format off
      Vector10 dynamic_params;

      const Scalar alpha = parameters[0];
      const Scalar d1 = parameters[1];
      const Scalar d2 = parameters[2];
      const Scalar d3 = parameters[3];
      const Scalar s12 = parameters[4];
      const Scalar s23 = parameters[5];
      const Scalar s13 = parameters[6];
      const Scalar t1 = parameters[7];
      const Scalar t2 = parameters[8];
      const Scalar t3 = parameters[9];

      const Scalar exp_d1 = exp(d1);
      const Scalar exp_d2 = exp(d2);
      const Scalar exp_d3 = exp(d3);

      dynamic_params[0] = 1;
      dynamic_params[1] = t1;
      dynamic_params[2] = t2;
      dynamic_params[3] = t3;
      dynamic_params[4] = pow(s23, 2) + pow(t2, 2) + pow(t3, 2) + pow(exp_d2, 2) + pow(exp_d3, 2);
      dynamic_params[5] = -s12 * exp_d2 - s13 * s23 - t1 * t2;
      dynamic_params[6] = pow(s12, 2) + pow(s13, 2) + pow(t1, 2) + pow(t3, 2) + pow(exp_d1, 2) + pow(exp_d3, 2);
      dynamic_params[7] = -s13 * exp_d3 - t1 * t3;
      dynamic_params[8] = -s23 * exp_d3 - t2 * t3;
      dynamic_params[9] = pow(s12, 2) + pow(s13, 2) + pow(s23, 2) + pow(t1, 2) + pow(t2, 2) + pow(exp_d1, 2) + pow(exp_d2, 2);

      const Scalar exp_2_alpha = exp(2 * alpha);
      dynamic_params *= exp_2_alpha;
      // clang-format on

      return dynamic_params;
    }

    /**
     * @brief Converts the LogCholeskyParametersTpl object to a PseudoInertiaTpl object.
     * @return A PseudoInertiaTpl object.
     */
    PseudoInertia toPseudoInertia() const
    {
      Vector10 dynamic_params = toDynamicParameters();
      return PseudoInertia::FromDynamicParameters(dynamic_params);
    }

    /**
     * @brief Converts the LogCholeskyParametersTpl object to an InertiaTpl object.
     * @return An InertiaTpl object.
     */
    InertiaTpl<Scalar, Options> toInertia() const
    {
      Vector10 dynamic_params = toDynamicParameters();
      return InertiaTpl<Scalar, Options>::FromDynamicParameters(dynamic_params);
    }

    /**
     * @brief Calculates the Jacobian of the log Cholesky parameters.
     * @return A 10x10 matrix representing the Jacobian.
     */
    Matrix10 calculateJacobian() const
    {
      using math::exp;
      using math::pow;

      Matrix10 jacobian = Matrix10::Zero();

      const Scalar alpha = parameters[0];
      const Scalar d1 = parameters[1];
      const Scalar d2 = parameters[2];
      const Scalar d3 = parameters[3];
      const Scalar s12 = parameters[4];
      const Scalar s23 = parameters[5];
      const Scalar s13 = parameters[6];
      const Scalar t1 = parameters[7];
      const Scalar t2 = parameters[8];
      const Scalar t3 = parameters[9];

      const Scalar exp_2alpha = exp(2 * alpha);
      const Scalar exp_2d1 = exp(2 * d1);
      const Scalar exp_2d2 = exp(2 * d2);
      const Scalar exp_2d3 = exp(2 * d3);
      const Scalar exp_d1 = exp(d1);
      const Scalar exp_d2 = exp(d2);
      const Scalar exp_d3 = exp(d3);

      // clang-format off
      jacobian(0, 0) = 2 * exp_2alpha;

      jacobian(1, 0) = 2 * t1 * exp_2alpha;
      jacobian(1, 7) = exp_2alpha;

      jacobian(2, 0) = 2 * t2 * exp_2alpha;
      jacobian(2, 8) = exp_2alpha;

      jacobian(3, 0) = 2 * t3 * exp_2alpha;
      jacobian(3, 9) = exp_2alpha;

      jacobian(4, 0) = 2 * (pow(s23, 2) + pow(t2, 2) + pow(t3, 2) + exp_2d2 + exp_2d3) * exp_2alpha;
      jacobian(4, 2) = 2 * exp_2alpha * exp_2d2;
      jacobian(4, 3) = 2 * exp_2alpha * exp_2d3;
      jacobian(4, 5) = 2 * s23 * exp_2alpha;
      jacobian(4, 8) = 2 * t2 * exp_2alpha;
      jacobian(4, 9) = 2 * t3 * exp_2alpha;

      jacobian(5, 0) = -2 * (s12 * exp_d2 + s13 * s23 + t1 * t2) * exp_2alpha;
      jacobian(5, 2) = -s12 * exp_2alpha * exp_d2;
      jacobian(5, 4) = -exp_2alpha * exp_d2;
      jacobian(5, 5) = -s13 * exp_2alpha;
      jacobian(5, 6) = -s23 * exp_2alpha;
      jacobian(5, 7) = -t2 * exp_2alpha;
      jacobian(5, 8) = -t1 * exp_2alpha;

      jacobian(6, 0) = 2 * (pow(s12, 2) + pow(s13, 2) + pow(t1, 2) + pow(t3, 2) + exp_2d1 + exp_2d3) * exp_2alpha;
      jacobian(6, 1) = 2 * exp_2alpha * exp_2d1;
      jacobian(6, 3) = 2 * exp_2alpha * exp_2d3;
      jacobian(6, 4) = 2 * s12 * exp_2alpha;
      jacobian(6, 6) = 2 * s13 * exp_2alpha;
      jacobian(6, 7) = 2 * t1 * exp_2alpha;
      jacobian(6, 9) = 2 * t3 * exp_2alpha;

      jacobian(7, 0) = -2 * (s13 * exp_d3 + t1 * t3) * exp_2alpha;
      jacobian(7, 3) = -s13 * exp_2alpha * exp_d3;
      jacobian(7, 6) = -exp_2alpha * exp_d3;
      jacobian(7, 7) = -t3 * exp_2alpha;
      jacobian(7, 9) = -t1 * exp_2alpha;

      jacobian(8, 0) = -2 * (s23 * exp_d3 + t2 * t3) * exp_2alpha;
      jacobian(8, 3) = -s23 * exp_2alpha * exp_d3;
      jacobian(8, 5) = -exp_2alpha * exp_d3;
      jacobian(8, 8) = -t3 * exp_2alpha;
      jacobian(8, 9) = -t2 * exp_2alpha;

      jacobian(9, 0) = 2 * (pow(s12, 2) + pow(s13, 2) + pow(s23, 2) + pow(t1, 2) + pow(t2, 2) + exp_2d1 + exp_2d2) * exp_2alpha;
      jacobian(9, 1) = 2 * exp_2alpha * exp_2d1;
      jacobian(9, 2) = 2 * exp_2alpha * exp_2d2;
      jacobian(9, 4) = 2 * s12 * exp_2alpha;
      jacobian(9, 5) = 2 * s23 * exp_2alpha;
      jacobian(9, 6) = 2 * s13 * exp_2alpha;
      jacobian(9, 7) = 2 * t1 * exp_2alpha;
      jacobian(9, 8) = 2 * t2 * exp_2alpha;
      // clang-format on

      return jacobian;
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    LogCholeskyParametersTpl<NewScalar, Options> cast() const
    {
      return LogCholeskyParametersTpl<NewScalar, Options>(parameters.template cast<NewScalar>());
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  alpha: " << parameters[0] << "\n"
         << "  d1: " << parameters[1] << "\n"
         << "  d2: " << parameters[2] << "\n"
         << "  d3: " << parameters[3] << "\n"
         << "  s12: " << parameters[4] << "\n"
         << "  s23: " << parameters[5] << "\n"
         << "  s13: " << parameters[6] << "\n"
         << "  t1: " << parameters[7] << "\n"
         << "  t2: " << parameters[8] << "\n"
         << "  t3: " << parameters[9] << "";
    }

    friend std::ostream & operator<<(std::ostream & os, const LogCholeskyParametersTpl & lcp)
    {
      lcp.disp_impl(os);
      return os;
    }
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_spatial_inertia_hpp__
