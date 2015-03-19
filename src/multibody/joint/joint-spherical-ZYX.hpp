#ifndef __se3_joint_spherical_ZYX_hpp__
#define __se3_joint_spherical_ZYX_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  struct JointDataSphericalZYX;
  struct JointModelSphericalZYX;

  struct JointSphericalZYX
  {
    struct BiasSpherical
    {
      Motion::Vector3 c_J;

      BiasSpherical ()  {c_J.fill (NAN);}
      //BiasSpherical (const Motion::Vector3 & c_J) c_J (c_J) {}

      operator Motion () const { return Motion (Motion::Vector3::Zero (), c_J); }

      Motion::Vector3 & operator() () { return c_J; }
      const Motion::Vector3 & operator() () const { return c_J; }


    }; // struct BiasSpherical

    friend const Motion operator+ (const Motion & v, const BiasSpherical & c) { return Motion (v.linear (), v.angular () + c ()); }
    friend const Motion operator+ (const BiasSpherical & c, const Motion & v) { return Motion (v.linear (), v.angular () + c ()); }

    struct MotionSpherical
    {
      MotionSpherical ()                       {w.fill(NAN);}
      MotionSpherical (const Motion::Vector3 & w) : w (w)  {}
      Motion::Vector3 w;

      Motion::Vector3 & operator() () { return w; }
      const Motion::Vector3 & operator() () const { return w; }

      operator Motion() const
      {
        return Motion (Motion::Vector3::Zero (), w);
      }
    }; // struct MotionSpherical

    friend const MotionSpherical operator+ (const MotionSpherical & m, const BiasSpherical & c)
    { return MotionSpherical (m.w + c.c_J); }

    friend Motion operator+ (const MotionSpherical & m1, const Motion & m2)
    {
      return Motion( m2.linear(),m2.angular() + m1.w);
    }

    struct ConstraintRotationalSubspace
    {
    public:
      typedef Motion::Scalar Scalar;
      typedef Eigen::Matrix <Motion::Scalar,3,3> Matrix3;

    public:
      Matrix3 S_minimal;

      Motion operator* (const MotionSpherical & vj) const
      { return Motion (Motion::Vector3::Zero (), S_minimal * vj ()); }

      ConstraintRotationalSubspace () : S_minimal () { S_minimal.fill (NAN); }
      ConstraintRotationalSubspace (const Matrix3 & subspace) : S_minimal (subspace) {}

      Matrix3 & operator() () { return S_minimal; }
      const Matrix3 & operator() () const { return S_minimal; }

      Matrix3 &  matrix () { return S_minimal; }
      const Matrix3 & matrix () const { return S_minimal; }

      struct ConstraintTranspose
      {
        const ConstraintRotationalSubspace & ref;
        ConstraintTranspose(const ConstraintRotationalSubspace & ref) : ref(ref) {}

        Force::Vector3 operator* (const Force & phi)
        {
          return ref.S_minimal.transpose () * phi.angular ();
        }


        /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
        template<typename D>
        friend typename Eigen::Matrix <typename Eigen::MatrixBase<D>::Scalar, 3, -1>
        operator*( const ConstraintTranspose & constraint_transpose, const Eigen::MatrixBase<D> & F )
        {
          assert(F.rows()==6);
          return constraint_transpose.ref.S_minimal.transpose () * F.template middleRows <3> (Inertia::ANGULAR);
        }
      }; // struct ConstraintTranspose

      ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }

      operator ConstraintXd () const
      {
        Eigen::Matrix<Scalar,6,3> S;
        S.block <3,3> (Inertia::LINEAR, 0).setZero ();
        S.block <3,3> (Inertia::ANGULAR, 0) = S_minimal;
        return ConstraintXd(S);
      }

      Eigen::Matrix <Scalar,6,3> se3Action (const SE3 & m) const
      {
        Eigen::Matrix <Scalar,6,3> X_subspace;
        X_subspace.block <3,3> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ();
        X_subspace.block <3,3> (Motion::ANGULAR, 0) = m.rotation ();

        return X_subspace * S_minimal;
      }

    }; // struct ConstraintRotationalSubspace

    template<typename D>
    friend Motion operator* (const ConstraintRotationalSubspace & S, const Eigen::MatrixBase<D> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
      return Motion (Motion::Vector3::Zero (), S () * v);
    }

  }; // struct JointSphericalZYX

  Motion operator^ (const Motion & m1, const JointSphericalZYX::MotionSpherical & m2)
  {
//    const Motion::Matrix3 m2_cross (skew (Motion::Vector3 (-m2.w)));
//    return Motion(m2_cross * m1.linear (), m2_cross * m1.angular ());
    return Motion(m1.linear ().cross (m2.w), m1.angular ().cross (m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix <Inertia::Scalar, 6, 3> operator* (const Inertia & Y, const JointSphericalZYX::ConstraintRotationalSubspace & S)
  {
    Eigen::Matrix <Inertia::Scalar, 6, 3> M;
//    M.block <3,3> (Inertia::LINEAR, 0) = - Y.mass () * skew(Y.lever ());
    M.block <3,3> (Inertia::LINEAR, 0) = alphaSkew ( -Y.mass (),  Y.lever ());
    M.block <3,3> (Inertia::ANGULAR, 0) = (Inertia::Matrix3)(Y.inertia () - typename Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ()));
    return M * S.matrix ();
  }

  namespace internal
  {
    template<>
    struct ActionReturn<JointSphericalZYX::ConstraintRotationalSubspace >
    { typedef Eigen::Matrix<double,6,3> Type; };
  }

  template<>
  struct traits<JointSphericalZYX>
  {
    typedef JointDataSphericalZYX JointData;
    typedef JointModelSphericalZYX JointModel;
    typedef JointSphericalZYX::ConstraintRotationalSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef JointSphericalZYX::MotionSpherical Motion_t;
    typedef JointSphericalZYX::BiasSpherical Bias_t;
    typedef Eigen::Matrix<double,6,3> F_t;
    enum {
      NQ = 3,
      NV = 3
    };
  };
  template<> struct traits<JointDataSphericalZYX> { typedef JointSphericalZYX Joint; };
  template<> struct traits<JointModelSphericalZYX> { typedef JointSphericalZYX Joint; };

  struct JointDataSphericalZYX : public JointDataBase<JointDataSphericalZYX>
  {
    typedef JointSphericalZYX Joint;
    SE3_JOINT_TYPEDEF;

    typedef Motion::Scalar Scalar;

    typedef Eigen::Matrix<Scalar,6,6> Matrix6;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    JointDataSphericalZYX () : M(1)
    {
      M.translation (Transformation_t::Vector3::Zero ());
    }
  };

  struct JointModelSphericalZYX : public JointModelBase<JointModelSphericalZYX>
  {
    typedef JointSphericalZYX Joint;
    SE3_JOINT_TYPEDEF;

    JointData createData() const { return JointData(); }

    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ>(idx_q ());

      double c0,s0; SINCOS (q(0), &s0, &c0);
      double c1,s1; SINCOS (q(1), &s1, &c1);
      double c2,s2; SINCOS (q(2), &s2, &c2);

      data.M.rotation () << c0 * c1,
                c0 * s1 * s2 - s0 * c2,
                c0 * s1 * c2 + s0 * s2,
                s0 * c1,
                s0 * s1 * s2 + c0 * c2,
                s0 * s1 * c2 - c0 * s2,
                -s1,
                c1 * s2,
                c1 * c2;

      data.S.matrix () <<  -s1, 0., 1., c1 * s2, c2, 0, c1 * c2, -s2, 0;
    }

    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NQ> (idx_v ());

      double c0,s0; SINCOS (q(0), &s0, &c0);
      double c1,s1; SINCOS (q(1), &s1, &c1);
      double c2,s2; SINCOS (q(2), &s2, &c2);

      data.M.rotation () << c0 * c1,
                c0 * s1 * s2 - s0 * c2,
                c0 * s1 * c2 + s0 * s2,
                s0 * c1,
                s0 * s1 * s2 + c0 * c2,
                s0 * s1 * c2 - c0 * s2,
                -s1,
                c1 * s2,
                c1 * c2;


      data.S.matrix () <<  -s1, 0., 1., c1 * s2, c2, 0, c1 * c2, -s2, 0;

      data.v () = data.S.matrix () * q_dot;

      data.c ()(0) = -c1 * q_dot (0) * q_dot (1);
      data.c ()(1) = -s1 * s2 * q_dot (0) * q_dot (1) + c1 * c2 * q_dot (0) * q_dot (2) - s2 * q_dot (1) * q_dot (2);
      data.c ()(2) = -s1 * c2 * q_dot (0) * q_dot (1) - c1 * s2 * q_dot (0) * q_dot (2) - c2 * q_dot (1) * q_dot (2);
    }
  };

} // namespace se3

#endif // ifndef __se3_joint_spherical_ZYX_hpp__