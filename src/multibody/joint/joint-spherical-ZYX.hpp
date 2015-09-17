//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_joint_spherical_ZYX_hpp__
#define __se3_joint_spherical_ZYX_hpp__
#include <iostream>
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  struct JointDataSphericalZYX;
  struct JointModelSphericalZYX;
  
  template <typename _Scalar, int _Options>
  struct JointSphericalZYXTpl
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef SE3Tpl<Scalar,Options> SE3;

    struct BiasSpherical
    {
      typename MotionTpl<Scalar,Options>::Vector3 c_J;

      BiasSpherical ()  {c_J.fill (NAN);}
      //BiasSpherical (const Motion::Vector3 & c_J) c_J (c_J) {}

      operator Motion () const { return Motion (Motion::Vector3::Zero (), c_J); }

      typename MotionTpl<Scalar,Options>::Vector3 & operator() () { return c_J; }
      const typename MotionTpl<Scalar,Options>::Vector3 & operator() () const { return c_J; }


    }; // struct BiasSpherical

    friend const Motion operator+ (const Motion & v, const BiasSpherical & c) { return Motion (v.linear (), v.angular () + c ()); }
    friend const Motion operator+ (const BiasSpherical & c, const Motion & v) { return Motion (v.linear (), v.angular () + c ()); }

    struct MotionSpherical
    {
      MotionSpherical ()                       {w.fill(NAN);}
      MotionSpherical (const typename MotionTpl<Scalar,Options>::Vector3 & w) : w (w)  {}
      typename MotionTpl<Scalar,Options>::Vector3 w;

      typename MotionTpl<Scalar,Options>::Vector3 & operator() () { return w; }
      const typename MotionTpl<Scalar,Options>::Vector3 & operator() () const { return w; }

      operator Motion() const
      {
        //return Motion (typename MotionTpl<_Scalar,_Options>::Vector3::Zero (), w);
        return Motion ( Motion::Vector3::Zero (), w);
      }
    }; // struct MotionSpherical

    friend const MotionSpherical operator+ (const MotionSpherical & m, const BiasSpherical & c)
    { return MotionSpherical (m.w + c.c_J); }

    friend MotionTpl<_Scalar,_Options> operator+ (const MotionSpherical & m1, 
                                                  const MotionTpl<_Scalar,_Options> & m2)
    {
      return MotionTpl<_Scalar,_Options>( m2.linear(),m2.angular() + m1.w);
    }

    struct ConstraintRotationalSubspace
    {
    public:
      typedef _Scalar Scalar;
      typedef Eigen::Matrix <_Scalar,3,3,_Options> Matrix3;
      typedef Eigen::Matrix <_Scalar,3,1,_Options> Vector3;

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

        //const Force::Vector3 
        const Eigen::CoeffBasedProduct<
          Eigen::Transpose<Eigen::Matrix<double, 3, 3> >&,
          const Eigen::Matrix<double, 3, 1>&
          ,6>::PlainObject
        operator* (const Force & phi)
        {
          return ref.S_minimal.transpose () * phi.angular ();
        }


        /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
        template<typename D>
        friend 
//        typename Eigen::Matrix <typename Eigen::MatrixBase<D>::Scalar, 3, -1>
        const typename Eigen::ProductReturnType<
        Eigen::Transpose<const Eigen::Matrix<typename Eigen::MatrixBase<D>::Scalar, 3, 3> >,
        Eigen::Block<const Eigen::Block<Eigen::Matrix<typename Eigen::MatrixBase<D>::Scalar,6,-1>,-1,-1>, 3, -1>
        >::Type
        operator*( const ConstraintTranspose & constraint_transpose, const Eigen::MatrixBase<D> & F )
        {
          assert(F.rows()==6);
          return constraint_transpose.ref.S_minimal.transpose () * 
            F.template middleRows <3> (Inertia::ANGULAR);
        }
      }; // struct ConstraintTranspose

      ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }

      operator ConstraintXd () const
      {
        typedef Eigen::Matrix<_Scalar,6,3,_Options> Matrix63;
        Eigen::Matrix<_Scalar,6,3,_Options> S;
        (S.template block <3,3> (Inertia::LINEAR, 0)).setZero ();
        S.template block <3,3> (Inertia::ANGULAR, 0) = S_minimal;
        return ConstraintXd(S);
      }

      //Eigen::Matrix <Scalar,6,3> 
      const typename 
      Eigen::CoeffBasedProduct<const Eigen::Matrix<double, 6, 3>&, 
                               const Eigen::Matrix<double, 3, 3>&, 6>::PlainObject
      se3Action (const SE3 & m) const
      {
        Eigen::Matrix <Scalar,6,3> X_subspace;
        X_subspace.template block <3,3> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ();
        X_subspace.template block <3,3> (Motion::ANGULAR, 0) = m.rotation ();

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

  typedef JointSphericalZYXTpl<double,0> JointSphericalZYX;

  Motion operator^ (const Motion & m1, const JointSphericalZYX::MotionSpherical & m2)
  {
//    const Motion::Matrix3 m2_cross (skew (Motion::Vector3 (-m2.w)));
//    return Motion(m2_cross * m1.linear (), m2_cross * m1.angular ());
    return Motion(m1.linear ().cross (m2.w), m1.angular ().cross (m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template <typename _Scalar, int _Options>
  //  Eigen::Matrix <_Scalar, 6, 3,_Options> 
  const typename Eigen::CoeffBasedProduct<
    Eigen::Matrix < _Scalar, 6, 3, _Options >,
    Eigen::Matrix < _Scalar, 3, 3, _Options >, 6 >::PlainObject
  operator* (const InertiaTpl<_Scalar,_Options> & Y, 
                                           const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace & S)
  {
    Eigen::Matrix < _Scalar, 6, 3, _Options > M;
    M.template topRows<3>() = alphaSkew ( -Y.mass (),  Y.lever () );
    M.template bottomRows<3> () = 
      (typename InertiaTpl<_Scalar,_Options>::Matrix3)(Y.inertia () - 
       typename Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ()));

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

    typedef Motion::Scalar_t Scalar;

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
