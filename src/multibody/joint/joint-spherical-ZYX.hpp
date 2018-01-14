//
// Copyright (c) 2015-2017 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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
#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

#include <stdexcept>

namespace se3
{
  
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
    typedef ForceTpl<Scalar,Options> Force;
    typedef SE3Tpl<Scalar,Options> SE3;

    struct BiasSpherical
    {
      typename MotionTpl<Scalar,Options>::Vector3 c_J;

      BiasSpherical ()  {c_J.fill (NAN);}
      //BiasSpherical (const Motion::Vector3 & c_J) c_J (c_J) {}

      operator Motion () const { return Motion (Motion::Vector3::Zero (), c_J); }

      operator BiasZero() const { return BiasZero();}

      typename MotionTpl<Scalar,Options>::Vector3 & operator() () { return c_J; }
      const typename MotionTpl<Scalar,Options>::Vector3 & operator() () const { return c_J; }


    }; // struct BiasSpherical

    inline friend const Motion operator+ (const Motion & v, const BiasSpherical & c) { return Motion (v.linear (), v.angular () + c ()); }
    inline friend const Motion operator+ (const BiasSpherical & c, const Motion & v) { return Motion (v.linear (), v.angular () + c ()); }

    struct MotionSpherical
    {
      typedef typename MotionTpl<Scalar,Options>::Vector3 Vector3;
      
      MotionSpherical () : w(Vector3::Constant(NAN)) {}
      MotionSpherical (const Vector3 & w) : w (w)  {}

      Vector3 & operator() () { return w; }
      const Vector3 & operator() () const { return w; }

      operator Motion() const { return Motion(Motion::Vector3::Zero(), w); }
      
      operator Vector3() const { return w; }
      
      Vector3 w;
    }; // struct MotionSpherical

    inline friend const MotionSpherical operator+ (const MotionSpherical & m, const BiasSpherical & c)
    { return MotionSpherical (m.w + c.c_J); }

    friend MotionTpl<_Scalar,_Options> operator+ (const MotionSpherical & m1, 
                                                  const MotionTpl<_Scalar,_Options> & m2)
    {
      return MotionTpl<_Scalar,_Options>( m2.linear(),m2.angular() + m1.w);
    }

    struct ConstraintRotationalSubspace
    {
      enum { NV = 3, Options = 0 };
      typedef _Scalar Scalar;
      typedef Eigen::Matrix <_Scalar,3,3,_Options> Matrix3;
      typedef Eigen::Matrix <_Scalar,3,1,_Options> Vector3;
      typedef Eigen::Matrix <_Scalar,6,3,_Options> ConstraintDense;
      typedef Eigen::Matrix <_Scalar,6,3,_Options> DenseBase;

      Matrix3 S_minimal;

      Motion operator* (const MotionSpherical & vj) const
      { return Motion (Motion::Vector3::Zero (), S_minimal * vj ()); }

      ConstraintRotationalSubspace () : S_minimal () { S_minimal.fill (NAN); }
      ConstraintRotationalSubspace (const Matrix3 & subspace) : S_minimal (subspace) {}

      Matrix3 & operator() () { return S_minimal; }
      const Matrix3 & operator() () const { return S_minimal; }

      Matrix3 & matrix () { return S_minimal; }
      const Matrix3 & matrix () const { return S_minimal; }

      int nv_impl() const { return NV; }

      struct ConstraintTranspose
      {
        const ConstraintRotationalSubspace & ref;
        ConstraintTranspose(const ConstraintRotationalSubspace & ref) : ref(ref) {}

#ifdef EIGEN3_FUTURE
        const typename Eigen::Product<
        Eigen::Transpose<const Matrix3>,
        Eigen::Block<const typename Force::Vector6,3,1>
        >
#else
        const typename Eigen::ProductReturnType<
        Eigen::Transpose<const Matrix3>,
//        typename Motion::ConstAngular_t::Base /* This feature leads currently to a bug */
        Eigen::Block<const typename Force::Vector6,3,1>
        >::Type
#endif
        operator* (const Force & phi) const
        {
          return ref.S_minimal.transpose () * phi.angular();
        }

        /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
        template<typename D>
#ifdef EIGEN3_FUTURE
        const typename Eigen::Product<
        typename Eigen::Transpose<const Matrix3>,
        typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
        >
#else
        const typename Eigen::ProductReturnType<
        typename Eigen::Transpose<const Matrix3>,
        typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
        >::Type
#endif
        operator* (const Eigen::MatrixBase<D> & F) const
        {
          EIGEN_STATIC_ASSERT(D::RowsAtCompileTime==6,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
          return ref.S_minimal.transpose () * F.template bottomRows<3> ();
        }
      }; // struct ConstraintTranspose

      ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }

      operator ConstraintXd () const
      {
        ConstraintDense S;
        (S.template block <3,3> (Inertia::LINEAR, 0)).setZero ();
        S.template block <3,3> (Inertia::ANGULAR, 0) = S_minimal;
        return ConstraintXd(S);
      }

//      const typename Eigen::ProductReturnType<
//      const ConstraintDense,
//      const Matrix3
//      >::Type
      Eigen::Matrix <Scalar,6,3, Options>
      se3Action (const SE3 & m) const
      {
//        Eigen::Matrix <Scalar,6,3,Options> X_subspace;
//        X_subspace.template block <3,3> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ();
//        X_subspace.template block <3,3> (Motion::ANGULAR, 0) = m.rotation ();
//
//        return (X_subspace * S_minimal).eval();
        
        Eigen::Matrix <Scalar,6,3,Options> result;
        result.template block <3,3> (Motion::ANGULAR, 0) = m.rotation () * S_minimal;
        for (int k = 0; k < 3; ++k)
          result.template block <3,3> (Motion::LINEAR, 0).col(k) =
          m.translation ().cross(result.template block <3,3> (Motion::ANGULAR, 0).col(k));
                                 
        return result;
      }
      
      DenseBase variation(const Motion & m) const
      {
        const typename Motion::ConstLinear_t v = m.linear();
        const typename Motion::ConstAngular_t w = m.angular();
        
        DenseBase res;
        res.template middleRows<3>(Motion::LINEAR) = cross(v,S_minimal);
        res.template middleRows<3>(Motion::ANGULAR) = cross(w,S_minimal);
        
        return res;
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

  inline Motion operator^ (const Motion & m1, const JointSphericalZYX::MotionSpherical & m2)
  {
//    const Motion::Matrix3 m2_cross (skew (Motion::Vector3 (-m2.w)));
//    return Motion(m2_cross * m1.linear (), m2_cross * m1.angular ());
    return Motion(m1.linear ().cross (m2.w), m1.angular ().cross (m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */

  template <typename _Scalar, int _Options>
//  const typename Eigen::ProductReturnType<
//  Eigen::Matrix < _Scalar, 6, 3, _Options >,
//  Eigen::Matrix < _Scalar, 3, 3, _Options> 
//  >::Type
  Eigen::Matrix <_Scalar,6,3,_Options>
  operator* (const InertiaTpl<_Scalar,_Options> & Y,
             const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace & S)
  {
    Eigen::Matrix < _Scalar, 6, 3, _Options > M;
    M.template topRows<3>() = alphaSkew ( -Y.mass (),  Y.lever () );
    M.template bottomRows<3> () =  (Y.inertia () -
       typename Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ())).matrix();

    return (M * S.matrix ()).eval();
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
//  template <typename _Scalar, int _Options>
//  inline Eigen::Matrix<_Scalar,6,3,_Options>
//#ifdef EIGEN3_FUTURE
//  const typename Eigen::Product<
//  const Eigen::template Block<const typename InertiaTpl<_Scalar,_Options>::Matrix6,6,3>,
//  const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace::Matrix3
//  >
//#else
//  const typename Eigen::ProductReturnType<
//  const Eigen::template Block<const typename InertiaTpl<_Scalar,_Options>::Matrix6,6,3>,
//  const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace::Matrix3
//  >::Type
//#endif
//  operator*(const typename InertiaTpl<_Scalar,_Options>::Matrix6 & Y,
//            const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace & S)
//  {
//    return Y.template middleCols<3>(Inertia::ANGULAR) * S.S_minimal;
//  }
  
  inline Eigen::Matrix<double,6,3>
  operator*(const Inertia::Matrix6 & Y,
            const JointSphericalZYX::ConstraintRotationalSubspace & S)
  {
    return (Y.middleCols<3>(Inertia::ANGULAR) * S.S_minimal).eval();
  }

  namespace internal
  {
    template<>
    struct ActionReturn<JointSphericalZYX::ConstraintRotationalSubspace >
    {
//      typedef const typename Eigen::ProductReturnType<
//      Eigen::Matrix <double,6,3,0>,
//      Eigen::Matrix <double,3,3,0>
//      >::Type Type;
      typedef Eigen::Matrix <double,6,3,0> Type;
    };
  }

  template<>
  struct traits<JointSphericalZYX>
  {
    enum {
      NQ = 3,
      NV = 3
    };
    typedef double Scalar;
    typedef JointDataSphericalZYX JointDataDerived;
    typedef JointModelSphericalZYX JointModelDerived;
    typedef JointSphericalZYX::ConstraintRotationalSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef JointSphericalZYX::MotionSpherical Motion_t;
    typedef JointSphericalZYX::BiasSpherical Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };
  template<> struct traits<JointDataSphericalZYX> { typedef JointSphericalZYX JointDerived; };
  template<> struct traits<JointModelSphericalZYX> { typedef JointSphericalZYX JointDerived; };

  struct JointDataSphericalZYX : public JointDataBase<JointDataSphericalZYX>
  {
    typedef JointSphericalZYX JointDerived;
    SE3_JOINT_TYPEDEF;


    typedef Eigen::Matrix<Scalar,6,6> Matrix6;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataSphericalZYX () : M(1), U(), Dinv(), UDinv() {}
    
  }; // strcut JointDataSphericalZYX

  struct JointModelSphericalZYX : public JointModelBase<JointModelSphericalZYX>
  {
    typedef JointSphericalZYX JointDerived;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelSphericalZYX>::id;
    using JointModelBase<JointModelSphericalZYX>::idx_q;
    using JointModelBase<JointModelSphericalZYX>::idx_v;
    using JointModelBase<JointModelSphericalZYX>::setIndexes;
    typedef Motion::Vector3 Vector3;

    JointDataDerived createData() const { return JointDataDerived(); }

    void calc (JointDataDerived & data,
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

    void calc (JointDataDerived & data,
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
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.middleCols<3> (Inertia::ANGULAR) * data.S.matrix();
      Inertia::Matrix3 tmp (data.S.matrix().transpose() * data.U.middleRows<3> (Inertia::ANGULAR));
      data.Dinv = tmp.inverse();
      data.UDinv = data.U * data.Dinv;
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());


      return(q + q_dot);
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    { 
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_0 = q0.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());

      return ((1-u) * q_0 + u * q_1);
    }

    ConfigVector_t random_impl() const
    { 
      ConfigVector_t result(ConfigVector_t::Random());
      return result;
    } 

    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & lower_pos_limit, const ConfigVector_t & upper_pos_limit ) const throw (std::runtime_error)
    { 
      ConfigVector_t result;
      for (int i = 0; i < result.size(); ++i)
      {
        if(lower_pos_limit[i] == -std::numeric_limits<double>::infinity() || 
            upper_pos_limit[i] == std::numeric_limits<double>::infinity() )
        {
          std::ostringstream error;
          error << "non bounded limit. Cannot uniformly sample joint nb " << id() ;
          assert(false && "non bounded limit. Cannot uniformly sample joint spherical ZYX");
          throw std::runtime_error(error.str());
        }
        result[i] = lower_pos_limit[i] + ( upper_pos_limit[i] - lower_pos_limit[i]) * rand()/RAND_MAX;
      }
      return result;
    } 

    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_0 = q0.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());

      return ( q_1 - q_0);

    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return difference_impl(q0, q1).norm();
    }
    
    ConfigVector_t neutralConfiguration_impl() const
    { 
      ConfigVector_t q;
      q << 0, 0, 0;
      return q;
    }

    bool isSameConfiguration_impl(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_2 = q2.segment<NQ> (idx_q ());

      return q_1.isApprox(q_2, prec);
    } 

    static std::string classname() { return std::string("JointModelSphericalZYX"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelSphericalZYX

} // namespace se3

#endif // ifndef __se3_joint_spherical_ZYX_hpp__
