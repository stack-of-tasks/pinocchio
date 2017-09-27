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

#ifndef __se3_joint_revolute_unaligned_hpp__
#define __se3_joint_revolute_unaligned_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include <stdexcept>

namespace se3
{

  struct MotionRevoluteUnaligned;
  template <>
  struct traits < MotionRevoluteUnaligned >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<double,0> Quaternion_t;
    typedef SE3Tpl<double,0> SE3;
    typedef ForceTpl<double,0> Force;
    typedef MotionTpl<double,0> Motion;
    typedef Symmetric3Tpl<double,0> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionRevoluteUnaligned

  struct MotionRevoluteUnaligned : MotionBase < MotionRevoluteUnaligned >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(MotionRevoluteUnaligned);

    MotionRevoluteUnaligned() : axis(Motion::Vector3::Constant(NAN)), w(NAN) {} 
    MotionRevoluteUnaligned( const Motion::Vector3 & axis, const double & w ) : axis(axis), w(w)  {}

    Vector3 axis;
    double w;

    operator Motion() const
    { 
      return Motion(Motion::Vector3::Zero(),
                    axis*w);
    }
  }; // struct MotionRevoluteUnaligned

  inline const MotionRevoluteUnaligned& operator+ (const MotionRevoluteUnaligned& m, const BiasZero&)
  { return m; }

  inline Motion operator+ (const MotionRevoluteUnaligned& m1, const Motion& m2)
  {
    return Motion( m2.linear(), m1.w*m1.axis+m2.angular() );
  }

  struct ConstraintRevoluteUnaligned;
  template <>
  struct traits < ConstraintRevoluteUnaligned >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Matrix3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<double,0> Quaternion_t;
    typedef SE3Tpl<double,0> SE3;
    typedef ForceTpl<double,0> Force;
    typedef MotionTpl<double,0> Motion;
    typedef Symmetric3Tpl<double,0> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef Eigen::Matrix<Scalar,1,1,0> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,0> JointForce;
    typedef Eigen::Matrix<Scalar,6,1> DenseBase;
  }; // traits ConstraintRevoluteUnaligned


    

    struct ConstraintRevoluteUnaligned : ConstraintBase <ConstraintRevoluteUnaligned>
    {
      SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintRevoluteUnaligned);
      enum { NV = 1, Options = 0 };
      typedef traits<ConstraintRevoluteUnaligned>::JointMotion JointMotion;
      typedef traits<ConstraintRevoluteUnaligned>::JointForce JointForce;
      typedef traits<ConstraintRevoluteUnaligned>::DenseBase DenseBase;

      ConstraintRevoluteUnaligned() : axis(Motion::Vector3::Constant(NAN)) {}
      ConstraintRevoluteUnaligned(const Motion::Vector3 & _axis) : axis(_axis) {}
      Vector3 axis;

      template<typename D>
      MotionRevoluteUnaligned operator*( const Eigen::MatrixBase<D> & v ) const
      { 
      	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,1);
      	return MotionRevoluteUnaligned(axis,v[0]); 
      }

      Eigen::Matrix<double,6,1> se3Action(const SE3 & m) const
      { 
        /* X*S = [ R pxR ; 0 R ] [ 0 ; a ] = [ px(Ra) ; Ra ] */
        Eigen::Matrix<double,6,1> res;
        res.tail<3>() = m.rotation() * axis;
        res.head<3>() = m.translation().cross(res.tail<3>());
        return res;
      }

      int nv_impl() const { return NV; }

      struct TransposeConst
      {
      	const ConstraintRevoluteUnaligned & ref; 
      	TransposeConst(const ConstraintRevoluteUnaligned & ref) : ref(ref) {} 

      	Eigen::Matrix<double,1,1> operator* (const Force & f) const
      	{
      	  return ref.axis.transpose()*f.angular();
      	}

        /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
        template<typename D>
        friend
#ifdef EIGEN3_FUTURE
        const Eigen::Product<
        Eigen::Transpose<const Vector3>,
        typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
        >
#else
        const typename Eigen::ProductReturnType<
        Eigen::Transpose<const Vector3>,
        typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
        >::Type
#endif
        operator* (const TransposeConst & tc, const Eigen::MatrixBase<D> & F)
        {
          EIGEN_STATIC_ASSERT(D::RowsAtCompileTime==6,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
          /* Return ax.T * F[3:end,:] */
          return tc.ref.axis.transpose () * F.template bottomRows<3> ();
        }

      };
      TransposeConst transpose() const { return TransposeConst(*this); }


      /* CRBA joint operators
       *   - ForceSet::Block = ForceSet
       *   - ForceSet operator* (Inertia Y,Constraint S)
       *   - MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
       *   - SE3::act(ForceSet::Block)
       */
      operator ConstraintXd () const
      {
      	DenseBase S;
      	S << Eigen::Vector3d::Zero(), axis;
      	return ConstraintXd(S);
      }
      
      DenseBase variation(const Motion & m) const
      {
        const Motion::ConstLinear_t v = m.linear();
        const Motion::ConstAngular_t w = m.angular();
        
        DenseBase res;
        res << v.cross(axis), w.cross(axis);
        
        return res;
      }
      
    }; // struct ConstraintRevoluteUnaligned


    inline Motion operator^( const Motion& m1, const MotionRevoluteUnaligned & m2)
    {
      /* m1xm2 = [ v1xw2 + w1xv2; w1xw2 ] = [ v1xw2; w1xw2 ] */
      const Motion::Vector3& v1 = m1.linear();
      const Motion::Vector3& w1 = m1.angular();
      const Motion::Vector3& w2 = m2.axis * m2.w ;
      return Motion( v1.cross(w2),w1.cross(w2));
    }

    /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
    inline Eigen::Matrix<double,6,1>
    operator*( const Inertia& Y,const ConstraintRevoluteUnaligned & cru)
    { 
      /* YS = [ m -mcx ; mcx I-mcxcx ] [ 0 ; w ] = [ mcxw ; Iw -mcxcxw ] */
      const double &m                 = Y.mass();
      const Inertia::Vector3 & c      = Y.lever();
      const Inertia::Symmetric3 & I   = Y.inertia();

      Eigen::Matrix<double,6,1> res;
      res.head<3>() = -m*c.cross(cru.axis);
      res.tail<3>() = I*cru.axis + c.cross(res.head<3>());
      return res;
    }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
//  inline Eigen::Matrix<double,6,1>
  inline
#ifdef EIGEN3_FUTURE
  const Eigen::Product<
  Eigen::Block<const Inertia::Matrix6,6,3>,
  ConstraintRevoluteUnaligned::Vector3
  >
#else
  const Eigen::ProductReturnType<
  Eigen::Block<const Inertia::Matrix6,6,3>,
  const ConstraintRevoluteUnaligned::Vector3
  >::Type
#endif
  operator*(const Inertia::Matrix6 & Y, const ConstraintRevoluteUnaligned & cru)
  {
    return Y.block<6,3> (0,Inertia::ANGULAR) * cru.axis;
  }
  
    namespace internal
    {
      template<>
      struct ActionReturn<ConstraintRevoluteUnaligned >  
      { typedef Eigen::Matrix<double,6,1> Type; };
    }

    struct JointRevoluteUnaligned;
    template<>
    struct traits< JointRevoluteUnaligned >
    {
      enum {
        NQ = 1,
        NV = 1
      };
      typedef double Scalar;
      typedef JointDataRevoluteUnaligned JointDataDerived;
      typedef JointModelRevoluteUnaligned JointModelDerived;
      typedef ConstraintRevoluteUnaligned Constraint_t;
      typedef SE3 Transformation_t;
      typedef MotionRevoluteUnaligned Motion_t;
      typedef BiasZero Bias_t;
      typedef Eigen::Matrix<double,6,NV> F_t;
      
      // [ABA]
      typedef Eigen::Matrix<double,6,NV> U_t;
      typedef Eigen::Matrix<double,NV,NV> D_t;
      typedef Eigen::Matrix<double,6,NV> UD_t;

      typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
      typedef Eigen::Matrix<double,NV,1> TangentVector_t;
      
    };

  template<> struct traits<JointDataRevoluteUnaligned> { typedef JointRevoluteUnaligned JointDerived; };
  template<> struct traits<JointModelRevoluteUnaligned> { typedef JointRevoluteUnaligned JointDerived; };

  struct JointDataRevoluteUnaligned : public JointDataBase< JointDataRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned JointDerived;
    SE3_JOINT_TYPEDEF;

    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataRevoluteUnaligned() 
      : M(1),S(Eigen::Vector3d::Constant(NAN)),v(Eigen::Vector3d::Constant(NAN),NAN)
      , U(), Dinv(), UDinv()
    {}
    
    JointDataRevoluteUnaligned(const Motion::Vector3 & axis) 
      : M(1),S(axis),v(axis,NAN)
      , U(), Dinv(), UDinv()
    {}

  }; // struct JointDataRevoluteUnaligned

  struct JointModelRevoluteUnaligned : public JointModelBase< JointModelRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned JointDerived;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRevoluteUnaligned>::id;
    using JointModelBase<JointModelRevoluteUnaligned>::idx_q;
    using JointModelBase<JointModelRevoluteUnaligned>::idx_v;
    using JointModelBase<JointModelRevoluteUnaligned>::setIndexes;
    typedef Motion::Vector3 Vector3;
    
    JointModelRevoluteUnaligned() : axis(Eigen::Vector3d::Constant(NAN))   {}
    JointModelRevoluteUnaligned(const double x, const double y, const double z)
    {
      axis << x, y, z ;
      axis.normalize();
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }
    JointModelRevoluteUnaligned(const Motion::Vector3 & axis) : axis(axis)
    {
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }

    JointDataDerived createData() const { return JointDataDerived(axis); }
    void calc( JointDataDerived& data, 
	       const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];
      
      data.M.rotation(Eigen::AngleAxisd(q, axis).toRotationMatrix());
    }

    void calc( JointDataDerived& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      data.M.rotation(Eigen::AngleAxisd(q, axis).toRotationMatrix());

      data.v.w = v;
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.block<6,3> (0,Inertia::ANGULAR) * axis;
      data.Dinv[0] = 1./axis.dot(data.U.segment <3> (Inertia::ANGULAR));
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
      const Scalar & q = qs[idx_q()];
      const Scalar & v = vs[idx_v()];

      ConfigVector_t result;
      result << (q + v);
      return result; 
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    { 
      const Scalar & q_0 = q0[idx_q()];
      const Scalar & q_1 = q1[idx_q()];

      ConfigVector_t result;
      result << ((1-u) * q_0 + u * q_1);
      return result; 
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
          assert(false && "non bounded limit. Cannot uniformly sample joint reolute unaligned");
          throw std::runtime_error(error.str());
        }
        result[i] = lower_pos_limit[i] + ( upper_pos_limit[i] - lower_pos_limit[i]) * rand()/RAND_MAX;
      }
      return result;
    } 

    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      const Scalar & q_0 = q0[idx_q()];
      const Scalar & q_1 = q1[idx_q()];

      TangentVector_t result;
      result << (q_1 - q_0);
      return result; 
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return fabs(difference_impl(q0,q1)[0]);
    }

    ConfigVector_t neutralConfiguration_impl() const
    { 
      ConfigVector_t q;
      q << 0;
      return q;
    } 

    bool isSameConfiguration_impl(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      const Scalar & q_1 = q1[idx_q()];
      const Scalar & q_2 = q2[idx_q()];

      return (fabs(q_1 - q_2) < prec);
    }

    static std::string classname() { return std::string("JointModelRevoluteUnaligned"); }
    std::string shortname() const { return classname(); }

    Motion::Vector3 axis;
  }; // struct JointModelRevoluteUnaligned

} //namespace se3


#endif // ifndef __se3_joint_revolute_unaligned_hpp__
