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

#ifndef __se3_joint_revolute_unaligned_hpp__
#define __se3_joint_revolute_unaligned_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-dense.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/math/sincos.hpp"

namespace se3
{

  struct JointDataRevoluteUnaligned;
  struct JointModelRevoluteUnaligned;

  struct MotionRevoluteUnaligned;
  template <>
  struct traits < MotionRevoluteUnaligned >
  {
    typedef double Scalar_t;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
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

    Motion::Vector3 axis; 
    double w;

    operator Motion() const
    { 
      return Motion(Motion::Vector3::Zero(),
                    axis*w);
    }
  }; // struct MotionRevoluteUnaligned
  
  const MotionRevoluteUnaligned& operator+ (const MotionRevoluteUnaligned& m, const BiasZero&)
  { return m; }

  Motion operator+ (const MotionRevoluteUnaligned& m1, const Motion& m2)
  {
    return Motion( m2.linear(), m1.w*m1.axis+m2.angular() );
  }

  struct ConstraintRevoluteUnaligned;
  template <>
  struct traits < ConstraintRevoluteUnaligned >
  {
    typedef double Scalar_t;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
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
    typedef Eigen::Matrix<Scalar_t,1,1,0> JointMotion;
    typedef Eigen::Matrix<Scalar_t,1,1,0> JointForce;
    typedef Eigen::Matrix<Scalar_t,6,1> DenseBase;
  }; // traits ConstraintRevoluteUnaligned


    

    struct ConstraintRevoluteUnaligned : ConstraintBase < ConstraintRevoluteUnaligned >
    {
      SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintRevoluteUnaligned);
      enum { NV = 1, Options = 0 };
      typedef traits<ConstraintRevoluteUnaligned>::JointMotion JointMotion;
      typedef traits<ConstraintRevoluteUnaligned>::JointForce JointForce;
      typedef traits<ConstraintRevoluteUnaligned>::DenseBase DenseBase;

      ConstraintRevoluteUnaligned() : axis(Eigen::Vector3d::Constant(NAN)) {}
      ConstraintRevoluteUnaligned(const Motion::Vector3 & _axis) : axis(_axis) {}
      Motion::Vector3 axis; 

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

      	const Eigen::Matrix<double, 1, 1>
      	operator*( const Force& f ) const
      	{
      	  return ref.axis.transpose()*f.angular();
      	}

        /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
        template<typename D>
        friend
        typename Eigen::ProductReturnType<
        Eigen::Transpose<const Eigen::Matrix<typename Eigen::MatrixBase<D>::Scalar, 3, 1> >,
        Eigen::Block<const Eigen::Block<Eigen::Matrix<typename Eigen::MatrixBase<D>::Scalar,6,-1>,-1,-1>, 3, -1>
        >::Type
        operator* (const TransposeConst & tc, const Eigen::MatrixBase<D> & F)
        {
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
      	Eigen::Matrix<double,6,1> S;
      	S << Eigen::Vector3d::Zero(), axis;
      	return ConstraintXd(S);
      }
      
    }; // struct ConstraintRevoluteUnaligned


    Motion operator^( const Motion& m1, const MotionRevoluteUnaligned & m2)
    {
      /* m1xm2 = [ v1xw2 + w1xv2; w1xw2 ] = [ v1xw2; w1xw2 ] */
      const Motion::Vector3& v1 = m1.linear();
      const Motion::Vector3& w1 = m1.angular();
      const Motion::Vector3& w2 = m2.axis * m2.w ;
      return Motion( v1.cross(w2),w1.cross(w2));
    }

    /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
    Eigen::Matrix<double,6,1>
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
      typedef JointDataRevoluteUnaligned JointData;
      typedef JointModelRevoluteUnaligned JointModel;
      typedef ConstraintRevoluteUnaligned Constraint_t;
      typedef SE3 Transformation_t;
      typedef MotionRevoluteUnaligned Motion_t;
      typedef BiasZero Bias_t;
      typedef Eigen::Matrix<double,6,1> F_t;
      enum {
        NQ = 1,
        NV = 1
      };
    };

  template<> struct traits<JointDataRevoluteUnaligned> { typedef JointRevoluteUnaligned Joint; };
  template<> struct traits<JointModelRevoluteUnaligned> { typedef JointRevoluteUnaligned Joint; };

  struct JointDataRevoluteUnaligned : public JointDataBase< JointDataRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned Joint;
    SE3_JOINT_TYPEDEF;

    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    F_t F;
    Eigen::AngleAxisd angleaxis;

    JointDataRevoluteUnaligned() 
      : M(1),S(Eigen::Vector3d::Constant(NAN)),v(Eigen::Vector3d::Constant(NAN),NAN)
      , angleaxis( NAN,Eigen::Vector3d::Constant(NAN))
    {}
    JointDataRevoluteUnaligned(const Motion::Vector3 & axis) 
      : M(1),S(axis),v(axis,NAN)
      ,angleaxis(NAN,axis)
    {}

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F);
    }
  }; // struct JointDataRevoluteUnaligned

  struct JointModelRevoluteUnaligned : public JointModelBase< JointModelRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRevoluteUnaligned>::id;
    using JointModelBase<JointModelRevoluteUnaligned>::idx_q;
    using JointModelBase<JointModelRevoluteUnaligned>::idx_v;
    using JointModelBase<JointModelRevoluteUnaligned>::lowerPosLimit;
    using JointModelBase<JointModelRevoluteUnaligned>::upperPosLimit;
    using JointModelBase<JointModelRevoluteUnaligned>::maxEffortLimit;
    using JointModelBase<JointModelRevoluteUnaligned>::maxVelocityLimit;
    using JointModelBase<JointModelRevoluteUnaligned>::setIndexes;
    
    JointModelRevoluteUnaligned() : axis(Eigen::Vector3d::Constant(NAN))   {}
    JointModelRevoluteUnaligned(double x, double y, double z)
    {
      axis << x, y, z ;
      axis.normalize();
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }
    JointModelRevoluteUnaligned( const Motion::Vector3 & axis ) : axis(axis)
    {
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }

    JointData createData() const { return JointData(axis); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];

      /* It should not be necessary to copy axis in jdata, however a current bug
       * in the fusion visitor prevents a proper access to jmodel::axis. A
       * by-pass is to access to a copy of it in jdata. */
      data.angleaxis.angle() = q;
      data.M.rotation(data.angleaxis.toRotationMatrix());
    }

    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      /* It should not be necessary to copy axis in jdata, however a current bug
       * in the fusion visitor prevents a proper access to jmodel::axis. A
       * by-pass is to access to a copy of it in jdata. */
      data.angleaxis.angle() = q;
      data.M.rotation(data.angleaxis.toRotationMatrix());
      data.v.w = v;
    }

    Eigen::Vector3d axis;

    JointModelDense<NQ, NV> toDense_impl() const
    {
      return JointModelDense<NQ, NV>( id(),
                                      idx_q(),
                                      idx_v(),
                                      lowerPosLimit(),
                                      upperPosLimit(),
                                      maxEffortLimit(),
                                      maxVelocityLimit()
                                    );
    }

    static const std::string shortname()
    {
      return std::string("JointModelRevoluteUnaligned");
    }

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModelRevoluteUnaligned> & jmodel) const
    {
      return jmodel.id() == id()
              && jmodel.idx_q() == idx_q()
              && jmodel.idx_v() == idx_v()
              && jmodel.lowerPosLimit() == lowerPosLimit()
              && jmodel.upperPosLimit() == upperPosLimit()
              && jmodel.maxEffortLimit() == maxEffortLimit()
              && jmodel.maxVelocityLimit() == maxVelocityLimit();
    }
  }; // struct JointModelRevoluteUnaligned

} //namespace se3


#endif // ifndef __se3_joint_revolute_unaligned_hpp__
