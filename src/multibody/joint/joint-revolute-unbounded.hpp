//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_joint_revolute_unbounded_hpp__
#define __se3_joint_revolute_unbounded_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

#include <stdexcept>

namespace se3
{

  template<int axis> struct JointRevoluteUnbounded;

  template<int axis> struct JointDataRevoluteUnbounded;
  template<int axis> struct JointModelRevoluteUnbounded;


  template<int axis>
  struct traits< JointRevoluteUnbounded<axis> >
  {
    enum {
      NQ = 2,
      NV = 1
    };
    typedef double Scalar;
    typedef JointDataRevoluteUnbounded<axis> JointDataDerived;
    typedef JointModelRevoluteUnbounded<axis> JointModelDerived;
    typedef ConstraintRevolute<axis> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionRevolute<axis> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };

  template<int axis> struct traits< JointDataRevoluteUnbounded<axis> > { typedef JointRevoluteUnbounded<axis> JointDerived; };
  template<int axis> struct traits< JointModelRevoluteUnbounded<axis> > { typedef JointRevoluteUnbounded<axis> JointDerived; };

  template<int axis>
  struct JointDataRevoluteUnbounded : public JointDataBase< JointDataRevoluteUnbounded<axis> >
  {
    typedef JointRevoluteUnbounded<axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;
    F_t F;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataRevoluteUnbounded() : M(1), U(), Dinv(), UDinv()
    {}

  }; // struct JointDataRevoluteUnbounded

  template<int axis>
  struct JointModelRevoluteUnbounded : public JointModelBase< JointModelRevoluteUnbounded<axis> >
  {
    typedef JointRevoluteUnbounded<axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelRevoluteUnbounded>::id;
    using JointModelBase<JointModelRevoluteUnbounded>::idx_q;
    using JointModelBase<JointModelRevoluteUnbounded>::idx_v;
    using JointModelBase<JointModelRevoluteUnbounded>::setIndexes;
    typedef Motion::Vector3 Vector3;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    void calc( JointDataDerived& data, 
     const Eigen::VectorXd & qs ) const
    {
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());

      const double & ca = q(0);
      const double & sa = q(1);

      data.M.rotation(JointRevolute<axis>::cartesianRotation(ca,sa));
    }

    void calc( JointDataDerived& data, 
     const Eigen::VectorXd & qs, 
     const Eigen::VectorXd & vs ) const
    {
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());

      const double & ca = q(0);
      const double & sa = q(1);
      const double & v = q_dot(0);

      data.M.rotation(JointRevolute<axis>::cartesianRotation(ca,sa));
      data.v.w = v;
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] = 1./I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis);
      data.UDinv = data.U * data.Dinv[0];
      
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
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());

      const double & ca = q(0);
      const double & sa = q(1);
      const double & omega = q_dot(0);

      double cosOmega,sinOmega; SINCOS(omega, &sinOmega, &cosOmega);
      // TODO check the cost of atan2 vs SINCOS

      ConfigVector_t result (cosOmega * ca - sinOmega * sa,
                             sinOmega * ca + cosOmega * sa);
      const double norm2 = q.squaredNorm();
      result *= (3 - norm2) / 2;

      return result;
    }


    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    { 
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qi = q0.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qf = q1.segment<NQ> (idx_q ());

      const double & c0 = qi(0), s0 = qi(1);
      const double & c1 = qf(0), s1 = qf(1);

      assert ( (qi.norm() - 1) < 1e-8 && "initial configuration not normalized");
      assert ( (qf.norm() - 1) < 1e-8 && "final configuration not normalized");
      double cosTheta = c0*c1 + s0*s1;
      double sinTheta = c0*s1 - s0*c1;
      double theta = atan2(sinTheta, cosTheta);
      assert (fabs (sin (theta) - sinTheta) < 1e-8);

      ConfigVector_t result;
      if (fabs (theta) > 1e-6 && fabs (theta) < PI - 1e-6)
      {
        result = (sin ((1-u)*theta)/sinTheta) * qi 
                  + (sin (u*theta)/sinTheta) * qf;
      } 
      else if (fabs (theta) < 1e-6) // theta = 0
      {
        result = (1-u) * qi + u * qf;
      }
      else // theta = +-PI
      {
        double theta0 = atan2 (s0, c0);
        result << cos (theta0 + u * theta),
                  sin (theta0 + u * theta);
      }
      
      return result; 
    }

    ConfigVector_t random_impl() const
    { 
      ConfigVector_t result(ConfigVector_t::Random());
      result.normalize();
      return result;
    } 


    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & , const ConfigVector_t &  ) const
    { 
      ConfigVector_t result;
      double angle = -PI + 2*PI * rand()/RAND_MAX;
      double ca,sa; SINCOS (angle, &sa, &ca);

      result << ca, sa;
      return result;
    } 


    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qi = q0.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qf = q1.segment<NQ> (idx_q ());

      const double & c0 = qi(0), s0 = qi(1);
      const double & c1 = qf(0), s1 = qf(1);

      TangentVector_t result;
      result << atan2 (s1*c0 - s0*c1, c0*c1 + s0*s1);
      return result; 
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return difference_impl(q0,q1).norm();
    }

    ConfigVector_t neutralConfiguration_impl() const
    { 
      ConfigVector_t q;
      q << 1,0;
      return q;
    } 

    void normalize_impl(Eigen::VectorXd& q) const
    {
      q.segment<NQ>(idx_q()).normalize();
    }

    bool isSameConfiguration_impl(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_2 = q2.segment<NQ> (idx_q ());

      return q_1.isApprox(q_2, prec);
    } 

    static std::string classname();
    std::string shortname() const { return classname(); }

  }; // struct JointModelRevoluteUnbounded

  typedef JointRevoluteUnbounded<0> JointRUBX;
  typedef JointDataRevoluteUnbounded<0> JointDataRUBX;
  typedef JointModelRevoluteUnbounded<0> JointModelRUBX;

  template<> inline
  std::string JointModelRevoluteUnbounded<0>::classname()
  {
    return std::string("JointModelRUBX") ;
  }

  typedef JointRevoluteUnbounded<1> JointRUBY;
  typedef JointDataRevoluteUnbounded<1> JointDataRUBY;
  typedef JointModelRevoluteUnbounded<1> JointModelRUBY;

  template<> inline
  std::string JointModelRevoluteUnbounded<1>::classname()
  {
    return std::string("JointModelRUBY") ;
  }

  typedef JointRevoluteUnbounded<2> JointRUBZ;
  typedef JointDataRevoluteUnbounded<2> JointDataRUBZ;
  typedef JointModelRevoluteUnbounded<2> JointModelRUBZ;

  template<> inline
  std::string JointModelRevoluteUnbounded<2>::classname()
  {
    return std::string("JointModelRUBZ") ;
  }

} //namespace se3

#endif // ifndef __se3_joint_revolute_unbounded_hpp__
