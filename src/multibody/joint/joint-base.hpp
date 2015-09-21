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

#ifndef __se3_joint_base_hpp__
#define __se3_joint_base_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/variant.hpp>
#include <limits>

namespace se3
{
  // template<class C> struct traits {};

  /* RNEA operations
   *
   * *** FORWARD ***
   * J::calc(q,vq)
   * SE3    = SE3 * J::SE3
   * Motion = J::Motion
   * Motion = J::Constraint*J::JointMotion + J::Bias + Motion^J::Motion
   * Force  = Inertia*Motion  + Inertia.vxiv(Motion)
   *
   * *** BACKWARD *** 
   * J::JointForce = J::Constraint::Transpose*J::Force
   */

  /* CRBA operations
   *
   * *** FORWARD ***
   * J::calc(q)
   * Inertia = Inertia
   *
   * *** BACKWARD *** 
   * Inertia += SE3::act(Inertia)
   * F = Inertia*J::Constraint
   * JointInertia.block = J::Constraint::Transpose*F
   * *** *** INNER ***
   *     F = SE3::act(f)
   *     JointInertia::block = J::Constraint::Transpose*F
   */

  /* Jacobian operations
   *
   * internal::ActionReturn<Constraint>::Type
   * Constraint::se3Action
   */
#ifdef __clang__

#define SE3_JOINT_TYPEDEF_ARG(prefix)              \
   typedef int Index;                \
   typedef prefix traits<Joint>::JointData JointData;        \
   typedef prefix traits<Joint>::JointModel JointModel;      \
   typedef prefix traits<Joint>::Constraint_t Constraint_t;      \
   typedef prefix traits<Joint>::Transformation_t Transformation_t; \
   typedef prefix traits<Joint>::Motion_t Motion_t;        \
   typedef prefix traits<Joint>::Bias_t Bias_t;        \
   typedef prefix traits<Joint>::F_t F_t;          \
   enum {                  \
    NQ = traits<Joint>::NQ,              \
    NV = traits<Joint>::NV               \
  }

#define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_ARG()
#define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_ARG(typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

#define SE3_JOINT_TYPEDEF_NOARG()       \
  typedef int Index;            \
  typedef traits<Joint>::JointData JointData;     \
  typedef traits<Joint>::JointModel JointModel;     \
  typedef traits<Joint>::Constraint_t Constraint_t;   \
  typedef traits<Joint>::Transformation_t Transformation_t; \
  typedef traits<Joint>::Motion_t Motion_t;     \
  typedef traits<Joint>::Bias_t Bias_t;       \
  typedef traits<Joint>::F_t F_t;       \
  enum {              \
    NQ = traits<Joint>::NQ,         \
    NV = traits<Joint>::NV          \
  }

#define SE3_JOINT_TYPEDEF_ARG(prefix)         \
  typedef int Index;              \
  typedef prefix traits<Joint>::JointData JointData;      \
  typedef prefix traits<Joint>::JointModel JointModel;      \
  typedef prefix traits<Joint>::Constraint_t Constraint_t;    \
  typedef prefix traits<Joint>::Transformation_t Transformation_t;  \
  typedef prefix traits<Joint>::Motion_t Motion_t;      \
  typedef prefix traits<Joint>::Bias_t Bias_t;        \
  typedef prefix traits<Joint>::F_t F_t;        \
  enum {                \
    NQ = traits<Joint>::NQ,           \
    NV = traits<Joint>::NV            \
  }

#define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_NOARG()
#define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_ARG(typename)

#else

#define SE3_JOINT_TYPEDEF_ARG()              \
  typedef int Index;                 \
  typedef typename traits<Joint>::JointData JointData;         \
  typedef typename traits<Joint>::JointModel JointModel;       \
  typedef typename traits<Joint>::Constraint_t Constraint_t;       \
  typedef typename traits<Joint>::Transformation_t Transformation_t; \
  typedef typename traits<Joint>::Motion_t Motion_t;         \
  typedef typename traits<Joint>::Bias_t Bias_t;         \
  typedef typename traits<Joint>::F_t F_t;           \
  enum {                   \
    NQ = traits<Joint>::NQ,              \
    NV = traits<Joint>::NV               \
  }

#define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_ARG()
#define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_ARG()

#endif

#define SE3_JOINT_USE_INDEXES \
  typedef JointModelBase<JointModel> Base; \
  using Base::idx_q; \
  using Base::idx_v


  template <int _NQ, int _NV> struct JointDataDense;
  template <int _NQ, int _NV> struct JointModelDense;
  template <int _NQ, int _NV> struct JointDense;

  template<typename _JointData>
  struct JointDataBase
  {
    typedef typename traits<_JointData>::Joint Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;
public:
    JointData& derived() { return *static_cast<JointData*>(this); }
    const JointData& derived() const { return *static_cast<const JointData*>(this); }

    const Constraint_t     & S() const  { return static_cast<const JointData*>(this)->S;   }
    const Transformation_t & M() const  { return static_cast<const JointData*>(this)->M;   }
    const Motion_t         & v() const  { return static_cast<const JointData*>(this)->v;   }
    const Bias_t           & c() const  { return static_cast<const JointData*>(this)->c;   }
    F_t& F()        { return static_cast<      JointData*>(this)->F; }

    JointDataDense<NQ, NV> toDense() const  { return static_cast<const JointData*>(this)->toDense_impl();   }

  };

  template<typename _JointModel>
  struct JointModelBase
  {
    typedef typename traits<_JointModel>::Joint Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    JointModel& derived() { return *static_cast<JointModel*>(this); }
    const JointModel& derived() const { return *static_cast<const JointModel*>(this); }

    JointData createData() const { return static_cast<const JointModel*>(this)->createData(); }
    void calc( JointData& data, 
      const Eigen::VectorXd & qs ) const
    { return static_cast<const JointModel*>(this)->calc(data,qs); }
    void calc( JointData& data, 
      const Eigen::VectorXd & qs, 
      const Eigen::VectorXd & vs ) const
    { return static_cast<const JointModel*>(this)->calc(data,qs,vs); }

  public:
    Index i_id; // ID of the joint in the multibody list.
    int i_q;    // Index of the joint configuration in the joint configuration vector.
    int i_v;    // Index of the joint velocity in the joint velocity vector.

    Eigen::Matrix<double,NQ,1> position_lower;
    Eigen::Matrix<double,NQ,1> position_upper;

    Eigen::Matrix<double,NQ,1> effortMax;
    Eigen::Matrix<double,NV,1> velocityMax;

  public:
    
    int     nv()    const { return NV; }
    int     nq()    const { return NQ; }
    const int &   idx_q() const { return i_q; }
    const int &   idx_v() const { return i_v; }
    const Index & id()    const { return i_id; }

    const Eigen::Matrix<double,NQ,1> & lowerPosLimit() const { return position_lower;}
    const Eigen::Matrix<double,NQ,1> & upperPosLimit() const { return position_upper;}

    const Eigen::Matrix<double,NQ,1> & maxEffortLimit() const { return effortMax;}
    const Eigen::Matrix<double,NV,1> & maxVelocityLimit() const { return velocityMax;}


    void setIndexes(Index id,int q,int v) { i_id = id, i_q = q; i_v = v; }

    void setLowerPositionLimit(const Eigen::VectorXd & lowerPos)
    {
      if (lowerPos.rows() == NQ)
        position_lower = lowerPos;
      else
        position_lower.fill(-std::numeric_limits<double>::infinity());
    }

    void setUpperPositionLimit(const Eigen::VectorXd & upperPos)
    {
      if (upperPos.rows() == NQ)
        position_upper = upperPos;
      else
        position_upper.fill(std::numeric_limits<double>::infinity());
    }

    void setMaxEffortLimit(const Eigen::VectorXd & effort)
    {
      if (effort.rows() == NV)
        effortMax = effort;
      else
        effortMax.fill(std::numeric_limits<double>::infinity());
    }

    void setMaxVelocityLimit(const Eigen::VectorXd & v)
    {
      if (v.rows() == NV)
        velocityMax = v;
      else
        velocityMax.fill(std::numeric_limits<double>::infinity());
    }


    template<typename D>
    typename D::template ConstFixedSegmentReturnType<NV>::Type jointMotion(const Eigen::MatrixBase<D>& a) const     { return a.template segment<NV>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<NV>::Type jointMotion(Eigen::MatrixBase<D>& a) const 
    { return a.template segment<NV>(i_v); }
    template<typename D>
    typename D::template ConstFixedSegmentReturnType<NV>::Type jointForce(const Eigen::MatrixBase<D>& tau) const 
    { return tau.template segment<NV>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<NV>::Type jointForce(Eigen::MatrixBase<D>& tau) const 
    { return tau.template segment<NV>(i_v); }

    template<typename D>
    typename D::template ConstFixedSegmentReturnType<NQ>::Type jointLimit(const Eigen::MatrixBase<D>& limit) const 
    { return limit.template segment<NQ>(i_q); }
    template<typename D>
    typename D::template FixedSegmentReturnType<NQ>::Type jointLimit(Eigen::MatrixBase<D>& limit) const 
    { return limit.template segment<NQ>(i_q); }

    template<typename D>
    typename D::template ConstFixedSegmentReturnType<NV>::Type jointTangentLimit(const Eigen::MatrixBase<D>& limit) const 
    { return limit.template segment<NV>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<NV>::Type jointTangentLimit(Eigen::MatrixBase<D>& limit) const 
    { return limit.template segment<NV>(i_v); }


    JointModelDense<NQ, NV> toDense() const  { return static_cast<const JointModel*>(this)->toDense_impl();   }
  };

  

  template<int _NQ, int _NV>
  struct traits< JointDense<_NQ, _NV > >
  {
    typedef JointDataDense<_NQ, _NV> JointData;
    typedef JointModelDense<_NQ, _NV> JointModel;
    typedef ConstraintXd Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> F_t;
    enum {
      NQ = _NQ, // pb 
      NV = _NV
    };
  };

  template<int _NQ, int _NV> struct traits< JointDataDense<_NQ, _NV > > { typedef JointDense<_NQ,_NV > Joint; };
  template<int _NQ, int _NV> struct traits< JointModelDense<_NQ, _NV > > { typedef JointDense<_NQ,_NV > Joint; };

  template <int _NQ, int _NV>
  struct JointDataDense : public JointDataBase< JointDataDense<_NQ, _NV > >
  {
    typedef JointDense<_NQ, _NV > Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;
public:
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;

    /// Removed Default constructor of JointDataDense because it was calling default constructor of
    /// ConstraintXd -> eigen_static_assert
    /// JointDataDense should always be instanciated from a JointDataXX.toDense()
    // JointDataDense() : M(1)
    // {
    //   M.translation(SE3::Vector3::Zero());
    // }

    JointDataDense( Constraint_t S,
                    Transformation_t M,
                    Motion_t v,
                    Bias_t c,
                    F_t F
                    )
    : S(S)
    , M(M)
    , v(v)
    , c(c)
    , F(F)
    {}

    JointDataDense<_NQ, _NV> toDense_impl() const
    {
      assert(false && "Trying to convert a jointDataDense to JointDataDense : useless"); // disapear with release optimizations
      return *this;
    }

  }; // struct JointDataDense

  template <int _NQ, int _NV>
  struct JointModelDense : public JointModelBase< JointModelDense<_NQ, _NV > >
  {
    typedef JointDense<_NQ, _NV > Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelDense<_NQ, _NV > >::idx_q;
    using JointModelBase<JointModelDense<_NQ, _NV > >::idx_v;
    using JointModelBase<JointModelDense<_NQ, _NV > >::setLowerPositionLimit;
    using JointModelBase<JointModelDense<_NQ, _NV > >::setUpperPositionLimit;
    using JointModelBase<JointModelDense<_NQ, _NV > >::setMaxEffortLimit;
    using JointModelBase<JointModelDense<_NQ, _NV > >::setMaxVelocityLimit;
    using JointModelBase<JointModelDense<_NQ, _NV > >::setIndexes;
    
    JointData createData() const
    {
      assert(false && "JointModelDense is read-only, should not createData");
      return JointData();
    }
    void calc( JointData& , 
     const Eigen::VectorXd &  ) const
    {
      assert(false && "JointModelDense is read-only, should not perform any calc");
    }

    void calc( JointData&  ,
     const Eigen::VectorXd & , 
     const Eigen::VectorXd &  ) const
    {
      assert(false && "JointModelDense is read-only, should not perform any calc");
    }

    JointModelDense()
    {
      setIndexes(1, 1, 1);
      setLowerPositionLimit(Eigen::Matrix<double,NQ,1>(1));
      setUpperPositionLimit(Eigen::Matrix<double,NQ,1>(1));
      setMaxEffortLimit(Eigen::Matrix<double,NQ,1>(1));
      setMaxVelocityLimit(Eigen::Matrix<double,NV,1>(1));
    }
    JointModelDense(  Index idx,
                      int idx_q,
                      int idx_v,
                      Eigen::Matrix<double,NQ,1> lowPos,
                      Eigen::Matrix<double,NQ,1> upPos,
                      Eigen::Matrix<double,NQ,1> maxEff,
                      Eigen::Matrix<double,NV,1> maxVel
                    )
    {
      setIndexes(idx, idx_q, idx_v);
      setLowerPositionLimit(lowPos);
      setUpperPositionLimit(upPos);
      setMaxEffortLimit(maxEff);
      setMaxVelocityLimit(maxVel);
    }

    JointModelDense<_NQ, _NV> toDense_impl() const
    {
      assert(false && "Trying to convert a jointModelDense to JointModelDense : useless"); // disapear with release optimizations
      return *this;
    }

  }; // struct JointModelDense

} // namespace se3

#endif // ifndef __se3_joint_base_hpp__
