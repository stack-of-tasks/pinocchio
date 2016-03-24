//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include <Eigen/Geometry>
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
   typedef prefix traits<Joint>::U_t U_t;       \
   typedef prefix traits<Joint>::D_t D_t;       \
   typedef prefix traits<Joint>::UD_t UD_t;       \
   enum {                  \
    NQ = traits<Joint>::NQ,              \
    NV = traits<Joint>::NV               \
  };                        \
  typedef prefix traits<Joint>::ConfigVector_t ConfigVector_t;        \
  typedef prefix traits<Joint>::TangentVector_t TangentVector_t

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
  typedef traits<Joint>::U_t U_t;       \
  typedef traits<Joint>::D_t D_t;       \
  typedef traits<Joint>::UD_t UD_t;       \
  enum {              \
    NQ = traits<Joint>::NQ,         \
    NV = traits<Joint>::NV          \
  };                        \
  typedef traits<Joint>::ConfigVector_t ConfigVector_t;        \
  typedef traits<Joint>::TangentVector_t TangentVector_t

#define SE3_JOINT_TYPEDEF_ARG(prefix)         \
  typedef int Index;              \
  typedef prefix traits<Joint>::JointData JointData;      \
  typedef prefix traits<Joint>::JointModel JointModel;      \
  typedef prefix traits<Joint>::Constraint_t Constraint_t;    \
  typedef prefix traits<Joint>::Transformation_t Transformation_t;  \
  typedef prefix traits<Joint>::Motion_t Motion_t;      \
  typedef prefix traits<Joint>::Bias_t Bias_t;        \
  typedef prefix traits<Joint>::F_t F_t;        \
  typedef prefix traits<Joint>::U_t U_t;       \
  typedef prefix traits<Joint>::D_t D_t;       \
  typedef prefix traits<Joint>::UD_t UD_t;       \
  enum {                \
    NQ = traits<Joint>::NQ,           \
    NV = traits<Joint>::NV            \
  };                        \
  typedef prefix traits<Joint>::ConfigVector_t ConfigVector_t;        \
  typedef prefix traits<Joint>::TangentVector_t TangentVector_t

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
  typedef typename traits<Joint>::U_t U_t;       \
  typedef typename traits<Joint>::D_t D_t;       \
  typedef typename traits<Joint>::UD_t UD_t;       \
  enum {                   \
    NQ = traits<Joint>::NQ,              \
    NV = traits<Joint>::NV               \
  };                        \
  typedef typename traits<Joint>::ConfigVector_t ConfigVector_t;        \
  typedef typename traits<Joint>::TangentVector_t TangentVector_t

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

    JointData& derived() { return *static_cast<JointData*>(this); }
    const JointData& derived() const { return *static_cast<const JointData*>(this); }

    const Constraint_t     & S() const  { return static_cast<const JointData*>(this)->S;   }
    const Transformation_t & M() const  { return static_cast<const JointData*>(this)->M;   }
    const Motion_t         & v() const  { return static_cast<const JointData*>(this)->v;   }
    const Bias_t           & c() const  { return static_cast<const JointData*>(this)->c;   }
    F_t & F()        { return static_cast<      JointData*>(this)->F; }
    
    // [ABA CCRBA]
    const U_t & U() const { return static_cast<const JointData*>(this)->U; }
    U_t & U() { return static_cast<JointData*>(this)->U; }
    const D_t & Dinv() const { return static_cast<const JointData*>(this)->Dinv; }
    const UD_t & UDinv() const { return static_cast<const JointData*>(this)->UDinv; }

    JointDataDense<NQ, NV> toDense() const  { return static_cast<const JointData*>(this)->toDense_impl();   }

  }; // struct JointDataBase

  template<int NV>
  struct SizeDepType
  {
    template<class Mat>
    struct SegmentReturn 
    {
      typedef typename Mat::template FixedSegmentReturnType<NV>::Type Type;
      typedef typename Mat::template ConstFixedSegmentReturnType<NV>::Type ConstType;
    };
    template<class Mat>
    struct ColsReturn
    {
      typedef typename Mat::template NColsBlockXpr<NV>::Type Type;
      typedef typename Mat::template ConstNColsBlockXpr<NV>::Type ConstType;
    };
  };
  template<>
  struct SizeDepType<Eigen::Dynamic>
  {
    template<class Mat>
    struct SegmentReturn 
    {
      typedef typename Mat::SegmentReturnType Type;
      typedef typename Mat::ConstSegmentReturnType ConstType;
    };
    template<class Mat>
    struct ColsReturn
    {
      typedef typename Mat::ColsBlockXpr Type;
      typedef typename Mat::ConstColsBlockXpr ConstType;
    };
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
    
    void calc_aba(JointData & data,
                  Inertia::Matrix6 & I,
                  const bool update_I = false) const
    { return static_cast<const JointModel*>(this)->calc_aba(data, I, update_I); }


    /**
     * @brief      Integrate joint's configuration for a constant derivative during unit time
     *
     * @param[in]  q     initatial configuration  (size jmodel.nq)
     * @param[in]  v     joint velocity (size jmodel.nv)
     *
     * @return     The configuration integrated
     */
    ConfigVector_t integrate(const Eigen::VectorXd & q,const Eigen::VectorXd & v) const
    { return derived().integrate_impl(q, v); } 


    /**
     * @brief      Interpolation between two joint's configurations
     *
     * @param[in]  q1    Initial configuration to interpolate
     * @param[in]  q2    Final configuration to interpolate
     * @param[in]  u     u in [0;1] position along the interpolation.
     *
     * @return     The interpolated configuration (q1 if u = 0, q2 if u = 1)
     */
    ConfigVector_t interpolate(const Eigen::VectorXd & q1,const Eigen::VectorXd & q2, double u) const
    { return derived().interpolate_impl(q1, q2, u); }


    /**
     * @brief      Generate a random joint configuration. Take into account quaternions
     *
     * \warning Do not take yet into account the joint limits
     *
     * @return     The joint configuration
     */
    ConfigVector_t random() const
    { return derived().random_impl(); } 


    
    /**
     * @brief      the constant derivative that must be integrated during unit time to go from q2 to q1
     *
     * @param[in]  q1    Wished configuration
     * @param[in]  q2    Initial configuration
     *
     * @return     The corresponding velocity
     */
    TangentVector_t difference(const Eigen::VectorXd & q1,const Eigen::VectorXd & q2) const
    { return derived().difference_impl(q1, q2); } 

    /**
     * @brief      Distance between two configurations of the joint
     *
     * @param[in]  q1    Configuration 1
     * @param[in]  q2    Configuration 2
     *
     * @return     The corresponding distance
     */
    double distance(const Eigen::VectorXd & q1,const Eigen::VectorXd & q2) const
    { return derived().distance_impl(q1, q2); } 

    Index i_id; // ID of the joint in the multibody list.
    int i_q;    // Index of the joint configuration in the joint configuration vector.
    int i_v;    // Index of the joint velocity in the joint velocity vector.

    ConfigVector_t position_lower;
    ConfigVector_t position_upper;

    TangentVector_t effortMax;
    TangentVector_t velocityMax;
    
    int     nv()    const { return derived().nv_impl(); }
    int     nq()    const { return derived().nq_impl(); }
    // Both _impl methods are reimplemented by dynamic-size joints.
    int     nv_impl() const { return NV; }
    int     nq_impl() const { return NQ; }

    const int &   idx_q() const { return i_q; }
    const int &   idx_v() const { return i_v; }
    const Index & id()    const { return i_id; }

    const ConfigVector_t & lowerPosLimit() const { return position_lower;}
    const ConfigVector_t & upperPosLimit() const { return position_upper;}

    const TangentVector_t & maxEffortLimit() const { return effortMax;}
    const TangentVector_t & maxVelocityLimit() const { return velocityMax;}

    void setIndexes(Index id,int q,int v) { i_id = id, i_q = q; i_v = v; }

    void setLowerPositionLimit(const ConfigVector_t & lowerPos)
    {
      position_lower = lowerPos;
    }

    void setUpperPositionLimit(const ConfigVector_t & upperPos)
    {
      position_upper = upperPos;
    }

    void setMaxEffortLimit(const TangentVector_t & effort)
    {
      effortMax = effort;
    }

    void setMaxVelocityLimit(const TangentVector_t & v)
    {
      velocityMax = v;
    }

    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType 
    jointConfigSelector(const Eigen::MatrixBase<D>& a) const       { return derived().jointConfigSelector_impl(a); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType 
    jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const   { return a.template segment<NQ>(i_q); }
    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type 
    jointConfigSelector( Eigen::MatrixBase<D>& a) const { return derived().jointConfigSelector_impl(a); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type 
    jointConfigSelector_impl( Eigen::MatrixBase<D>& a) const { return a.template segment<NQ>(i_q); }

    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType 
    jointVelocitySelector(const Eigen::MatrixBase<D>& a) const       { return derived().jointVelocitySelector_impl(a); }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType 
    jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const   { return a.template segment<NV>(i_v); }
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type 
    jointVelocitySelector( Eigen::MatrixBase<D>& a) const { return derived().jointVelocitySelector_impl(a); }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type 
    jointVelocitySelector_impl( Eigen::MatrixBase<D>& a) const { return a.template segment<NV>(i_v); }


    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols(const Eigen::MatrixBase<D>& A) const       { return derived().jointCols_impl(A); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols_impl(const Eigen::MatrixBase<D>& A) const       { return A.template middleCols<NV>(i_v); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols(Eigen::MatrixBase<D>& A) const       { return derived().jointCols_impl(A); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols_impl(Eigen::MatrixBase<D>& A) const       { return A.template middleCols<NV>(i_v); }

    JointModelDense<NQ, NV> toDense() const  { return static_cast<const JointModel*>(this)->toDense_impl();   }
  }; // struct JointModelBase

} // namespace se3

#endif // ifndef __se3_joint_base_hpp__
