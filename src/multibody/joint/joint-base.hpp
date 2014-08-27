#ifndef __se3_joint_base_hpp__
#define __se3_joint_base_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/variant.hpp>

namespace se3
{
  template<class C> struct traits {};

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

#define SE3_JOINT_TYPEDEF \
  typedef typename traits<Joint>::JointData JointData; \
  typedef typename traits<Joint>::JointModel JointModel; \
  typedef typename traits<Joint>::Constraint_t Constraint_t; \
  typedef typename traits<Joint>::Transformation_t Transformation_t; \
  typedef typename traits<Joint>::Motion_t Motion_t; \
  typedef typename traits<Joint>::Bias_t Bias_t; \
  typedef typename traits<Joint>::F_t F_t; \
  enum { \
    nq = traits<Joint>::nq, \
    nv = traits<Joint>::nv \
  }

#define SE3_JOINT_USE_INDEXES \
    typedef JointModelBase<JointModel> Base; \
    using Base::idx_q; \
    using Base::idx_v

  template<typename _JointData>
  struct JointDataBase
  {
    typedef typename traits<_JointData>::Joint Joint;
    SE3_JOINT_TYPEDEF;

    JointData& derived() { return *static_cast<JointData*>(this); }
    const JointData& derived() const { return *static_cast<const JointData*>(this); }

    const Constraint_t     & S() const  { return static_cast<const JointData*>(this)->S;   }
    const Transformation_t & M() const  { return static_cast<const JointData*>(this)->M;   }
    const Motion_t         & v() const  { return static_cast<const JointData*>(this)->v;   }
    const Bias_t           & c() const  { return static_cast<const JointData*>(this)->c;   }
    F_t& F()        { return static_cast<      JointData*>(this)->F; }
  };

  template<typename _JointModel>
  struct JointModelBase
  {
    typedef typename traits<_JointModel>::Joint Joint;
    SE3_JOINT_TYPEDEF;

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

  private:
    int i_q,i_v;
  public:
    const int & idx_q() const { return i_q; }
    const int & idx_v() const { return i_v; }
    void setIndexes(int q,int v) { i_q = q; i_v = v; }

    template<typename D>
    typename D::template ConstFixedSegmentReturnType<nv>::Type jointMotion(const Eigen::MatrixBase<D>& a) const     { return a.template segment<nv>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<nv>::Type jointMotion(Eigen::MatrixBase<D>& a) const 
    { return a.template segment<nv>(i_v); }
    template<typename D>
    typename D::template ConstFixedSegmentReturnType<nv>::Type jointForce(const Eigen::MatrixBase<D>& tau) const 
    { return tau.template segment<nv>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<nv>::Type jointForce(Eigen::MatrixBase<D>& tau) const 
    { return tau.template segment<nv>(i_v); }
  };

} // namespace se3

#endif // ifndef __se3_joint_base_hpp__
