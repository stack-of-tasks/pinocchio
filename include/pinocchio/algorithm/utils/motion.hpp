//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_algorithm_utils_motion_hpp__
#define __pinocchio_algorithm_utils_motion_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{

  ///
  /// @copydoc changeReferenceFrame(const SE3Tpl<Scalar,Options> &,const MotionDense<MotionIn>
  /// &,const ReferenceFrame,const ReferenceFrame) \param[out] m_out Resulting motion quantity.
  ///
  template<typename Scalar, int Options, typename MotionIn, typename MotionOut>
  void changeReferenceFrame(
    const SE3Tpl<Scalar, Options> & placement,
    const MotionDense<MotionIn> & m_in,
    const ReferenceFrame rf_in,
    const ReferenceFrame rf_out,
    MotionDense<MotionOut> & m_out)
  {
    if (rf_in == rf_out)
    {
      m_out = m_in;
      return;
    }

    // case: LOCAL/WORLD and WORLD/LOCAL
    if (rf_in == LOCAL && rf_out == WORLD)
    {
      m_out = placement.act(m_in);
      return;
    }
    if (rf_in == WORLD && rf_out == LOCAL)
    {
      m_out = placement.actInv(m_in);
      return;
    }

    // case: LOCAL/LOCAL_WORLD_ALIGNED and LOCAL_WORLD_ALIGNED/LOCAL
    if (rf_in == LOCAL && rf_out == LOCAL_WORLD_ALIGNED)
    {
      m_out.linear().noalias() = placement.rotation() * m_in.linear();
      m_out.angular().noalias() = placement.rotation() * m_in.angular();
      return;
    }
    if (rf_in == LOCAL_WORLD_ALIGNED && rf_out == LOCAL)
    {
      m_out.linear().noalias() = placement.rotation().transpose() * m_in.linear();
      m_out.angular().noalias() = placement.rotation().transpose() * m_in.angular();
      return;
    }

    // case: WORLD/LOCAL_WORLD_ALIGNED and LOCAL_WORLD_ALIGNED/WORLD
    if (rf_in == WORLD && rf_out == LOCAL_WORLD_ALIGNED)
    {
      m_out.angular() = m_in.angular();
      m_out.linear().noalias() = m_in.linear() + m_in.angular().cross(placement.translation());
      return;
    }
    if (rf_in == LOCAL_WORLD_ALIGNED && rf_out == WORLD)
    {
      m_out.angular() = m_in.angular();
      m_out.linear().noalias() = m_in.linear() - m_in.angular().cross(placement.translation());
      return;
    }

    assert(false && "This must never happened.");
  }

  ///
  ///  \brief Change the expression of a given Motion vector from one reference frame to another
  /// reference frame.
  ///
  ///  \example If ones has an initial m_in Motion expressed locally (rf_in=LOCAL) in a Frame
  /// localized at a given placement value,           ones may want to retrieve its value inside
  /// another reference frame e.g. the world (rf_out=WORLD).
  ///
  /// \param[in] placement Placement of the frame having velocity m_in
  /// \param[in] m_in Input motion quantity.
  /// \param[in] rf_in Reference frame in which m_in is expressed
  /// \param[in] rf_out Reference frame in which the result m_out is expressed
  /// \param[out] m_out Resulting motion quantity.
  ///
  template<typename Scalar, int Options, typename MotionIn>
  typename MotionIn::MotionPlain changeReferenceFrame(
    const SE3Tpl<Scalar, Options> & placement,
    const MotionDense<MotionIn> & m_in,
    const ReferenceFrame rf_in,
    const ReferenceFrame rf_out)
  {
    typedef typename MotionIn::MotionPlain ReturnType;
    ReturnType res;

    changeReferenceFrame(placement, m_in, rf_in, rf_out, res);

    return res;
  }

} // namespace pinocchio

#endif // #ifndef __pinocchio_multibody_fwd_hpp__
