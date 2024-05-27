//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_algorithm_utils_force_hpp__
#define __pinocchio_algorithm_utils_force_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{
  ///
  /// @copydoc changeReferenceFrame(const SE3Tpl<Scalar,Options> &,const ForceDense<MotionIn>
  /// &,const ReferenceFrame,const ReferenceFrame) \param[out] f_out Resulting force quantity.
  ///
  template<typename Scalar, int Options, typename ForceIn, typename ForceOut>
  void changeReferenceFrame(
    const SE3Tpl<Scalar, Options> & placement,
    const ForceDense<ForceIn> & f_in,
    const ReferenceFrame rf_in,
    const ReferenceFrame rf_out,
    ForceDense<ForceOut> & f_out)
  {
    if (rf_in == rf_out)
    {
      f_out = f_in;
      return;
    }

    // case: LOCAL/WORLD and WORLD/LOCAL
    if (rf_in == LOCAL && rf_out == WORLD)
    {
      f_out = placement.act(f_in);
      return;
    }
    if (rf_in == WORLD && rf_out == LOCAL)
    {
      f_out = placement.actInv(f_in);
      return;
    }

    // case: LOCAL/LOCAL_WORLD_ALIGNED and LOCAL_WORLD_ALIGNED/LOCAL
    if (rf_in == LOCAL && rf_out == LOCAL_WORLD_ALIGNED)
    {
      f_out.angular().noalias() = placement.rotation() * f_in.angular();
      f_out.linear().noalias() = placement.rotation() * f_in.linear();
      return;
    }
    if (rf_in == LOCAL_WORLD_ALIGNED && rf_out == LOCAL)
    {
      f_out.angular().noalias() = placement.rotation().transpose() * f_in.angular();
      f_out.linear().noalias() = placement.rotation().transpose() * f_in.linear();
      return;
    }

    // case: WORLD/LOCAL_WORLD_ALIGNED and LOCAL_WORLD_ALIGNED/WORLD
    if (rf_in == WORLD && rf_out == LOCAL_WORLD_ALIGNED)
    {
      f_out.linear() = f_in.linear();
      f_out.angular().noalias() = f_in.angular() + f_in.linear().cross(placement.translation());
      return;
    }
    if (rf_in == LOCAL_WORLD_ALIGNED && rf_out == WORLD)
    {
      f_out.linear() = f_in.linear();
      f_out.angular().noalias() = f_in.angular() - f_in.linear().cross(placement.translation());
      return;
    }

    assert(false && "This must never happened.");
  }

  ///
  ///  \brief Change the expression of a given Force vector from one reference frame to another
  /// reference frame.
  ///
  ///  \example If ones has an initial f_in Force expressed locally (rf_in=LOCAL) in a Frame
  /// localized at a given placement value,           ones may want to retrieve its value inside
  /// another reference frame e.g. the world (rf_out=WORLD).
  ///
  /// \param[in] placement Placement of the frame having force f_in
  /// \param[in] f_in Input force quantity.
  /// \param[in] rf_in Reference frame in which m_in is expressed
  /// \param[in] rf_out Reference frame in which the result m_out is expressed
  /// \param[out] f_out Resulting force quantity.
  ///
  template<typename Scalar, int Options, typename ForceIn>
  typename ForceIn::ForcePlain changeReferenceFrame(
    const SE3Tpl<Scalar, Options> & placement,
    const ForceDense<ForceIn> & f_in,
    const ReferenceFrame rf_in,
    const ReferenceFrame rf_out)
  {
    typedef typename ForceIn::ForcePlain ReturnType;
    ReturnType res;

    changeReferenceFrame(placement, f_in, rf_in, rf_out, res);

    return res;
  }

} // namespace pinocchio

#endif // #ifndef __pinocchio_algorithm_utils_force_hpp__
