//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_copy_hpp__
#define __pinocchio_copy_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"
  
namespace pinocchio
{
  ///
  /// \brief Copy part of the data from <orig> to <dest>. Template parameter can be 
  /// used to select at which differential level the copy should occur.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] orig  Data from which the values are copied.
  /// \param[out] dest  Data to which the values are copied
  /// \param[in] LEVEL if =0, copy oMi. If =1, also copy v. If =2, also copy a, a_gf and f.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void
  copy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
       const DataTpl<Scalar,Options,JointCollectionTpl> & origin,
       DataTpl<Scalar,Options,JointCollectionTpl> & dest,
       int LEVEL);

} // namespace pinocchio 

/* --- Details -------------------------------------------------------------------- */
//#include "pinocchio/algorithm/copy.hxx"


namespace pinocchio
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void
  copy(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
       const DataTpl<Scalar,Options,JointCollectionTpl> & origin,
       DataTpl<Scalar,Options,JointCollectionTpl> & dest,
       int LEVEL)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    for(JointIndex jid=1; jid<(JointIndex)model.njoints; ++jid)
      {
        assert(LEVEL>=0);

        dest.oMi[jid]      = origin.oMi [jid];
        if(LEVEL>=1) 
          {
            dest.v[jid]    = origin.v   [jid];
          }
        if(LEVEL>=2) 
          {
            dest.a[jid]    = origin.a   [jid];
            dest.a_gf[jid] = origin.a_gf[jid];
            dest.f[jid]    = origin.f   [jid];
          }
      }
  }


} // namespace pinocchio

#endif // ifndef __pinocchio_copy_hpp__
