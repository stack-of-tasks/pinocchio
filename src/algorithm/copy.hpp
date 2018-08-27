//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_copy_hpp__
#define __se3_copy_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"
  
namespace se3
{
  ///
  /// \brief Copy part of the data from <orig> to <dest>. Template parameter can be 
  /// used to select at which differential level the copy should occur.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] orig  Data from which the values are copied.
  /// \param[in] dest  Data to which the values are copied
  /// \param[in] LEVEL if =0, copy oMi. If =1, also copy v. If =2, also copy a, a_gf and f.
  ///
  template<int level>
  inline void
  copy (const Model& model, const Data & origin, Data & dest );  

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
//#include "pinocchio/algorithm/copy.hxx"


namespace se3
{
  template<int LEVEL>
  inline void
  copy (const Model& model, const Data & origin, Data & dest )
  {
    for( se3::JointIndex jid=1; int(jid)<model.njoints; ++ jid )
      {
        assert(LEVEL>=0);

        dest.oMi[jid]      = origin.oMi [jid];
        if(LEVEL>=1) 
          {
            dest.v[jid]    = origin.v   [jid];
          }
        if(LEVEL>=2) 
          {
            dest.a[jid]    = origin.v   [jid];
            dest.a_gf[jid] = origin.a_gf[jid];
            dest.f[jid]    = origin.f   [jid];
          }
      }
  }


} // namespace se3

#endif // ifndef __se3_copy_hpp__
