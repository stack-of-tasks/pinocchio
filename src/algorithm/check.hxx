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

#ifndef __se3_check_hxx__
#define __se3_check_hxx__

#include <boost/fusion/algorithm.hpp>
#include <boost/foreach.hpp>

namespace se3
{
  namespace internal
  {
    // Dedicated structure for the fusion::accumulate algorithm: validate the check-algorithm
    // for all elements in a fusion list of AlgoCheckers.
    struct AlgoFusionChecker
    {
      typedef bool result_type;
      const Model& model;
      
      AlgoFusionChecker(const Model&model) : model(model) {}

      template<typename T>
      inline bool operator()(const bool& accumul, const T& t) const
      { return accumul && t.checkModel(model); }
    };
  } // namespace internal

  // Calls model.check for each checker in the fusion::list.
  // Each list element is supposed to implement the AlgorithmCheckerBase API.
  template<class T1,class T2,class T3,class T4,class T5,
           class T6,class T7,class T8,class T9,class T10>
  inline bool Model::check( const boost::fusion::list<T1,T2,T3,T4,T5,T6,T7,T8,T9,T10> & checkerList ) const
  { return boost::fusion::accumulate(checkerList,true,internal::AlgoFusionChecker(*this)); }

  // Check the validity of the kinematic tree defined by parents.
  inline bool ParentChecker::checkModel_impl( const Model& model ) const
  {
    for( JointIndex j=1;(int)j<model.njoints;++j )
      if( model.parents[j]>=j ) return false;

    return true;
  }

  inline bool checkData(const Model & model, const Data & data)
  {
#define CHECK_DATA(a)  if(!(a)) return false;

    // TODO JMinvJt,sDUiJt are never explicitly initialized.
    // TODO impulse_c
    // They are not check neither

    CHECK_DATA( (int)data.joints.size()   == model.njoints );
    CHECK_DATA( (int)data.a.size()        == model.njoints );
    CHECK_DATA( (int)data.a_gf.size()     == model.njoints );
    CHECK_DATA( (int)data.v.size()        == model.njoints );
    CHECK_DATA( (int)data.f.size()        == model.njoints );
    CHECK_DATA( (int)data.oMi.size()      == model.njoints );
    CHECK_DATA( (int)data.liMi.size()     == model.njoints );
    CHECK_DATA( (int)data.Ycrb.size()     == model.njoints );
    CHECK_DATA( (int)data.Yaba.size()     == model.njoints );
    CHECK_DATA( (int)data.Fcrb.size()     == model.njoints );
    BOOST_FOREACH(const Data::Matrix6x & F,data.Fcrb) CHECK_DATA( F.cols() == model.nv );
    CHECK_DATA( (int)data.iMf.size()      == model.njoints );
    CHECK_DATA( (int)data.iMf.size()      == model.njoints );
    CHECK_DATA( (int)data.com.size()      == model.njoints );
    CHECK_DATA( (int)data.vcom.size()     == model.njoints );
    CHECK_DATA( (int)data.acom.size()     == model.njoints );
    CHECK_DATA( (int)data.mass.size()     == model.njoints );

    CHECK_DATA( data.tau.size()      == model.nv );
    CHECK_DATA( data.nle.size()      == model.nv );
    CHECK_DATA( data.ddq.size()      == model.nv );
    CHECK_DATA( data.u.size()        == model.nv );
    CHECK_DATA( data.M.rows()        == model.nv );
    CHECK_DATA( data.M.cols()        == model.nv );
    CHECK_DATA( data.Ag.cols()       == model.nv );
    CHECK_DATA( data.U.cols()        == model.nv );
    CHECK_DATA( data.U.rows()        == model.nv );
    CHECK_DATA( data.D.size()        == model.nv );
    CHECK_DATA( data.tmp.size()      == model.nv );
    CHECK_DATA( data.J.cols()        == model.nv );
    CHECK_DATA( data.Jcom.cols()     == model.nv );
    CHECK_DATA( data.torque_residual.size() == model.nv );
    CHECK_DATA( data.dq_after.size() == model.nv );
    //CHECK_DATA( data.impulse_c.size()== model.nv );

    CHECK_DATA( (int)data.oMf.size()      == model.nframes );

    CHECK_DATA( (int)data.lastChild.size()         == model.njoints );
    CHECK_DATA( (int)data.nvSubtree.size()         == model.njoints );
    CHECK_DATA( (int)data.parents_fromRow.size()   == model.nv );
    CHECK_DATA( (int)data.nvSubtree_fromRow.size() == model.nv );

    for( JointIndex j=1;int(j)<model.njoints;++j )
      {
        JointIndex c = (JointIndex)data.lastChild[j];
        CHECK_DATA((int)c<model.njoints);
        int nv=model.joints[j].nv();
        for( JointIndex d=j+1;d<=c;++d ) // explore all descendant
          {
            CHECK_DATA( model.parents[d]>=j );
            nv+=model.joints[d].nv();
          }
        CHECK_DATA(nv==data.nvSubtree[j]);
        
        for( JointIndex d=c+1;(int)d<model.njoints;++d)
          CHECK_DATA( (model.parents[d]<j)||(model.parents[d]>c) );

        int row = model.joints[j].idx_v();
        CHECK_DATA(data.nvSubtree[j] == data.nvSubtree_fromRow[(size_t)row]);
        
        const JointModel & jparent = model.joints[model.parents[j]];
        if(row==0) { CHECK_DATA(data.parents_fromRow[(size_t)row]==-1); }
        else       { CHECK_DATA(jparent.idx_v()+jparent.nv()-1 == data.parents_fromRow[(size_t)row]); }
      }

#undef CHECK_DATA
    return true;
  }


} // namespace se3 

#endif // ifndef __se3_check_hxx__
