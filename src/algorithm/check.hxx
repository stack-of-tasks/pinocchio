//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_check_hxx__
#define __pinocchio_check_hxx__

#include <boost/fusion/algorithm.hpp>
#include <boost/foreach.hpp>

namespace pinocchio
{
  namespace internal
  {
    // Dedicated structure for the fusion::accumulate algorithm: validate the check-algorithm
    // for all elements in a fusion list of AlgoCheckers.
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    struct AlgoFusionChecker
    {
      typedef bool result_type;
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      const Model & model;
      
      AlgoFusionChecker(const Model & model) : model(model) {}

      inline bool operator()(const bool & accumul, const boost::fusion::void_ &) const
      { return accumul; }
      
      template<typename T>
      inline bool operator()(const bool & accumul, const AlgorithmCheckerBase<T> & t) const
      { return accumul && t.checkModel(model); }
    };
  } // namespace internal

  // Check the validity of the kinematic tree defined by parents.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool ParentChecker::checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    for(JointIndex j=1;j<(JointIndex)model.njoints;++j)
      if(model.parents[j]>=j)
        return false;

    return true;
  }

#if !defined(BOOST_FUSION_HAS_VARIADIC_LIST)
  template<BOOST_PP_ENUM_PARAMS(PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE,class T)>
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  bool AlgorithmCheckerList<BOOST_PP_ENUM_PARAMS(PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE,T)>
  ::checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const
  {
    return boost::fusion::accumulate(checkerList,
                                     true,
                                     internal::AlgoFusionChecker<Scalar,Options,JointCollectionTpl>(model));
  }
#else
  template<class ...T>
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  bool AlgorithmCheckerList<T...>::checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const
  {
    return boost::fusion::accumulate(checkerList,
                                     true,
                                     internal::AlgoFusionChecker<Scalar,Options,JointCollectionTpl>(model));
  }
#endif

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool checkData(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef typename Model::JointModel JointModel;
    
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
    BOOST_FOREACH(const typename Data::Matrix6x & F,data.Fcrb)
    {
      CHECK_DATA( F.cols() == model.nv );
    }
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
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar,Options,JointCollectionTpl>::
  check(const DataTpl<Scalar,Options,JointCollectionTpl> & data) const
  { return checkData(*this,data); }


} // namespace pinocchio 

#endif // ifndef __pinocchio_check_hxx__
