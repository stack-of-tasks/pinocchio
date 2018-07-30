//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_crba_hxx__
#define __se3_crba_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace se3 
{
  template<typename JointCollection, typename ConfigVectorType>
  struct CrbaForwardStep
  : public fusion::JointVisitorBase< CrbaForwardStep<JointCollection,ConfigVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      jmodel.calc(jdata.derived(),q.derived());
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
    }

  };

  template<typename JointCollection>
  struct CrbaBackwardStep
  : public fusion::JointVisitorBase< CrbaBackwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
				                          Data &>  ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      /*
       * F[1:6,i] = Y*S
       * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
       * if li>0 
       *   Yli += liXi Yi
       *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
       */
      
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Matrix6x::ColsBlockXpr Block;
      const JointIndex & i = jmodel.id();

      /* F[1:6,i] = Y*S */
      //data.Fcrb[i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[i] * jdata.S();
      jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]) 
      = jdata.S().transpose()*data.Fcrb[i].middleCols(jmodel.idx_v(),data.nvSubtree[i]);

      const JointIndex & parent = model.parents[i];
      if(parent>0)
      {
        /*   Yli += liXi Yi */
        data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
        
        /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
        Block jF
        = data.Fcrb[parent].middleCols(jmodel.idx_v(),data.nvSubtree[i]);
        Block iF
        = data.Fcrb[i].middleCols(jmodel.idx_v(),data.nvSubtree[i]);
        forceSet::se3Action(data.liMi[i], iF, jF);
      }
    }
  };
  
  template<typename JointCollection, typename ConfigVectorType>
  struct CrbaForwardStepMinimal
  : public fusion::JointVisitorBase< CrbaForwardStepMinimal<JointCollection,ConfigVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      jmodel.calc(jdata.derived(),q.derived());
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      const JointIndex & parent = model.parents[i];
      if (parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else data.oMi[i] = data.liMi[i];
      
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      
      data.Ycrb[i] = model.inertias[i];
    }
    
  };
  
  template<typename JointCollection>
  struct CrbaBackwardStepMinimal
  : public fusion::JointVisitorBase< CrbaBackwardStepMinimal<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &>  ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      const JointIndex & i = jmodel.id();
      
      /* F[1:6,i] = Y*S */
      jdata.U() = data.Ycrb[i] * jdata.S();
      ColsBlock jF = data.Ag.template middleCols<JointModel::NV>(jmodel.idx_v());
      //        = data.Ag.middleCols(jmodel.idx_v(), jmodel.nv());
      
      forceSet::se3Action(data.oMi[i],jdata.U(),jF);
      
      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = jmodel.jointCols(data.J).transpose()*data.Ag.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      const JointIndex & parent = model.parents[i];
      /*   Yli += liXi Yi */
      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
    }
  };

  
  template<typename JointCollection, typename ConfigVectorType>
  inline const typename DataTpl<JointCollection>::MatrixXs &
  crba(const ModelTpl<JointCollection> & model,
       DataTpl<JointCollection> & data,
       const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    typedef CrbaForwardStep<JointCollection,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived()));
    }
    
    typedef CrbaBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    return data.M;
  }
  
  template<typename JointCollection, typename ConfigVectorType>
  inline const typename DataTpl<JointCollection>::MatrixXs &
  crbaMinimal(const ModelTpl<JointCollection> & model,
              DataTpl<JointCollection> & data,
              const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    typedef CrbaForwardStepMinimal<JointCollection,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived()));
    }
    
    typedef CrbaBackwardStepMinimal<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    // Retrieve the Centroidal Momemtum map
    typedef DataTpl<JointCollection> Data;
    typedef typename Data::Force Force;
    typedef Eigen::Block<typename Data::Matrix6x,3,-1> Block3x;
    
    data.com[0] = data.Ycrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for(long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    return data.M;
  }
  
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------

  namespace internal
  {
    template<typename JointCollection>
    inline bool isDescendant(const ModelTpl<JointCollection> & model,
                             const typename ModelTpl<JointCollection>::JointIndex j,
                             const typename ModelTpl<JointCollection>::JointIndex root)
    {
      typedef ModelTpl<JointCollection> Model;
      typedef typename Model::JointIndex JointIndex;
      
      if(j>=(JointIndex)model.njoints)  return false;
      if(j==0)                          return root==0;
      return (j==root) || isDescendant(model,model.parents[j],root);
    }
  }
  
  template<typename JointCollection>
  inline bool CRBAChecker::checkModel_impl(const ModelTpl<JointCollection> & model) const
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    
    // For CRBA, the tree must be "compact", i.e. all descendants of a node i are stored
    // immediately after i in the "parents" map, i.e. forall joint i, the interval i+1..n-1
    // can be separated in two intervals [i+1..k] and [k+1..n-1], where any [i+1..k] is a descendant
    // of i and none of [k+1..n-1] is a descendant of i.
    for(JointIndex i=1; i < (JointIndex)(model.njoints-1); ++i) // no need to check joints 0 and N-1
      {
        JointIndex k=i+1;
        while(internal::isDescendant(model,k,i)) ++k;
        for( ; int(k)<model.njoints; ++k ) 
          if( internal::isDescendant(model,k,i) ) return false;
      }
    return true;
  }


} // namespace se3

/// @endcond

#endif // ifndef __se3_crba_hxx__
