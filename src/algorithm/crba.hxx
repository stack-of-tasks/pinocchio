//
// Copyright (c) 2015-2017 CNRS
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
  struct CrbaForwardStep : public fusion::JointVisitor<CrbaForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model&,
                                  se3::Data &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(CrbaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
		     const se3::Model & model,
		     se3::Data & data,
		     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
    }

  };

  struct CrbaBackwardStep : public fusion::JointVisitor<CrbaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&>  ArgsType;
    
    JOINT_VISITOR_INIT(CrbaBackwardStep);

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
      typedef Data::Matrix6x::ColsBlockXpr Block;
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();

      /* F[1:6,i] = Y*S */
      //data.Fcrb[i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[i] * jdata.S();
      jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]) 
      = jdata.S().transpose()*data.Fcrb[i].middleCols(jmodel.idx_v(),data.nvSubtree[i]);

      const Model::JointIndex & parent   = model.parents[i];
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

  inline const Eigen::MatrixXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    for( Model::JointIndex i=1;i<(Model::JointIndex)(model.njoints);++i )
    {
      CrbaForwardStep::run(model.joints[i],data.joints[i],
                           CrbaForwardStep::ArgsType(model,data,q));
    }
    
    for( Model::JointIndex i=(Model::JointIndex)(model.njoints-1);i>0;--i )
    {
      CrbaBackwardStep::run(model.joints[i],data.joints[i],
                            CrbaBackwardStep::ArgsType(model,data));
    }

    return data.M;
  }
  
  struct CcrbaForwardStep : public fusion::JointVisitor<CcrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(CcrbaForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
      
      if (parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else data.oMi[i] = data.liMi[i];
    }
    
  }; // struct CcrbaForwardStep
  
  struct CcrbaBackwardStep : public fusion::JointVisitor<CcrbaBackwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
                                  se3::Data &
                                  > ArgsType;
    
    JOINT_VISITOR_INIT(CcrbaBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data)
    {
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
      
      jdata.U() = data.Ycrb[i] * jdata.S();

      ColsBlock jF
        = data.Ag.middleCols <JointModel::NV> (jmodel.idx_v());

      forceSet::se3Action(data.oMi[i],jdata.U(),jF);
    }
    
//    static void algo(const se3::JointModelBase<JointModelComposite> & jmodel,
//                     se3::JointDataBase<JointDataComposite> & jdata,
//                     const se3::Model & model,
//                     se3::Data & data)
//    {
//      typedef SizeDepType<JointModel::NV>::ColsReturn<Data::Matrix6x>::Type ColsBlock;
//      
//      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
//      const Model::Index & parent = model.parents[i];
//      
//      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
//      
//      jdata.U() = data.Ycrb[i] * jdata.S();
//      
//      ColsBlock jF
//        = data.Ag.middleCols(jmodel.idx_v(), jmodel.nv());
//
//      forceSet::se3Action(data.oMi[i],jdata.U(),jF);
//    }

  }; // struct CcrbaBackwardStep
  
  inline const Data::Matrix6x &
  ccrba(const Model & model, Data & data,
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    typedef Eigen::Block <Data::Matrix6x,3,-1> Block3x;
    
    forwardKinematics(model, data, q);
    data.Ycrb[0].setZero();
    for(Model::Index i=1;i<(Model::Index)(model.njoints);++i )
      data.Ycrb[i] = model.inertias[i];
    
    
    for(Model::Index i=(Model::Index)(model.njoints-1);i>0;--i)
    {
      CcrbaBackwardStep::run(model.joints[i],data.joints[i],
                             CcrbaBackwardStep::ArgsType(model,data));
    }
    data.com[0] = data.Ycrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.middleRows<3> (Force::LINEAR);
    Block3x Ag_ang = data.Ag.middleRows<3>  (Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    data.hg = data.Ag*v;
    
    data.Ig.mass() = data.Ycrb[0].mass();
    data.Ig.lever().setZero();
    data.Ig.inertia() = data.Ycrb[0].inertia();
    
    return data.Ag;
  }
  
  struct DCcrbaForwardStep : public fusion::JointVisitor<DCcrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(DCcrbaForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
      
      data.v[i] = jdata.v();
      
      if (parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else data.oMi[i] = data.liMi[i];
    }
    
  }; // struct DCcrbaForwardStep
  
  struct DCcrbaBackwardStep : public fusion::JointVisitor<DCcrbaBackwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &
    > ArgsType;
    
    JOINT_VISITOR_INIT(DCcrbaBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data)
    {
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;

      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      const Motion & v = data.v[i];
      const Inertia & Y = data.oYo[i];
      const Inertia::Matrix6 & doYo = data.doYo[i];
      
      ColsBlock J_cols = jmodel.jointCols(data.J);
      J_cols = data.oMi[i].act(jdata.S());
      
      ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
      const Motion ov(data.oMi[i].act(v));
      motionSet::motionAction(ov,J_cols,dJ_cols);
      
      data.oYo[parent] += Y;
      if(parent > 0)
        data.doYo[parent] += doYo;
      
      // Calc Ag
      ColsBlock Ag_cols = jmodel.jointCols(data.Ag);
      rhsInertiaMult(Y,J_cols,Ag_cols);
      
      // Calc dAg = Ivx + vxI
      ColsBlock dAg_cols = jmodel.jointCols(data.dAg);
      rhsInertiaMult(Y,dJ_cols,dAg_cols);
      dAg_cols += doYo * J_cols;
    }
    
    template<typename Min, typename Mout>
    static void rhsInertiaMultVector(const Inertia & Y,
                                     const Eigen::MatrixBase<Min> & m,
                                     const Eigen::MatrixBase<Mout> & f)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Min,6);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Mout,6);
      Mout & f_ = const_cast<Mout &>(f.derived());
      
      f_.template segment<3>(Inertia::LINEAR) = -Y.mass() * Y.lever().cross(m.template segment<3>(Motion::ANGULAR));
      
      f_.template segment<3>(Inertia::ANGULAR) = Y.inertia() * m.template segment<3>(Motion::ANGULAR);
      f_.template segment<3>(Inertia::ANGULAR) += Y.lever().cross(f_.template segment<3>(Inertia::LINEAR));
      f_.template segment<3>(Inertia::ANGULAR) += Y.mass() * Y.lever().cross(m.template segment<3>(Motion::LINEAR));
      
      f_.template segment<3>(Inertia::LINEAR) += Y.mass() * m.template segment<3>(Motion::LINEAR);
    }
    
    template<typename Min, typename Mout>
    static void rhsInertiaMult(const Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      assert(J.cols() == F.cols());
      Mout & F_ = const_cast<Mout &>(F.derived());
      
      for(int i = 0; i < J.cols(); ++i)
      {
        rhsInertiaMultVector(Y,J.col(i),F_.col(i));
      }
      
    }
    
  }; // struct DCcrbaBackwardStep
  
  inline const Data::Matrix6x &
  dccrba(const Model & model, Data & data,
         const Eigen::VectorXd & q,
         const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    typedef Eigen::Block <Data::Matrix6x,3,-1> Block3x;
    
    forwardKinematics(model,data,q,v);
    data.oYo[0].setZero();
    for(Model::Index i=1;i<(Model::Index)(model.njoints);++i)
    {
      data.oYo[i] = data.oMi[i].act(model.inertias[i]);
      const Motion ov(data.oMi[i].act(data.v[i])); // v_i expressed in the world frame
      data.doYo[i] = data.oYo[i].variation(ov);
    }
    
    for(Model::Index i=(Model::Index)(model.njoints-1);i>0;--i)
    {
      DCcrbaBackwardStep::run(model.joints[i],data.joints[i],
                              DCcrbaBackwardStep::ArgsType(model,data));
    }
    data.com[0] = data.oYo[0].lever();
    
    const Block3x Ag_lin = data.Ag.middleRows<3> (Force::LINEAR);
    Block3x Ag_ang = data.Ag.middleRows<3>  (Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    data.hg = data.Ag*v;
    data.vcom[0] = data.hg.linear()/data.oYo[0].mass();
    
    const Block3x dAg_lin = data.dAg.middleRows<3>(Force::LINEAR);
    Block3x dAg_ang = data.dAg.middleRows<3>(Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      dAg_ang.col(i) += dAg_lin.col(i).cross(data.com[0]);

    data.Ig.mass() = data.oYo[0].mass();
    data.Ig.lever().setZero();
    data.Ig.inertia() = data.oYo[0].inertia();
    
    return data.dAg;
  }

  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------

  namespace internal
  {
    inline bool isDescendant(const Model& model, const JointIndex j, const JointIndex root)
    {
      if(int(j)>=model.njoints)  return false;
      if(j==0)                 return root==0;
      return (j==root) || isDescendant(model,model.parents[j],root);
    }
  }
  
  inline bool CRBAChecker::checkModel_impl( const Model& model ) const
  {
    // For CRBA, the tree must be "compact", i.e. all descendants of a node i are stored
    // immediately after i in the "parents" map, i.e. forall joint i, the interval i+1..n-1
    // can be separated in two intervals [i+1..k] and [k+1..n-1], where any [i+1..k] is a descendant
    // of i and none of [k+1..n-1] is a descendant of i.
    for( JointIndex i=1; int(i)<model.njoints-1; ++i ) // no need to check joints 0 and N-1
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
