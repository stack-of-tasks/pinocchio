//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_crba_hpp__
#define __se3_crba_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <iostream>
  
namespace se3
{
  ///
  /// \brief Computes the upper triangular part of the joint space inertia matrix M by
  ///        using the Composite Rigid Body Algorithm (Chapter 6, Rigid-Body Dynamics Algorithms, R. Featherstone, 2008).
  ///        The result is accessible throw data.M.
  ///
  /// \note You can easly get data.M symetric by copying the stricly upper trinangular part
  ///       in the stricly lower tringular part with
  ///       data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The joint space inertia matrix with only the upper triangular part computed.
  ///
  inline const Eigen::MatrixXd &
  crba(const Model & model,
       Data & data,
       const Eigen::VectorXd & q);
  
  ///
  /// \brief Computes the upper triangular part of the joint space inertia matrix M by
  ///        using the Composite Rigid Body Algorithm (Chapter 6, Rigid-Body Dynamics Algorithms, R. Featherstone, 2008).
  ///        The result is accessible throw data.M.
  ///
  /// \note You can easly get data.M symetric by copying the stricly upper trinangular part
  ///       in the stricly lower tringular part with
  ///       data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint configuration vector (dim model.nv).
  ///
  /// \return The joint space inertia matrix with only the upper triangular part computed.
  ///
  inline const Data::Matrix6x &
  ccrba(const Model & model,
        Data & data,
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & v);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct CrbaForwardStep : public fusion::JointVisitor<CrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(CrbaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;

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
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model& model,
		     Data& data)
    {
      /*
       * F[1:6,i] = Y*S
       * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
       * if li>0 
       *   Yli += liXi Yi
       *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
       */
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();

      /* F[1:6,i] = Y*S */
      //data.Fcrb[i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[i] * jdata.S();
      jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]) 
      = jdata.S().transpose()*data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);

      const Model::JointIndex & parent   = model.parents[i];
      if(parent>0)
      {
        /*   Yli += liXi Yi */
        data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
        
        /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
        Eigen::Block<typename Data::Matrix6x> jF
        = data.Fcrb[parent].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);
        Eigen::Block<typename Data::Matrix6x> iF
        = data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);
        forceSet::se3Action(data.liMi[i], iF, jF);
      }
      
      // std::cout << "iYi = " << (Inertia::Matrix6)data.Ycrb[i] << std::endl;
      // std::cout << "iSi = " << ConstraintXd(jdata.S()).matrix() << std::endl;
      // std::cout << "liFi = " << jdata.F() << std::endl;
      // std::cout << "M = " <<  data.M << std::endl;
    }
  };

  inline const Eigen::MatrixXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q)
  {
    for( Model::JointIndex i=1;i<(Model::JointIndex)(model.nbody);++i )
      {
	CrbaForwardStep::run(model.joints[i],data.joints[i],
			     CrbaForwardStep::ArgsType(model,data,q));
      }
    
    for( Model::JointIndex i=(Model::JointIndex)(model.nbody-1);i>0;--i )
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
    const Model::Index,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(CcrbaForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::Index i,
                     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;
      
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
    se3::Data &,
    const Model::Index
    > ArgsType;
    
    JOINT_VISITOR_INIT(CcrbaBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::Index i)
    {
      using namespace Eigen;
      using namespace se3;
      
      const Model::Index & parent = model.parents[i];
      
      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
      
      jdata.U() = data.Ycrb[i] * jdata.S();
      Eigen::Block<typename Data::Matrix6x> jF
      = data.Ag.block(0,jmodel.idx_v(),6,JointModel::NV);
      Eigen::Block<typename JointModel::U_t> iF
      = jdata.U().block(0,0,6,JointModel::NV);
      forceSet::se3Action(data.oMi[i],iF,jF);
    }
    
  }; // struct CcrbaBackwardStep
  
  inline const Data::Matrix6x &
  ccrba(const Model & model, Data & data,
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & v)
  {
    typedef Eigen::Block <Data::Matrix6x,3,-1> Block3x;
    
    forwardKinematics(model, data, q);
    for( Model::Index i=1;i<(Model::Index)(model.nbody);++i )
    {
//      CcrbaForwardStep::run(model.joints[i],data.joints[i],
//                            CcrbaForwardStep::ArgsType(model,data,i,q));
      data.Ycrb[i] = model.inertias[i];
    }
    
    data.Ycrb[0].setZero();
    for(Model::Index i=(Model::Index)(model.nbody-1);i>0;--i)
    {
      CcrbaBackwardStep::run(model.joints[i],data.joints[i],
                             CcrbaBackwardStep::ArgsType(model,data,i));
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
} // namespace se3

#endif // ifndef __se3_crba_hpp__

