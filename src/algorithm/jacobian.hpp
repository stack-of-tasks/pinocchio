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

#ifndef __se3_jacobian_hpp__
#define __se3_jacobian_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include <iostream>
 
namespace se3
{
  // inline const Eigen::MatrixXd&
  // jacobian(const Model & model, 
  // 	   Data& data,
  // 	   const Eigen::VectorXd & q);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct JacobiansForwardStep : public fusion::JointVisitor<JacobiansForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(JacobiansForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;

      const Model::Index & i = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else         data.oMi[i] = data.liMi[i];

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
    }

  };


  inline const Eigen::MatrixXd&
  computeJacobians(const Model & model, Data& data,
		   const Eigen::VectorXd & q)
  {
    for( Model::Index i=1; i< (Model::Index) model.nbody;++i )
      {
	JacobiansForwardStep::run(model.joints[i],data.joints[i],
				  JacobiansForwardStep::ArgsType(model,data,q));
      }

    return data.J;
  }

  /* Return the jacobian of the output frame attached to joint <jointId> in the
     world frame or in the local frame depending on the template argument. The
     function computeJacobians should have been called first. */
  template<bool localFrame>
  void getJacobian(const Model & model, const Data& data,
		   Model::Index jointId, Eigen::MatrixXd & J)
  {
    assert( J.rows() == data.J.rows() );
    assert( J.cols() == data.J.cols() );

    const SE3 & oMjoint = data.oMi[jointId];
    int colRef = nv(model.joints[jointId])+idx_v(model.joints[jointId])-1;
    for(int j=colRef;j>=0;j=data.parents_fromRow[(Model::Index)j])
      {
	if(! localFrame )   J.col(j) = data.J.col(j);
	else                J.col(j) = oMjoint.actInv(Motion(data.J.col(j))).toVector();
      }
  }


  struct JacobianForwardStep : public fusion::JointVisitor<JacobianForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(JacobianForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;

      const Model::Index & i = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.iMf[parent] = data.liMi[i]*data.iMf[i];

      jmodel.jointCols(data.J) = data.iMf[i].inverse().act(jdata.S());
    }

  };

  /* Compute the jacobian of the output frame of the joint <idx> in the local frame. */
  const Eigen::MatrixXd&
  jacobian(const Model & model, Data& data,
	   const Eigen::VectorXd & q,
	   const Model::Index & idx )
  {
    data.iMf[idx] = SE3::Identity();
    for( Model::Index i=idx;i>0;i=model.parents[i] )
      {
	JacobianForwardStep::run(model.joints[i],data.joints[i],
				 JacobianForwardStep::ArgsType(model,data,q));
      }

    return data.J;
  }





} // namespace se3

#endif // ifndef __se3_jacobian_hpp__

