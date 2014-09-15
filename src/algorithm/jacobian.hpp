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

      const Model::Index & i = jmodel.id();
      const Model::Index & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else         data.oMi[i] = data.liMi[i];

      data.J.block(0,jmodel.idx_v(),6,jmodel.nv()) = data.oMi[i].act(jdata.S());
    }

  };


  inline const Eigen::MatrixXd&
  computeJacobians(const Model & model, Data& data,
		   const Eigen::VectorXd & q)
  {
    for( int i=1;i<model.nbody;++i )
      {
	JacobiansForwardStep::run(model.joints[i],data.joints[i],
				  JacobiansForwardStep::ArgsType(model,data,q));
      }

    return data.J;
  }

  template<bool localFrame>
  void getJacobian(const Model & model, const Data& data,
		   Model::Index jointId, Eigen::MatrixXd & J)
  {
    assert( J.rows() == data.J.rows() );
    assert( J.cols() == data.J.cols() );

    const SE3 & oMjoint = data.oMi[jointId];
    int colRef = nv(model.joints[jointId])+idx_v(model.joints[jointId])-1;
    for(int j=colRef;j>=0;j=data.parents_fromRow[j])
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

      const Model::Index & i = jmodel.id();
      const Model::Index & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.iMf[parent] = data.liMi[i]*data.iMf[i];

      data.J.block(0,jmodel.idx_v(),6,jmodel.nv()) = data.iMf[i].inverse().act(jdata.S());
    }

  };

  /* Compute the jacobian in the local frame. */
  const Eigen::MatrixXd&
  jacobian(const Model & model, Data& data,
	   const Eigen::VectorXd & q,
	   const Model::Index & idx )
  {
    data.iMf[idx] = SE3::Identity();
    for( int i=idx;i>0;i=model.parents[i] )
      {
	JacobianForwardStep::run(model.joints[i],data.joints[i],
				 JacobianForwardStep::ArgsType(model,data,q));
      }

    return data.J;
  }





} // namespace se3

#endif // ifndef __se3_jacobian_hpp__

