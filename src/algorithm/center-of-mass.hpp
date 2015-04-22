#ifndef __se3_center_of_mass_hpp__
#define __se3_center_of_mass_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include <iostream>
 
namespace se3
{
  const Eigen::Vector3d &
  centerOfMass        (const Model & model, Data& data,
		       const Eigen::VectorXd & q,
		       const bool & computeSubtreeComs = true);
  /* The Jcom algorithm also compute the center of mass, using a less efficient
   * approach.  The com can be accessed afterward using data.com[0]. */
  const  Eigen::Matrix<double,3,Eigen::Dynamic> &
  jacobianCenterOfMass(const Model & model, Data& data,
		       const Eigen::VectorXd & q,
		       const bool & computeSubtreeComs = true);

  /* If the CRBA has been run, then both COM and Jcom are easily available from
   * the mass matrix. Use the following methods to access them. In that case,
   * the COM subtrees (also easily available from CRBA data) are not
   * explicitely set. Use data.Ycrb[i].lever() to get them. */
  const Eigen::Vector3d & 
  getComFromCrba        (const Model & model, Data& data);
  const  Eigen::Matrix<double,3,Eigen::Dynamic> &
  getJacobianComFromCrba(const Model & model, Data& data);
} // namespace se3 

/* --- DETAILS -------------------------------------------------------------------- */
/* --- DETAILS -------------------------------------------------------------------- */
/* --- DETAILS -------------------------------------------------------------------- */
namespace se3 
{
 
  struct CenterOfMassForwardStep : public fusion::JointVisitor<CenterOfMassForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Eigen::VectorXd &,
				   const bool &
				   > ArgsType;

    JOINT_VISITOR_INIT(CenterOfMassForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const Eigen::VectorXd & q,
		     const bool & computeSubtreeComs)
    {
      using namespace Eigen;
      using namespace se3;

      const Model::Index & i      = jmodel.id();
      const Model::Index & parent = model.parents[(std::size_t)i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[(std::size_t)i]      = model.jointPlacements[(std::size_t)i]*jdata.M();
      data.com[(std::size_t)parent]  += (data.liMi[(std::size_t)i].rotation()*data.com[(std::size_t)i]
			    +data.mass[(std::size_t)i]*data.liMi[(std::size_t)i].translation());
      data.mass[(std::size_t)parent] += data.mass[(std::size_t)i];  

      if( computeSubtreeComs )
	data.com[(std::size_t)i] /= data.mass[(std::size_t)i]; 
    }

  };

  /* Compute the centerOfMass in the local frame. */
  const Eigen::Vector3d &
  centerOfMass(const Model & model, Data& data,
	       const Eigen::VectorXd & q,
	       const bool & computeSubtreeComs )
  {
    data.mass[0] = 0; 
    data.com[0].setZero ();

    for( int i=1;i<model.nbody;++i )
      {
	data.com[(std::size_t)i]  = model.inertias[(std::size_t)i].mass()*model.inertias[(std::size_t)i].lever();
	data.mass[(std::size_t)i] = model.inertias[(std::size_t)i].mass();
      }

    for( int i=model.nbody-1;i>0;--i )
      {
	CenterOfMassForwardStep
	  ::run(model.joints[(std::size_t)i],data.joints[(std::size_t)i],
		CenterOfMassForwardStep::ArgsType(model,data,q,computeSubtreeComs));
      }
    data.com[0] /= data.mass[0];

    return data.com[0];
  }

  const Eigen::Vector3d & getComFromCrba(const Model & , Data& data)
  {
    return data.com[0] = data.Ycrb[1].lever();
  }

  /* --- JACOBIAN ---------------------------------------------------------- */
  /* --- JACOBIAN ---------------------------------------------------------- */
  /* --- JACOBIAN ---------------------------------------------------------- */
  struct JacobianCenterOfMassForwardStep 
    : public fusion::JointVisitor<JacobianCenterOfMassForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(JacobianCenterOfMassForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;

      const Model::Index & i      = jmodel.id();
      const Model::Index & parent = model.parents[(std::size_t)i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[(std::size_t)i]      = model.jointPlacements[(std::size_t)i]*jdata.M();
      if(parent>0) data.oMi[(std::size_t)i] = data.oMi[(std::size_t)parent]*data.liMi[(std::size_t)i];
      else         data.oMi[(std::size_t)i] = data.liMi[(std::size_t)i];
      
      data.com[(std::size_t)i]   = model.inertias[(std::size_t)i].mass()*data.oMi[(std::size_t)i].act(model.inertias[(std::size_t)i].lever());
      data.mass[(std::size_t)i]  = model.inertias[(std::size_t)i].mass();
    }

  };

  struct JacobianCenterOfMassBackwardStep 
    : public fusion::JointVisitor<JacobianCenterOfMassBackwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const bool &
				   > ArgsType;

    JOINT_VISITOR_INIT(JacobianCenterOfMassBackwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const bool & computeSubtreeComs )
    {
      using namespace Eigen;
      using namespace se3;

      const Model::Index & i      = jmodel.id();
      const Model::Index & parent = model.parents[(std::size_t)i];

      data.com[(std::size_t)parent]  += data.com[(std::size_t)i];
      data.mass[(std::size_t)parent] += data.mass[(std::size_t)i];

      const Eigen::Matrix<double,6,JointModel::NV> & oSk
	= data.oMi[(std::size_t)i].act(jdata.S());

      if( JointModel::NV==1 )
      	data.Jcom.col(jmodel.idx_v()) // Using head and tail would confuse g++
      	  = data.mass[(std::size_t)i]*oSk.template topLeftCorner<3,1>() 
      	  - data.com[(std::size_t)i].cross(oSk.template bottomLeftCorner<3,1>()) ;
      else
      	data.Jcom.template block<3,JointModel::NV>(0,jmodel.idx_v())
      	  = data.mass[(std::size_t)i]*oSk.template topRows<3>() 
      	  - skew(data.com[(std::size_t)i]) * oSk.template bottomRows<3>() ;

      if(computeSubtreeComs)
	data.com[(std::size_t)i]       /= data.mass[(std::size_t)i];
    }

  };

  /* Compute the centerOfMass in the local frame. */
  const Eigen::Matrix<double,3,Eigen::Dynamic> &
  jacobianCenterOfMass(const Model & model, Data& data,
		       const Eigen::VectorXd & q,
		       const bool & computeSubtreeComs )
  {
    data.com[0].setZero ();
    data.mass[0] = 0;
    for( int i=1;i<model.nbody;++i )
      {
	JacobianCenterOfMassForwardStep
	  ::run(model.joints[(std::size_t)i],data.joints[(std::size_t)i],
		JacobianCenterOfMassForwardStep::ArgsType(model,data,q));
      }
    for( int i=model.nbody-1;i>0;--i )
      {
	JacobianCenterOfMassBackwardStep
	  ::run(model.joints[(std::size_t)i],data.joints[(std::size_t)i],
		JacobianCenterOfMassBackwardStep::ArgsType(model,data,computeSubtreeComs));
      }

    data.com[0] /= data.mass[0];
    data.Jcom /=  data.mass[0];
    return data.Jcom;
  }

  const Eigen::Matrix<double,3,Eigen::Dynamic> &
  getJacobianComFromCrba(const Model & , Data& data)
  {
    data.Jcom = data.M.topRows<3>()/data.M(0,0);
    return data.Jcom;
  }



} // namespace se3

#endif // ifndef __se3_center_of_mass_hpp__

