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

#ifndef __se3_center_of_mass_hpp__
#define __se3_center_of_mass_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <iostream>
 
namespace se3
{
  
  ///
  /// \brief Computes the center of mass position of a given model according to a particular joint configuration.
  ///        The result is accessible throw data.com[0] for the full body com and data.com[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the rigid body system expressed in the world frame (vector 3).
  ///
  const Eigen::Vector3d &
  centerOfMass(const Model & model, Data& data,
               const Eigen::VectorXd & q,
               const bool & computeSubtreeComs = true);
  
  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to a particular joint configuration, velocity and acceleration.
  ///        The result is accessible throw data.com[0], data.vcom[0], data.acom[0] for the full body com position, velocity and acceleation.
  ///        And data.com[i], data.vcom[i] and data.acom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  void centerOfMassAcceleration(const Model & model, Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v,
                                const Eigen::VectorXd & a,
                                const bool & computeSubtreeComs = true);
  
  ///
  /// \brief Computes both the jacobian and the the center of mass position of a given model according to a particular joint configuration.
  ///        The results are accessible throw data.Jcom and data.com[0] and are both expressed in the world frame.
  ///        And data.com[i] gives the center of mass of the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The jacobian of center of mass position of the rigid body system expressed in the world frame (matrix 3 x model.nv).
  ///
  const Eigen::Matrix<double,3,Eigen::Dynamic> &
  jacobianCenterOfMass(const Model & model, Data & data,
                       const Eigen::VectorXd & q,
                       const bool & computeSubtreeComs = true);

  /* If the CRBA has been run, then both COM and Jcom are easily available from
   * the mass matrix. Use the following methods to access them. In that case,
   * the COM subtrees (also easily available from CRBA data) are not
   * explicitely set. Use data.Ycrb[i].lever() to get them. */
  ///
  /// \brief Extracts the center of mass position from the joint space inertia matrix (also called the mass matrix).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The center of mass position of the rigid body system expressed in the world frame (vector 3).
  ///
  const Eigen::Vector3d & 
  getComFromCrba(const Model & model, Data & data);
  
  ///
  /// \brief Extracts both the jacobian of the center of mass (CoM) and the CoM position from the joint space inertia matrix (also called the mass matrix).
  ///        The results are accessible throw data.Jcom and data.com[0] and are both expressed in the world frame.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The jacobian of the CoM expressed in the world frame (matrix 3 x model.nv).
  ///
  const Eigen::Matrix<double,3,Eigen::Dynamic> &
  getJacobianComFromCrba(const Model & model, Data & data);
  
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

      const Model::Index & i      = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i]      = model.jointPlacements[i]*jdata.M();
      data.com[parent]  += (data.liMi[i].rotation()*data.com[i]
			    +data.mass[i]*data.liMi[i].translation());
      data.mass[parent] += data.mass[i];

      if( computeSubtreeComs )
	data.com[i] /= data.mass[i]; 
    }

  };

  /* Compute the centerOfMass in the local frame of the root joint. */
  const Eigen::Vector3d &
  centerOfMass(const Model & model, Data& data,
	       const Eigen::VectorXd & q,
	       const bool & computeSubtreeComs )
  {
    data.mass[0] = 0; 
    data.com[0].setZero ();

    for( Model::Index i=1;i<(Model::Index)(model.nbody);++i )
      {
	data.com[i]  = model.inertias[i].mass()*model.inertias[i].lever();
	data.mass[i] = model.inertias[i].mass();
        
        
      }

    for( Model::Index i=(Model::Index)(model.nbody-1);i>0;--i )
      {
	CenterOfMassForwardStep
	  ::run(model.joints[i],data.joints[i],
		CenterOfMassForwardStep::ArgsType(model,data,q,computeSubtreeComs));
        
        
      }
    data.com[0] /= data.mass[0];

    return data.com[0];
  }
  
  /* Compute the centerOfMass position, velocity and acceleration in the local frame of the root joint. */
  void
  centerOfMassAcceleration(const Model & model, Data & data,
                           const Eigen::VectorXd & q,
                           const Eigen::VectorXd & v,
                           const Eigen::VectorXd & a,
                           const bool & computeSubtreeComs)
  {
    using namespace se3;
    
    data.mass[0] = 0;
    data.com[0].setZero ();
    data.vcom[0].setZero ();
    data.acom[0].setZero ();
    
    // Forward Step
    dynamics(model, data, q, v, a);
    for(Model::Index i=1;i<(Model::Index)(model.nbody);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      const Motion & v = data.v[i];
      const Motion & a = data.a[i];
      
      data.com[i]  = mass * lever;
      data.mass[i] = mass;
      
      SE3::Vector3 vcom_local (v.angular().cross(lever) + v.linear());
      data.vcom[i] = mass * (vcom_local);
      data.acom[i] = mass * (a.angular().cross(lever) + a.linear() + v.angular().cross(vcom_local)); // take into accound the coriolis part of the acceleration
      
    }
    
    // Backward Step
    for(Model::Index i=(Model::Index)(model.nbody-1); i>0; --i)
    {
      const Model::Index & parent = model.parents[i];
      
      const SE3 & liMi = data.liMi[i];
      
      data.com[parent] += (liMi.rotation()*data.com[i]
                           + data.mass[i] * liMi.translation());
      
      data.vcom[parent] += liMi.rotation()*data.vcom[i];
      data.acom[parent] += liMi.rotation()*data.acom[i];
      data.mass[parent] += data.mass[i];
      
      if( computeSubtreeComs )
      {
        data.com[i] /= data.mass[i];
        data.vcom[i] /= data.mass[i];
        data.acom[i] /= data.mass[i];
      }
    }
    
    data.com[0] /= data.mass[0];
    data.vcom[0] /= data.mass[0];
    data.acom[0] /= data.mass[0];
  }

  const Eigen::Vector3d & getComFromCrba(const Model & , Data& data)
  {
    return data.com[0] = data.liMi[1].act(data.Ycrb[1].lever());
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

      const Model::Index & i      = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i]      = model.jointPlacements[i]*jdata.M();
      if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else         data.oMi[i] = data.liMi[i];
      
      data.com[i]   = model.inertias[i].mass()*data.oMi[i].act(model.inertias[i].lever());
      data.mass[i]  = model.inertias[i].mass();
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

      const Model::Index & i      = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];

      data.com[parent]  += data.com[i];
      data.mass[parent] += data.mass[i];

      const Eigen::Matrix<double,6,JointModel::NV> & oSk
	= data.oMi[i].act(jdata.S());

      if( JointModel::NV==1 )
      	data.Jcom.col(jmodel.idx_v()) // Using head and tail would confuse g++
      	  = data.mass[i]*oSk.template topLeftCorner<3,1>() 
      	  - data.com[i].cross(oSk.template bottomLeftCorner<3,1>()) ;
      else
      	jmodel.jointCols(data.Jcom)
          //data.Jcom.template block<3,JointModel::NV>(0,jmodel.idx_v())
      	  = data.mass[i]*oSk.template topRows<3>() 
      	  - skew(data.com[i]) * oSk.template bottomRows<3>() ;

      if(computeSubtreeComs)
	data.com[i]       /= data.mass[i];
    }

  };

  /* Compute the centerOfMass in the local frame of the root joint. */
  const Eigen::Matrix<double,3,Eigen::Dynamic> &
  jacobianCenterOfMass(const Model & model, Data& data,
		       const Eigen::VectorXd & q,
		       const bool & computeSubtreeComs )
  {
    data.com[0].setZero ();
    data.mass[0] = 0;
    for( Model::Index i=1;i<(Model::Index)model.nbody;++i )
      {
	JacobianCenterOfMassForwardStep
	  ::run(model.joints[i],data.joints[i],
		JacobianCenterOfMassForwardStep::ArgsType(model,data,q));
      }
    for( Model::Index i= (Model::Index) (model.nbody-1);i>0;--i )
      {
	JacobianCenterOfMassBackwardStep
	  ::run(model.joints[i],data.joints[i],
		JacobianCenterOfMassBackwardStep::ArgsType(model,data,computeSubtreeComs));
      }

    data.com[0] /= data.mass[0];
    data.Jcom /=  data.mass[0];
    return data.Jcom;
  }

  const Eigen::Matrix<double,3,Eigen::Dynamic> &
  getJacobianComFromCrba(const Model &, Data & data)
  {
    const SE3 & oM1 = data.liMi[1];
  
    // As the 6 first rows of M*a is a wrench, we just need to multiply by the
    // relative rotation between the first joint and the world
    const SE3::Matrix3 & oR1_over_m = oM1.rotation() / data.M(0,0);
  
    data.Jcom = oR1_over_m * data.M.topRows<3> ();
    return data.Jcom;
  }

} // namespace se3

#endif // ifndef __se3_center_of_mass_hpp__

