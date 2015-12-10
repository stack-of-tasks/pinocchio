//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_urdf_hpp__
#define __se3_urdf_hpp__

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>
#include <boost/foreach.hpp>
#include "pinocchio/multibody/model.hpp"

namespace urdf
{
  typedef boost::shared_ptr<ModelInterface> ModelInterfacePtr;
  typedef boost::shared_ptr<const Joint> JointConstPtr;
  typedef boost::shared_ptr<const Link> LinkConstPtr;
  typedef boost::shared_ptr<Link> LinkPtr;
  typedef boost::shared_ptr<const Inertial> InertialConstPtr;
}

namespace se3
{
  namespace urdf
  {
    Inertia convertFromUrdf( const ::urdf::Inertial& Y )
    {
      const ::urdf::Vector3 & p = Y.origin.position;
      const ::urdf::Rotation & q = Y.origin.rotation;

      const Eigen::Vector3d com(p.x,p.y,p.z);
      const Eigen::Matrix3d & R = Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix();

      Eigen::Matrix3d I; I << Y.ixx,Y.ixy,Y.ixz
			   ,  Y.ixy,Y.iyy,Y.iyz
			   ,  Y.ixz,Y.iyz,Y.izz;
      return Inertia(Y.mass,com,R*I*R.transpose());
    }

    SE3 convertFromUrdf( const ::urdf::Pose & M )
    {
      const ::urdf::Vector3 & p = M.position;
      const ::urdf::Rotation & q = M.rotation;
      return SE3( Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix(), Eigen::Vector3d(p.x,p.y,p.z));
    }

    enum AxisCartesian { AXIS_X, AXIS_Y, AXIS_Z, AXIS_UNALIGNED };
    AxisCartesian extractCartesianAxis( const ::urdf::Vector3 & axis )
    {
      if( (axis.x==1.0)&&(axis.y==0.0)&&(axis.z==0.0) )
	return AXIS_X;
      else if( (axis.x==0.0)&&(axis.y==1.0)&&(axis.z==0.0) )
	return AXIS_Y;
      else if( (axis.x==0.0)&&(axis.y==0.0)&&(axis.z==1.0) )
	return AXIS_Z;
      else
	return AXIS_UNALIGNED;
    }


    void parseTree( ::urdf::LinkConstPtr link, Model & model, const SE3 & placementOffset = SE3::Identity()) 
{


  ::urdf::JointConstPtr joint = link->parent_joint;
  SE3 nextPlacementOffset = SE3::Identity(); // OffSet of the next link. In case we encounter a fixed joint, we need to propagate the length of its attached body to next joint.

  // std::cout << " *** " << link->name << "    < attached by joint ";
  // if(joint)
  //   std::cout << "#" << link->parent_joint->name << std::endl;
  // else std::cout << "###ROOT" << std::endl;


  //assert(link->inertial && "The parser cannot accept trivial mass");
  // FIXME: Inertia must not be set to identity when link has no inertial tag.
  const Inertia & Y = (link->inertial) ?  convertFromUrdf(*link->inertial) :
                                          Inertia::Identity();

  // std::cout << "placementOffset: " << placementOffset << std::endl;

  bool visual = (link->visual) ? true : false;

  if(joint!=NULL)
  {
    assert(link->getParent()!=NULL);

    Model::Index parent = (link->getParent()->parent_joint==NULL) ? (model.existJointName("root_joint") ? model.getJointId("root_joint") : 0) :
                                                                    model.getJointId( link->getParent()->parent_joint->name );
    //std::cout << joint->name << " === " << parent << std::endl;

    const SE3 & jointPlacement = placementOffset*convertFromUrdf(joint->parent_to_joint_origin_transform);

    //std::cout << "Parent = " << parent << std::endl;
    //std::cout << "Placement = " << (Matrix4)jointPlacement << std::endl;

    switch(joint->type)
    {
      case ::urdf::Joint::REVOLUTE:
      case ::urdf::Joint::CONTINUOUS: // Revolute with no joint limits
      {

        Eigen::VectorXd maxEffort;
        Eigen::VectorXd velocity;
        Eigen::VectorXd lowerPosition;
        Eigen::VectorXd upperPosition;

        if (joint->limits)
        {
          maxEffort.resize(1);      maxEffort     << joint->limits->effort;
          velocity.resize(1);       velocity      << joint->limits->velocity;
          lowerPosition.resize(1);  lowerPosition << joint->limits->lower;
          upperPosition.resize(1);  upperPosition << joint->limits->upper;
        }

        Eigen::Vector3d jointAxis(Eigen::Vector3d::Zero());
        AxisCartesian axis = extractCartesianAxis(joint->axis);
        switch(axis)
        {
          case AXIS_X:
            model.addBody(  parent, JointModelRX(), jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          case AXIS_Y:
            model.addBody(  parent, JointModelRY(), jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          case AXIS_Z:
            model.addBody(  parent, JointModelRZ(), jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          case AXIS_UNALIGNED:
            jointAxis= Eigen::Vector3d( joint->axis.x,joint->axis.y,joint->axis.z );
            jointAxis.normalize();
            model.addBody(  parent, JointModelRevoluteUnaligned(jointAxis), 
                            jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          default:
            assert( false && "Fatal Error while extracting revolute joint axis");
            break;
        }
        break;
      }
      case ::urdf::Joint::PRISMATIC:
      {
        Eigen::VectorXd maxEffort = Eigen::VectorXd(0.);
        Eigen::VectorXd velocity = Eigen::VectorXd(0.);
        Eigen::VectorXd lowerPosition = Eigen::VectorXd(0.);
        Eigen::VectorXd upperPosition = Eigen::VectorXd(0.);

        if (joint->limits)
        {
          maxEffort.resize(1);      maxEffort     << joint->limits->effort;
          velocity.resize(1);       velocity      << joint->limits->velocity;
          lowerPosition.resize(1);  lowerPosition << joint->limits->lower;
          upperPosition.resize(1);  upperPosition << joint->limits->upper;
        }
        AxisCartesian axis = extractCartesianAxis(joint->axis);   
        switch(axis)
        {
          case AXIS_X:
            model.addBody(  parent, JointModelPX(), jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          case AXIS_Y:
            model.addBody(  parent, JointModelPY(), jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          case AXIS_Z:
            model.addBody(  parent, JointModelPZ(), jointPlacement, Y,
                            maxEffort, velocity, lowerPosition, upperPosition,
                            joint->name,link->name, visual );
            break;
          case AXIS_UNALIGNED:
            std::cerr << "Bad axis = (" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")" << std::endl;
            assert(false && "Only X, Y or Z axis are accepted." );
            break;
          default:
            assert( false && "Fatal Error while extracting prismatic joint axis");
            break;
        }
        break;
      }
      case ::urdf::Joint::FIXED:
      {
        // In case of fixed joint, if link has inertial tag:
        //    -add the inertia of the link to his parent in the model
        // Otherwise do nothing.
        // In all cases:
        //    -let all the children become children of parent
        //    -inform the parser of the offset to apply
        //    -add fixed body in model to display it in gepetto-viewer
        if (link->inertial)
        {
          model.mergeFixedBody(parent, jointPlacement, Y); //Modify the parent inertia in the model
        }

        SE3 ptjot_se3 = convertFromUrdf(link->parent_joint->parent_to_joint_origin_transform);

        //transformation of the current placement offset
        nextPlacementOffset = placementOffset*ptjot_se3;

        //add the fixed Body in the model for the viewer
        model.addFixedBody(parent,nextPlacementOffset,link->name,visual);

        BOOST_FOREACH(::urdf::LinkPtr child_link,link->child_links)
        {
          child_link->setParent(link->getParent() );  //skip the fixed generation
        }
        break;
      }
      default:
      {
        std::cerr << "The joint type " << joint->type << " is not supported." << std::endl;
        assert(false && "Only revolute, prismatic and fixed joints are accepted." );
        break;
      }
    }
  }



  BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
  {
    parseTree(child, model, nextPlacementOffset);
  }
}

    template <typename D>
    void parseTree( ::urdf::LinkConstPtr link, Model & model, const SE3 & placementOffset , const JointModelBase<D> &  root_joint  )
    {
      const Inertia & Y = (link->inertial) ?
      convertFromUrdf(*link->inertial)
      : Inertia::Identity();
      model.addBody( 0, root_joint, placementOffset, Y , "root_joint", link->name, true );
      BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
      {
        parseTree(child, model, SE3::Identity());
      }
    }


    template <typename D>
    Model buildModel( const std::string & filename, const JointModelBase<D> &  root_joint )
    {
      Model model;

      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      if (urdfTree)
        parseTree(urdfTree->getRoot(), model, SE3::Identity(), root_joint);
      else
      {
        std::cerr << "The URDF tree seems to be empty" << std::endl; // In future, raise exception instead
      }
      
      return model;
    }

    Model buildModel( const std::string & filename)
    {
      Model model;

      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      if (urdfTree)
        parseTree(urdfTree->getRoot(), model, SE3::Identity());
      else
      {
        std::cerr << "The URDF tree seems to be empty" << std::endl; // In future, raise exception instead
      }

      return model;
    }

  } // namespace urdf
} // namespace se3

#endif // ifndef __se3_urdf_hpp__

