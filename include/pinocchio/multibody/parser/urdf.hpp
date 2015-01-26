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

    enum AxisCartesian { AXIS_X, AXIS_Y, AXIS_Z, AXIS_ERROR };
    AxisCartesian extractCartesianAxis( const ::urdf::Vector3 & axis )
    {
      if( (axis.x==1.0)&&(axis.y==0.0)&&(axis.z==0.0) )
	return AXIS_X;
      else if( (axis.x==0.0)&&(axis.y==1.0)&&(axis.z==0.0) )
	return AXIS_Y;
      else if( (axis.x==0.0)&&(axis.y==0.0)&&(axis.z==1.0) )
	return AXIS_Z;

      std::cerr << "Axis = (" <<axis.x<<","<<axis.y<<","<<axis.z<<")" << std::endl;
      assert( false && "Only cartesian axis are accepted." );
      return AXIS_ERROR;
    }

    void parseTree( ::urdf::LinkConstPtr link, Model & model, bool freeFlyer )
    {
      ::urdf::JointConstPtr joint = link->parent_joint;

      // std::cout << " *** " << link->name << "    < attached by joint ";
      // if(joint)
      //   std::cout << "#" << link->parent_joint->name << std::endl;
      // else std::cout << "###ROOT" << std::endl;
 
      //assert(link->inertial && "The parser cannot accept trivial mass");
      const Inertia & Y = (link->inertial) ?
	convertFromUrdf(*link->inertial)
	: Inertia::Identity();

      //std::cout << "Inertia: " << Y << std::endl;

      if(joint!=NULL)
	{
	  assert(link->getParent()!=NULL);
	  if(joint->type != ::urdf::Joint::FIXED)
	    {
	      // This is a bypass to be corrected later. TODO


	  Model::Index parent 
	    = (link->getParent()->parent_joint==NULL) ?
	    (freeFlyer ? 1 : 0)
	    : model.getBodyId( link->getParent()->parent_joint->name );
	  //std::cout << joint->name << " === " << parent << std::endl;

	  const SE3 & jointPlacement = convertFromUrdf(joint->parent_to_joint_origin_transform);

	  //std::cout << "Parent = " << parent << std::endl;
	  //std::cout << "Placement = " << (Matrix4)jointPlacement << std::endl;

	  switch(joint->type)
	    {
	    case ::urdf::Joint::REVOLUTE:
	    case ::urdf::Joint::CONTINUOUS: // Revolute with no joint limits
	      {
		AxisCartesian axis = extractCartesianAxis(joint->axis);
		switch(axis)
		  {
		  case AXIS_X:
		    model.addBody( parent, JointModelRX(), jointPlacement, Y, joint->name );
		    break;
		  case AXIS_Y:
		    model.addBody( parent, JointModelRY(), jointPlacement, Y, joint->name );
		    break;
		  case AXIS_Z:
		    model.addBody( parent, JointModelRZ(), jointPlacement, Y, joint->name );
		    break;
		  default:
		    std::cerr << "Bad axis = (" <<joint->axis.x<<","<<joint->axis.y
			      <<","<<joint->axis.z<<")" << std::endl;
		    assert(false && "Only X, Y or Z axis are accepted." );
		    break;
		  }
		break;
	      }
	    case ::urdf::Joint::FIXED:
	      {
		/* To fixed this, "spot" point should be added. TODO. */
		//std::cerr << "For now, fixed joint are not accepted. " << std::endl;
		break;
	      }
	    default:
	      {
		std::cerr << "The joint type " << joint->type << " is not supported." << std::endl;
		assert(false && "Only revolute joint are accepted." );
		break;
	      }
	    }
	    }}
      else if(freeFlyer)/* (joint==NULL) */
	{ /* The link is the root of the body. */
	  model.addBody( 0, JointModelFreeFlyer(), SE3::Identity(), Y, "root" );
	}

      BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
	{
	  parseTree( child,model,freeFlyer );
	}
    }  

    Model buildModel( const std::string & filename, bool freeFlyer = false )
    {
      Model model;

      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      parseTree(urdfTree->getRoot(),model,freeFlyer);
      return model;
    }

  } // namespace urdf
} // namespace se3

#endif // ifndef __se3_urdf_hpp__

