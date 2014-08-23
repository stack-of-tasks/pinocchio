#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pinocchio/multibody/model.hpp"

namespace urdf
{
  typedef boost::shared_ptr<ModelInterface> ModelInterfacePtr;
  typedef boost::shared_ptr<const Joint> JointConstPtr;
  typedef boost::shared_ptr<const Link> LinkConstPtr;
  typedef boost::shared_ptr<Link> LinkPtr;

  typedef boost::shared_ptr<const Inertial> InertialConstPtr;
}

se3::Inertia convertFromUrdf( const urdf::Inertial& Y )
{
  const urdf::Vector3 & p = Y.origin.position;
  const urdf::Rotation & q = Y.origin.rotation;

  const Eigen::Vector3d com(p.x,p.y,p.z);
  const Eigen::Matrix3d & R = Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix();

  Eigen::Matrix3d I; I << Y.ixx,Y.ixy,Y.ixz
		       ,  Y.ixy,Y.iyy,Y.iyz
		       ,  Y.ixz,Y.iyz,Y.izz;

  return se3::Inertia(Y.mass,com,R*I*R.transpose());
}
se3::SE3 convertFromUrdf( const urdf::Pose & M )
{
  const urdf::Vector3 & p = M.position;
  const urdf::Rotation & q = M.rotation;
  return se3::SE3( Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix(), Eigen::Vector3d(p.x,p.y,p.z));
}

enum AxisCartesian { AXIS_X, AXIS_Y, AXIS_Z };
AxisCartesian extractCartesianAxis( const urdf::Vector3 & axis )
{
  if( (axis.x==1.0)&&(axis.y==0.0)&&(axis.z==0.0) )
    return AXIS_X;
  else if( (axis.x==0.0)&&(axis.y==1.0)&&(axis.z==0.0) )
    return AXIS_Y;
  else if( (axis.x==0.0)&&(axis.y==0.0)&&(axis.z==1.0) )
    return AXIS_Z;

  std::cerr << "Axis = (" <<axis.x<<","<<axis.y<<","<<axis.z<<")" << std::endl;
  assert( false && "Only cartesian axis are accepted." );
}


void parseTree( urdf::LinkConstPtr link, const urdf::ModelInterfacePtr & model, se3::Model & multibody )
{
  urdf::JointConstPtr joint = link->parent_joint;

  std::cout << " *** " << link->name << "    < attached by joint ";
  if(joint)
    std::cout << "#" << link->parent_joint->name << std::endl;
  else std::cout << "###ROOT" << std::endl;
 
  assert(link->inertial && "The parser cannot accept trivial mass");
  const se3::Inertia & Y = convertFromUrdf(*link->inertial);
  //std::cout << "Inertia: " << Y << std::endl;

  if(joint!=NULL)
    {
      assert(link->getParent()!=NULL);
      se3::Model::Index parent 
	= (link->getParent()->parent_joint==NULL) ?
	1 : multibody.getBodyId( link->getParent()->parent_joint->name );
      const se3::SE3 & jointPlacement = convertFromUrdf(joint->parent_to_joint_origin_transform);

      std::cout << "Parent = " << parent << std::endl;
      std::cout << "Placement = " << jointPlacement << std::endl;

      switch(joint->type)
	{
	case urdf::Joint::REVOLUTE:
	case urdf::Joint::CONTINUOUS: // Revolute with no joint limits
	  {
	    AxisCartesian axis = extractCartesianAxis(joint->axis);
	    switch(axis)
	      {
	      case AXIS_X:
		multibody.addBody( parent, se3::JointModelRX(), jointPlacement, Y, joint->name );
		break;
	      default:
		std::cerr << "Axis = (" <<joint->axis.x<<","<<joint->axis.y
			  <<","<<joint->axis.z<<")" << std::endl;
		assert(false && "Only X axis are accepted." );
	      }
	    break;
	  }
	default:
	  std::cerr << "The joint type " << joint->type << " is not supported." << std::endl;
	}
    }
  else // (joint==NULL)
    { // The link is the root of the body.
      std::cout << "Parent = 0 (universe)" << std::endl;
      multibody.addBody( 0, se3::JointModelFreeFlyer(), se3::SE3::Identity(), Y, "root" );
    }


  BOOST_FOREACH(urdf::LinkConstPtr child,link->child_links)
    {
      parseTree( child,model,multibody );
    }
}  


void buildModel( const std::string & filename )
{
  urdf::ModelInterfacePtr model = urdf::parseURDFFile (filename);
  urdf::LinkConstPtr root = model->getRoot();
  se3::Model multibody;

  parseTree(root,model,multibody);
}

int main(int argc, const char**argv)
{
  std::string filename = "/home/nmansard/src/rbdl/rbdl_evaluate_performances/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];

  buildModel(filename);

  return 0;
}
