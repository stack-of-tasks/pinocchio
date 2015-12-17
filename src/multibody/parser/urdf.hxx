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

#ifndef __se3_urdf_hxx__
#define __se3_urdf_hxx__

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include "pinocchio/multibody/model.hpp"

#include <exception>

namespace se3
{
  namespace urdf
  {

    inline Inertia convertFromUrdf (const ::urdf::Inertial & Y)
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
    
    inline SE3 convertFromUrdf (const ::urdf::Pose & M)
    {
      const ::urdf::Vector3 & p = M.position;
      const ::urdf::Rotation & q = M.rotation;
      return SE3( Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix(), Eigen::Vector3d(p.x,p.y,p.z));
    }
    
    inline AxisCartesian extractCartesianAxis (const ::urdf::Vector3 & axis)
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
    
    
    inline void parseTree (::urdf::LinkConstPtr link, Model & model, const SE3 & placementOffset, bool verbose) throw (std::invalid_argument)
    {

      // Parent joint of the current body
      ::urdf::JointConstPtr joint = link->parent_joint;
      
      // OffSet of the next link. In case we encounter a fixed joint, we need to propagate the offset placement of its attached body to next joint.
      SE3 nextPlacementOffset = SE3::Identity();
      
      if(joint != NULL) // if the link is not the root of the tree
      {
        assert(link->getParent()!=NULL);

        const std::string & joint_name = joint->name;
        const std::string & link_name = link->name;
        const std::string & parent_link_name = link->getParent()->name;
        std::string joint_info = "";

        // check if inertial information is provided
        if (!link->inertial && joint->type != ::urdf::Joint::FIXED)
        {
          const std::string exception_message (link->name + " - spatial inertial information missing.");
          throw std::invalid_argument(exception_message);
        }

        Model::Index parent_joint_id = (link->getParent()->parent_joint==NULL) ? (model.existJointName("root_joint") ? model.getJointId("root_joint") : 0) :
        model.getJointId( link->getParent()->parent_joint->name );

        // Transformation from the parent link to the joint origin
        const SE3 & jointPlacement = placementOffset*convertFromUrdf(joint->parent_to_joint_origin_transform);

        const Inertia & Y = (link->inertial) ? convertFromUrdf(*link->inertial) :
        Inertia::Zero();

        const bool has_visual = (link->visual) ? true : false;

        switch(joint->type)
        {
          case ::urdf::Joint::FLOATING:
          {
            joint_info = "joint FreeFlyer";

            typedef JointModelFreeFlyer::ConfigVector_t ConfigVector_t;
            typedef JointModelFreeFlyer::TangentVector_t TangentVector_t;
            typedef ConfigVector_t::Scalar Scalar_t;


            ConfigVector_t lower_position;
            lower_position.fill(std::numeric_limits<Scalar_t>::min());

            ConfigVector_t upper_position;
            upper_position.fill(std::numeric_limits<Scalar_t>::max());

            TangentVector_t max_effort;
            max_effort.fill(std::numeric_limits<Scalar_t>::max());

            TangentVector_t max_velocity;
            max_velocity.fill(std::numeric_limits<Scalar_t>::max());

            model.addBody(parent_joint_id, JointModelFreeFlyer(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name, link->name, has_visual);
            break;
          }
          case ::urdf::Joint::REVOLUTE:
          {
            joint_info = "joint REVOLUTE with axis";

            typedef JointModelRX::ConfigVector_t ConfigVector_t;
            typedef JointModelRX::TangentVector_t TangentVector_t;

            TangentVector_t max_effort;
            TangentVector_t max_velocity;
            ConfigVector_t lower_position;
            ConfigVector_t upper_position;

            if (joint->limits)
            {
              max_effort << joint->limits->effort;
              max_velocity << joint->limits->velocity;
              lower_position << joint->limits->lower;
              upper_position << joint->limits->upper;
            }

            Eigen::Vector3d jointAxis(Eigen::Vector3d::Zero());
            AxisCartesian axis = extractCartesianAxis(joint->axis);

            switch(axis)
            {
              case AXIS_X:
              joint_info += " along X";
              model.addBody(  parent_joint_id, JointModelRX(), jointPlacement, Y,
                max_effort, max_velocity, lower_position, upper_position,
                joint->name,link->name, has_visual );
              break;
              case AXIS_Y:
              joint_info += " along Y";
              model.addBody(  parent_joint_id, JointModelRY(), jointPlacement, Y,
                max_effort, max_velocity, lower_position, upper_position,
                joint->name,link->name, has_visual );
              break;
              case AXIS_Z:
              joint_info += " along Z";
              model.addBody(  parent_joint_id, JointModelRZ(), jointPlacement, Y,
                max_effort, max_velocity, lower_position, upper_position,
                joint->name,link->name, has_visual );
              break;
              case AXIS_UNALIGNED:
              {
                std::stringstream axis_value;
                axis_value << std::setprecision(5);
                axis_value << "(" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")";
                joint_info += " unaligned " + axis_value.str();

                jointAxis = Eigen::Vector3d( joint->axis.x,joint->axis.y,joint->axis.z );
                jointAxis.normalize();
                model.addBody(  parent_joint_id, JointModelRevoluteUnaligned(jointAxis),
                  jointPlacement, Y,
                  max_effort, max_velocity, lower_position, upper_position,
                  joint->name,link->name, has_visual );
                break;
              }
              default:
              assert( false && "Fatal Error while extracting revolute joint axis");
              break;
            }
            break;
          }
        case ::urdf::Joint::CONTINUOUS: // Revolute with no joint limits
        {
          joint_info = "joint CONTINUOUS with axis";
          
          typedef JointModelRX::ConfigVector_t ConfigVector_t;
          typedef JointModelRX::TangentVector_t TangentVector_t;
          
          TangentVector_t max_effort;
          TangentVector_t max_velocity;
          ConfigVector_t lower_position;
          ConfigVector_t upper_position;
          
          if (joint->limits)
          {
            max_effort << joint->limits->effort;
            max_velocity << joint->limits->velocity;
            lower_position << joint->limits->lower;
            upper_position << joint->limits->upper;
          }
          
          Eigen::Vector3d jointAxis(Eigen::Vector3d::Zero());
          AxisCartesian axis = extractCartesianAxis(joint->axis);
          
          switch(axis)
          {
            case AXIS_X:
            joint_info += " along X";
            model.addBody(  parent_joint_id, JointModelRX(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name,link->name, has_visual );
            break;
            case AXIS_Y:
            joint_info += " along Y";
            model.addBody(  parent_joint_id, JointModelRY(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name,link->name, has_visual );
            break;
            case AXIS_Z:
            joint_info += " along Z";
            model.addBody(  parent_joint_id, JointModelRZ(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name,link->name, has_visual );
            break;
            case AXIS_UNALIGNED:
            {
              std::stringstream axis_value;
              axis_value << std::setprecision(5);
              axis_value << "(" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")";
              joint_info += " unaligned " + axis_value.str();
              
              jointAxis = Eigen::Vector3d( joint->axis.x,joint->axis.y,joint->axis.z );
              jointAxis.normalize();
              model.addBody(  parent_joint_id, JointModelRevoluteUnaligned(jointAxis),
                jointPlacement, Y,
                max_effort, max_velocity, lower_position, upper_position,
                joint->name,link->name, has_visual );
              break;
            }
            default:
            assert( false && "Fatal Error while extracting revolute joint axis");
            break;
          }
          break;
        }
        case ::urdf::Joint::PRISMATIC:
        {
          joint_info = "joint PRISMATIC with axis";
          
          typedef JointModelRX::ConfigVector_t ConfigVector_t;
          typedef JointModelRX::TangentVector_t TangentVector_t;
          
          TangentVector_t max_effort;
          TangentVector_t max_velocity;
          ConfigVector_t lower_position;
          ConfigVector_t upper_position;
          
          if (joint->limits)
          {
            max_effort << joint->limits->effort;
            max_velocity << joint->limits->velocity;
            lower_position << joint->limits->lower;
            upper_position << joint->limits->upper;
          }
          
          AxisCartesian axis = extractCartesianAxis(joint->axis);
          switch(axis)
          {
            case AXIS_X:
            joint_info += " along X";
            model.addBody(  parent_joint_id, JointModelPX(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name,link->name, has_visual );
            break;
            case AXIS_Y:
            joint_info += " along Y";
            model.addBody(  parent_joint_id, JointModelPY(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name,link->name, has_visual );
            break;
            case AXIS_Z:
            joint_info += " along Z";
            model.addBody(  parent_joint_id, JointModelPZ(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name,link->name, has_visual );
            break;
            case AXIS_UNALIGNED:
            {
              std::stringstream axis_value;
              axis_value << std::setprecision(5);
              axis_value << "(" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")";
              joint_info += " unaligned " + axis_value.str();
              std::cerr << "Bad axis = " << axis_value << std::endl;
              assert(false && "Only X, Y or Z axis are accepted." );
              break;
            }
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

          joint_info = "fixed joint";
          if (link->inertial)
          {
            model.mergeFixedBody(parent_joint_id, jointPlacement, Y); //Modify the parent inertia in the model
          }
          
          //transformation of the current placement offset
          nextPlacementOffset = jointPlacement;
          
          //add the fixed Body in the model for the viewer
          model.addFixedBody(parent_joint_id, nextPlacementOffset, link->name, has_visual);
          
          //for the children of the current link, set their parent to be
          //the the parent of the current link.
          BOOST_FOREACH(::urdf::LinkPtr child_link, link->child_links)
          {
            child_link->setParent(link->getParent() );
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
      
      if (verbose)
      {
        std::cout << "Adding Body" << std::endl;
        std::cout << "\"" << link_name << "\" connected to " << "\"" << parent_link_name << "\" throw joint " << "\"" << joint_name << "\"" << std::endl;
        std::cout << "joint type: " << joint_info << std::endl;
        std::cout << "joint placement:\n" << jointPlacement;
        std::cout << "body info: " << std::endl;
        std::cout << "  " << "mass: " << Y.mass() << std::endl;
        std::cout << "  " << "lever: " << Y.lever().transpose() << std::endl;
        std::cout << "  " << "inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): " << Y.inertia().data().transpose() << std::endl << std::endl;
      }
      
    }
    else if (link->getParent() != NULL)
    {
      const std::string exception_message (link->name + " - joint information missing.");
      throw std::invalid_argument(exception_message);
    }
    
    
    BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
    {
      parseTree(child, model, nextPlacementOffset, verbose);
    }
  }
  
  
            template <typename D>
  void parseTree (::urdf::LinkConstPtr link, Model & model, const SE3 & placementOffset, const JointModelBase<D> & root_joint, const bool verbose) throw (std::invalid_argument)
  {
    if (!link->inertial)
    {
      const std::string exception_message (link->name + " - spatial inertia information missing.");
      throw std::invalid_argument(exception_message);
    }
    
    const Inertia & Y = convertFromUrdf(*link->inertial);
    const bool has_visual = (link->visual) ? true : false;
    model.addBody(0, root_joint, placementOffset, Y , "root_joint", link->name, has_visual);
    
    BOOST_FOREACH(::urdf::LinkConstPtr child,link->child_links)
    {
      parseTree(child, model, SE3::Identity(), verbose);
    }
  }
  
  
  template <typename D>
  Model buildModel (const std::string & filename, const JointModelBase<D> & root_joint, bool verbose) throw (std::invalid_argument)
  {
    Model model;
    
    ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
    if (urdfTree)
      parseTree(urdfTree->getRoot(), model, SE3::Identity(), root_joint, verbose);
    else
    {
      const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
      throw std::invalid_argument(exception_message);
    }
    
    return model;
  }
  
  
  inline Model buildModel (const std::string & filename, const bool verbose) throw (std::invalid_argument)
  {
    Model model;
    
    ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
    if (urdfTree)
      parseTree(urdfTree->getRoot(), model, SE3::Identity(), verbose);
    else
    {
      const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
      throw std::invalid_argument(exception_message);
    }
    
    return model;
  }

  } // namespace urdf
  
} // namespace se3

#endif // ifndef __se3_urdf_hxx__
