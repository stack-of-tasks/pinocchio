//
// Copyright (c) 2015-2016 CNRS
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

/// @cond DEV

namespace se3
{
  namespace urdf
  {

    ///
    /// \brief Convert URDF Inertial quantity to Spatial Inertia.
    ///
    /// \param[in] Y The input URDF Inertia.
    ///
    /// \return The converted Spatial Inertia se3::Inertia.
    ///
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
    
    ///
    /// \brief Convert URDF Pose quantity to SE3.
    ///
    /// \param[in] M The input URDF Pose.
    ///
    /// \return The converted pose/transform se3::SE3.
    ///
    inline SE3 convertFromUrdf (const ::urdf::Pose & M)
    {
      const ::urdf::Vector3 & p = M.position;
      const ::urdf::Rotation & q = M.rotation;
      return SE3( Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix(), Eigen::Vector3d(p.x,p.y,p.z));
    }
    
    ///
    /// \brief The four possible cartesian types of an 3D axis.
    ///
    enum AxisCartesian { AXIS_X, AXIS_Y, AXIS_Z, AXIS_UNALIGNED };
    
    ///
    /// \brief Extract the cartesian property of a particular 3D axis.
    ///
    /// \param[in] axis The input URDF axis.
    ///
    /// \return The property of the particular axis se3::urdf::AxisCartesian.
    ///
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
    
    ///
    /// \brief Recursive procedure for reading the URDF tree.
    ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
    ///
    /// \param[in] link The current URDF link.
    /// \param[in] model The model where the link must be added.
    /// \param[in] placementOffset The relative placement of the link relative to the closer non fixed joint in the tree.
    ///
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

        Model::JointIndex parent_joint_id = (link->getParent()->parent_joint==NULL) ? (model.existJointName("root_joint") ? model.getJointId("root_joint") : 0) :
        model.getJointId( link->getParent()->parent_joint->name );

        // Transformation from the parent link to the joint origin
        const SE3 & jointPlacement = placementOffset*convertFromUrdf(joint->parent_to_joint_origin_transform);

        const Inertia & Y = (link->inertial) ? convertFromUrdf(*link->inertial) :
        Inertia::Zero();


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

            model.addJointAndBody(parent_joint_id, JointModelFreeFlyer(), jointPlacement, Y,
              max_effort, max_velocity, lower_position, upper_position,
              joint->name, link->name);
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
              {
                joint_info += " along X";
                model.addJointAndBody(parent_joint_id, JointModelRX(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_Y:
              {
                joint_info += " along Y";
                model.addJointAndBody(parent_joint_id, JointModelRY(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_Z:
              {
                joint_info += " along Z";
                model.addJointAndBody(parent_joint_id, JointModelRZ(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_UNALIGNED:
              {
                std::stringstream axis_value;
                axis_value << std::setprecision(5);
                axis_value << "(" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")";
                joint_info += " unaligned " + axis_value.str();
                
                jointAxis = Eigen::Vector3d( joint->axis.x,joint->axis.y,joint->axis.z );
                jointAxis.normalize();
                model.addJointAndBody( parent_joint_id, JointModelRevoluteUnaligned(jointAxis),
                                      jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              default:
              {
                assert(false && "The axis type of the revolute joint is of wrong type.");
                break;
              }
            }
            break;
          }
          case ::urdf::Joint::CONTINUOUS: // Revolute joint with no joint limits
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
              {
                joint_info += " along X";
                model.addJointAndBody(parent_joint_id, JointModelRX(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_Y:
              {
                joint_info += " along Y";
                model.addJointAndBody(parent_joint_id, JointModelRY(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_Z:
              {
                joint_info += " along Z";
                model.addJointAndBody(parent_joint_id, JointModelRZ(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_UNALIGNED:
              {
                std::stringstream axis_value;
                axis_value << std::setprecision(5);
                axis_value << "(" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")";
                joint_info += " unaligned " + axis_value.str();
                
                jointAxis = Eigen::Vector3d( joint->axis.x,joint->axis.y,joint->axis.z );
                jointAxis.normalize();
                model.addJointAndBody(  parent_joint_id, JointModelRevoluteUnaligned(jointAxis),
                  jointPlacement, Y,
                  max_effort, max_velocity, lower_position, upper_position,
                  joint->name,link->name );
                break;
              }
              default:
              {
                assert(false && "The axis type of the revolute joint is of wrong type.");
                break;
              }
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
              {
                joint_info += " along X";
                model.addJointAndBody(parent_joint_id, JointModelPX(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_Y:
              {
                joint_info += " along Y";
                model.addJointAndBody(parent_joint_id, JointModelPY(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name);
                break;
              }
              case AXIS_Z:
              {
                joint_info += " along Z";
                model.addJointAndBody(parent_joint_id, JointModelPZ(), jointPlacement, Y,
                                      max_effort, max_velocity, lower_position, upper_position,
                                      joint->name,link->name );
                break;
              }
              case AXIS_UNALIGNED:
              {
                std::stringstream axis_value;
                axis_value << std::setprecision(5);
                axis_value << "(" << joint->axis.x <<"," << joint->axis.y << "," << joint->axis.z << ")";
                joint_info += " unaligned " + axis_value.str();
      
                Eigen::Vector3d jointAxis(Eigen::Vector3d(joint->axis.x,joint->axis.y,joint->axis.z));
                jointAxis.normalize();
                model.addJointAndBody(parent_joint_id, JointModelPrismaticUnaligned(jointAxis),
                              jointPlacement, Y,
                              max_effort, max_velocity, lower_position, upper_position,
                              joint->name,link->name);
                
                break;
              }
              default:
              {
                assert(false && "The axis type of the prismatic joint is of wrong type.");
                break;
              }
            }
            break;
          }
          case ::urdf::Joint::PLANAR:
          {
            joint_info = "joint PLANAR with normal axis along Z";
            
            typedef JointModelPlanar::ConfigVector_t ConfigVector_t;
            typedef JointModelPlanar::TangentVector_t TangentVector_t;
            
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
            
            model.addJointAndBody(parent_joint_id, JointModelPlanar(), jointPlacement, Y,
                                  max_effort, max_velocity, lower_position, upper_position,
                                  joint->name,link->name );

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
              model.appendBodyToJoint(parent_joint_id, jointPlacement, Y, link->name); //Modify the parent inertia in the model
            }
            
            //transformation of the current placement offset
            nextPlacementOffset = jointPlacement;

            // Add a frame in the model to keep trace of this fixed joint
            model.addFrame(joint->name, parent_joint_id, nextPlacementOffset, FIXED_JOINT);
            
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
            const std::string exception_message ("The type of joint " + joint_name + " is not supported.");
            throw std::invalid_argument(exception_message);
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
  void parseRootTree (::urdf::LinkConstPtr root_link, Model & model, const JointModelBase<D> & root_joint, const bool verbose) throw (std::invalid_argument)
  {
    // If the root link has no inertial info (because it is a base_link for example),
    // we have to merge its inertial info with all its children connected to it with fixed joints
    if (!root_link->inertial)
    {
      // If the root link has only one child
      if (root_link->child_links.size() == 1)
      {
        ::urdf::LinkPtr child_link = root_link->child_links[0];
        assert(child_link->inertial != NULL && "Inertial information missing for parsing the root link.");
        const Inertia & Y = convertFromUrdf(*child_link->inertial);
        model.addJointAndBody(0, root_joint, SE3::Identity(), Y, "root_joint", child_link->name);
      
        // Change the name of the parent joint in the URDF tree
        child_link->parent_joint->name = "root_joint";
      
        BOOST_FOREACH(::urdf::LinkConstPtr child, child_link->child_links)
        {
          parseTree(child, model, SE3::Identity(), verbose);
        }
      }
      else
      {
        const std::string exception_message (root_link->name + " has no inertial information and has more than one child link. It corresponds to a disjoint tree.");
        throw std::invalid_argument(exception_message);
      }
    
    }
    else // otherwise, it is a plain body with inertial info. It processes as usual.
    {
      const Inertia & Y = convertFromUrdf(*root_link->inertial);
      model.addJointAndBody(0, root_joint, SE3::Identity(), Y , "root_joint", root_link->name);
    
      BOOST_FOREACH(::urdf::LinkPtr child, root_link->child_links)
      {
        parseTree(child, model, SE3::Identity(), verbose);
      }
    }
  
  }
  
  
  template <typename D>
  Model buildModel (const std::string & filename, const JointModelBase<D> & root_joint, bool verbose) throw (std::invalid_argument)
  {
    Model model;
    
    ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
    if (urdfTree)
      parseRootTree(urdfTree->getRoot(), model, root_joint, verbose);
    else
    {
      const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
      throw std::invalid_argument(exception_message);
    }
    
    return model;
  }
              
  void parseRootTree (::urdf::LinkConstPtr root_link, Model & model, const bool verbose) throw (std::invalid_argument)
  {
    // If the root link has no inertial info, it may be because it is a base_link.
    // In this case, we look for its child links which indeed contribute to the dynamics, they are not fixed to the universe.
    // TODO: it may be necessary to compute joint placement variable instead of setting it to SE3::Identity()
    if (!root_link->inertial)
    {
      typedef std::vector< ::urdf::LinkSharedPtr> LinkSharedPtrVector_t;
      LinkSharedPtrVector_t movable_child_links;
      LinkSharedPtrVector_t direct_child_links(root_link->child_links);
      LinkSharedPtrVector_t next_direct_child_links; // to visit
      LinkSharedPtrVector_t pathologic_child_links;
      do
      {
        next_direct_child_links.clear();
        BOOST_FOREACH(::urdf::LinkPtr child, direct_child_links)
        {
          if (child->parent_joint->type != ::urdf::Joint::FIXED)
            movable_child_links.push_back(child);
          else
          {
            if (child->inertial)
              pathologic_child_links.push_back(child);
            next_direct_child_links.insert (next_direct_child_links.end(), child->child_links.begin(), child->child_links.end());
          }
          
        }
        direct_child_links = next_direct_child_links;
      }
      while (!direct_child_links.empty());
      
      if (!pathologic_child_links.empty())
      {
        std::cout << "[INFO] The links:" << std::endl;
        for (LinkSharedPtrVector_t::iterator it = pathologic_child_links.begin();
             it < pathologic_child_links.end(); ++it)
        {
          std::cout << "  - " << (*it)->name << std::endl;
        }
        std::cout << "are fixed regarding to the universe (base_link) and convey inertial info. They won't affect the dynamics of the output model. Maybe, a root joint is missing connecting this links to the universe." << std::endl;
        
      }
      
      BOOST_FOREACH(::urdf::LinkPtr child, movable_child_links)
      {
        child->getParent()->parent_joint->name = model.names[0];
        parseTree(child, model, SE3::Identity(), verbose);
      }
      
    }
    else // Otherwise, we have to rase an exception because the first link will no be added to the model.
         // It seems a root joint is missing.
    {
      std::cout << "[INFO] The root link " << root_link->name << " of the model tree contains inertial information. It seems that a root joint is missing connecting this root link to the universe. The root link won't affect the dynamics of the model." << std::endl;
      
      BOOST_FOREACH(::urdf::LinkPtr child, root_link->child_links)
      {
        parseTree(child, model, SE3::Identity(), verbose);
      }
    }
    
    
  }
  
  
  inline Model buildModel (const std::string & filename, const bool verbose) throw (std::invalid_argument)
  {
    Model model;
    
    ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
    if (urdfTree)
      parseRootTree(urdfTree->getRoot(),model,verbose);
    else
    {
      const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
      throw std::invalid_argument(exception_message);
    }
    
    return model;
  }

  } // namespace urdf
  
} // namespace se3
              
/// @endcond

#endif // ifndef __se3_urdf_hxx__
