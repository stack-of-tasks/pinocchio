//
// Copyright (c) 2016 CNRS
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

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/utils.hpp"
#include "pinocchio/multibody/model.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <sstream>
#include <boost/foreach.hpp>
#include <limits>

namespace se3
{
  namespace urdf
  {
    namespace details
    {
      void appendBodyToJoint(Model& model, const Model::JointIndex jid,
                             const boost::shared_ptr< ::urdf::Inertial> Y,
                             const SE3 & placement,
                             const std::string & body_name)
      {
        if (Y == NULL) {
          model.addFrame(Frame(body_name, jid, placement, BODY));
        } else {
          model.appendBodyToJoint(jid, convertFromUrdf(*Y), placement, body_name);
        }
        assert (!model.inertias[jid].lever().hasNaN() && ! model.inertias[jid].inertia().data().hasNaN());
      }

      ///
      /// \brief Shortcut for adding a joint and directly append a body to it.
      ///
      template<typename JointModel>
      void addJointAndBody(Model & model, const JointModelBase<JointModel> & jmodel, const Model::JointIndex parent_id,
                           const SE3 & joint_placement, const std::string & joint_name,
                           const boost::shared_ptr< ::urdf::Inertial> Y,
                           const std::string & body_name,
                           const typename JointModel::TangentVector_t & max_effort = JointModel::TangentVector_t::Constant(std::numeric_limits<double>::max()),
                           const typename JointModel::TangentVector_t & max_velocity = JointModel::TangentVector_t::Constant(std::numeric_limits<double>::max()),
                           const typename JointModel::ConfigVector_t & min_config = JointModel::ConfigVector_t::Constant(-std::numeric_limits<double>::min()),
                           const typename JointModel::ConfigVector_t & max_config = JointModel::ConfigVector_t::Constant(std::numeric_limits<double>::max()))
      {
        Model::JointIndex idx;
        
        idx = model.addJoint(parent_id,jmodel,
                             joint_placement,joint_name,
                             max_effort,max_velocity,
                             min_config,max_config);
        
        appendBodyToJoint(model, idx, Y, SE3::Identity(), body_name);
      }
      
      ///
      /// \brief Handle the case of JointModelDense which is dynamic.
      ///
      void addJointAndBody(Model & model, const JointModelBase< JointModelDense<-1,-1> > & jmodel, const Model::JointIndex parent_id,
                           const SE3 & joint_placement, const std::string & joint_name,
                           const boost::shared_ptr< ::urdf::Inertial> Y,
                           const std::string & body_name)
      {
        Model::JointIndex idx;
        
        idx = model.addJoint(parent_id,jmodel,
                             joint_placement,joint_name);
        
        appendBodyToJoint(model, idx, Y, SE3::Identity(), body_name);
      }

      ///
      /// \brief Recursive procedure for reading the URDF tree.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current URDF link.
      /// \param[in] model The model where the link must be added.
      /// \param[in] placementOffset The relative placement of the link relative to the closer non fixed joint in the tree.
      ///
      void parseTree(::urdf::LinkConstPtr link, Model & model, const SE3 & placementOffset, bool verbose) throw (std::invalid_argument)
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
          std::ostringstream joint_info;
          
          // check if inertial information is provided
          if (!link->inertial && joint->type != ::urdf::Joint::FIXED)
          {
            const std::string exception_message (link->name + " - spatial inertial information missing.");
            throw std::invalid_argument(exception_message);
          }
          
          Model::JointIndex parent_joint_id = (link->getParent()->parent_joint==NULL)
            ? (model.existJointName("root_joint") ? model.getJointId("root_joint") : 0)
            : model.getJointId( link->getParent()->parent_joint->name );
          
          // Transformation from the parent link to the joint origin
          const SE3 & jointPlacement = placementOffset*convertFromUrdf(joint->parent_to_joint_origin_transform);
          
          const boost::shared_ptr< ::urdf::Inertial> Y = link->inertial;
         
          switch(joint->type)
          {
            case ::urdf::Joint::FLOATING:
            {
              joint_info << "joint FreeFlyer";
              addJointAndBody(model,JointModelFreeFlyer(),
                              parent_joint_id,jointPlacement,joint->name,
                              Y,link->name);
              
              break;
            }
            case ::urdf::Joint::REVOLUTE:
            {
              joint_info << "joint REVOLUTE with axis";
              
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
              //            else
              //              assert(false && "Joint bounds information missing.");
              
              CartesianAxis axis = extractCartesianAxis(joint->axis);
              
              switch(axis)
              {
                case AXIS_X:
                {
                  joint_info << " along X";
                  addJointAndBody(model,JointModelRX(),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                case AXIS_Y:
                {
                  joint_info << " along Y";
                  addJointAndBody(model,JointModelRY(),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                case AXIS_Z:
                {
                  joint_info << " along Z";
                  addJointAndBody(model,JointModelRZ(),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                case AXIS_UNALIGNED:
                {
                  SE3::Vector3 joint_axis(SE3::Vector3::Zero());
                  joint_axis << joint->axis.x,joint->axis.y,joint->axis.z;
                  joint_info << " unaligned along (" << joint_axis.transpose() << ")";
                  
                  addJointAndBody(model,JointModelRevoluteUnaligned(joint_axis.normalized()),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
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
                joint_info << "joint CONTINUOUS with axis";
                
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
                
                CartesianAxis axis = extractCartesianAxis(joint->axis);
                
                switch(axis)
                {
                  case AXIS_X:
                  {
                    joint_info << " along X";
                    addJointAndBody(model,JointModelRX(),
                                    parent_joint_id,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,
                                    lower_position, upper_position);
                    break;
                  }
                  case AXIS_Y:
                  {
                    joint_info << " along Y";
                    addJointAndBody(model,JointModelRY(),
                                    parent_joint_id,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,
                                    lower_position, upper_position);
                    break;
                  }
                  case AXIS_Z:
                  {
                    joint_info << " along Z";
                    addJointAndBody(model,JointModelRZ(),
                                    parent_joint_id,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,
                                    lower_position, upper_position);
                    break;
                  }
                  case AXIS_UNALIGNED:
                  {
                    SE3::Vector3 joint_axis(SE3::Vector3::Zero());
                    joint_axis << joint->axis.x,joint->axis.y,joint->axis.z;
                    joint_info << " unaligned along (" << joint_axis.transpose() << ")";
                    
                    addJointAndBody(model,JointModelRevoluteUnaligned(joint_axis.normalized()),
                                    parent_joint_id,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,
                                    lower_position, upper_position);
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
              joint_info << "joint PRISMATIC with axis";
              
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
              
              CartesianAxis axis = extractCartesianAxis(joint->axis);
              switch(axis)
              {
                case AXIS_X:
                {
                  joint_info << " along X";
                  addJointAndBody(model,JointModelPX(),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                case AXIS_Y:
                {
                  joint_info << " along Y";
                  addJointAndBody(model,JointModelPY(),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                case AXIS_Z:
                {
                  joint_info << " along Z";
                  addJointAndBody(model,JointModelPZ(),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                case AXIS_UNALIGNED:
                {
                  SE3::Vector3 joint_axis(SE3::Vector3::Zero());
                  joint_axis << joint->axis.x,joint->axis.y,joint->axis.z;
                  joint_info << " unaligned along (" << joint_axis.transpose() << ")";
                  
                  addJointAndBody(model,JointModelPrismaticUnaligned(joint_axis.normalized()),
                                  parent_joint_id,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
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
              joint_info << "joint PLANAR with normal axis along Z";
              
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
              
              addJointAndBody(model,JointModelPlanar(),
                              parent_joint_id,jointPlacement,joint->name,
                              Y,link->name,
                              max_effort,max_velocity,
                              lower_position, upper_position);
              
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
              
              joint_info << "fixed joint";
              appendBodyToJoint(model, parent_joint_id, Y, jointPlacement, link_name);
              
              //transformation of the current placement offset
              nextPlacementOffset = jointPlacement;
              
              // Add a frame in the model to keep trace of this fixed joint
              model.addFrame(Frame(joint->name, parent_joint_id, nextPlacementOffset, FIXED_JOINT));
              
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
            const Inertia YY = (Y==NULL) ? Inertia::Zero() : convertFromUrdf(*Y);
            std::cout << "Adding Body" << std::endl;
            std::cout << "\"" << link_name << "\" connected to " << "\"" << parent_link_name << "\" throw joint " << "\"" << joint_name << "\"" << std::endl;
            std::cout << "joint type: " << joint_info << std::endl;
            std::cout << "joint placement:\n" << jointPlacement;
            std::cout << "body info: " << std::endl;
            std::cout << "  " << "mass: " << YY.mass() << std::endl;
            std::cout << "  " << "lever: " << YY.lever().transpose() << std::endl;
            std::cout << "  " << "inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): " << YY.inertia().data().transpose() << std::endl << std::endl;
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

      ///
      /// \brief Parse a tree with a specific root joint linking the model to the environment.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current URDF link.
      /// \param[in] model The model where the link must be added.
      /// \param[in] verbose Print parsing info.
      ///
      void parseRootTree (::urdf::LinkConstPtr root_link, Model & model, const bool verbose) throw (std::invalid_argument)
      {
        BOOST_FOREACH(::urdf::LinkPtr child, root_link->child_links)
        {
          parseTree(child, model, SE3::Identity(), verbose);
        }

        // FIXME: check the inertias
        // The check in appendBodyToJoint only ensures inertias are not NaN
        // but pinocchio requires non-zero inertias.
        // Are zero inertias a problem for kinematic algorithm ?
      }

      ///
      /// \brief Parse a tree with a specific root joint linking the model to the environment.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current URDF link.
      /// \param[in] model The model where the link must be added.
      /// \param[in] root_joint The specific root joint.
      /// \param[in] verbose Print parsing info.
      ///
      template <typename D>
      void parseRootTree (::urdf::LinkConstPtr root_link, Model & model, const JointModelBase<D> & root_joint, const bool verbose) throw (std::invalid_argument)
      {
        addJointAndBody(model,root_joint,
            0,SE3::Identity(),"root_joint",
            root_link->inertial,root_link->name);

        BOOST_FOREACH(::urdf::LinkPtr child, root_link->child_links)
        {
          parseTree(child, model, SE3::Identity(), verbose);
        }

        // FIXME: See fixme in previous parseRootTree definition
      }
    } // namespace details
              
    ///
    /// \brief Call parse root tree templated function
    ///
    struct ParseRootTreeVisitor : public boost::static_visitor<>
    {
      ::urdf::LinkConstPtr m_root_link;
      Model & m_model;
      const bool m_verbose;
     
      ParseRootTreeVisitor(::urdf::LinkConstPtr root_link, Model & model, const bool verbose)
      : m_root_link(root_link)
      , m_model(model)
      , m_verbose(verbose)
      {}
      
      template<typename Derived>
      void operator()(const JointModelBase<Derived> & root_joint) const
      {
        details::parseRootTree(m_root_link,m_model,root_joint,m_verbose);
      }
      
      static void run(::urdf::LinkConstPtr root_link, Model & model, const JointModelVariant & root_joint, const bool verbose)
      {
        boost::apply_visitor(ParseRootTreeVisitor(root_link,model,verbose),root_joint);
      }
    }; // struct ParseRootTreeVisitor

    Model& buildModel(const std::string & filename, 
                      const JointModelVariant & root_joint, 
                      Model & model,
                      const bool verbose) 
      throw (std::invalid_argument)
    {
      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      if (urdfTree)
        ParseRootTreeVisitor::run(urdfTree->getRoot(),model,root_joint,verbose);
      else
      {
        const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
      }
      return model;
    }
                
    Model& buildModel(const std::string & filename, Model& model,const bool verbose) 
    throw (std::invalid_argument)
    {
      ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDFFile (filename);
      if (urdfTree)
        details::parseRootTree(urdfTree->getRoot(),model,verbose);
      else
      {
        const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
      }
      
      return model;
    }

  } // namespace urdf
} // namespace se3
