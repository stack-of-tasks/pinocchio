//
// Copyright (c) 2015-2020 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/utils.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <sstream>
#include <boost/foreach.hpp>
#include <limits>

namespace pinocchio
{
  namespace urdf
  {
    namespace details
    {
      ///
      /// \brief Convert URDF Inertial quantity to Spatial Inertia.
      ///
      /// \param[in] Y The input URDF Inertia.
      ///
      /// \return The converted Spatial Inertia pinocchio::Inertia.
      ///
      static Inertia convertFromUrdf(const ::urdf::Inertial & Y)
      {
        const ::urdf::Vector3 & p = Y.origin.position;
        const ::urdf::Rotation & q = Y.origin.rotation;

        const Inertia::Vector3 com(p.x,p.y,p.z);
        const Inertia::Matrix3 & R = Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix();

        Inertia::Matrix3 I;
        I << Y.ixx,Y.ixy,Y.ixz,
             Y.ixy,Y.iyy,Y.iyz,
             Y.ixz,Y.iyz,Y.izz;
        return Inertia(Y.mass,com,R*I*R.transpose());
      }

      static Inertia convertFromUrdf(const ::urdf::InertialSharedPtr & Y)
      {
        if (Y) return convertFromUrdf(*Y);
        return Inertia::Zero();
      }

      static FrameIndex getParentLinkFrame(const ::urdf::LinkConstSharedPtr link,
                                           UrdfVisitorBase & model)
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(link && link->getParent());
        FrameIndex id = model.getBodyId(link->getParent()->name);
        return id;
      }

      ///
      /// \brief Recursive procedure for reading the URDF tree.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current URDF link.
      /// \param[in] model The model where the link must be added.
      ///
      void parseTree(::urdf::LinkConstSharedPtr link,
                     UrdfVisitorBase & model)
      {
        typedef UrdfVisitorBase::Scalar Scalar;
        typedef UrdfVisitorBase::SE3 SE3;
        typedef UrdfVisitorBase::Vector Vector;
        typedef UrdfVisitorBase::Vector3 Vector3;
        typedef Model::FrameIndex FrameIndex;

        // Parent joint of the current body
        const ::urdf::JointConstSharedPtr joint =
        ::urdf::const_pointer_cast< ::urdf::Joint>(link->parent_joint);

        if(joint) // if the link is not the root of the tree
        {
          PINOCCHIO_CHECK_INPUT_ARGUMENT(link->getParent());

          const std::string & joint_name = joint->name;
          const std::string & link_name = link->name;
          const std::string & parent_link_name = link->getParent()->name;
          std::ostringstream joint_info;

          FrameIndex parentFrameId = getParentLinkFrame(link, model);

          // Transformation from the parent link to the joint origin
          const SE3 jointPlacement
          = convertFromUrdf(joint->parent_to_joint_origin_transform);

          const Inertia Y = convertFromUrdf(link->inertial);

          Vector max_effort(1), max_velocity(1), min_config(1), max_config(1);
          Vector friction(Vector::Constant(1,0.)), damping(Vector::Constant(1,0.));
          Vector3 axis (joint->axis.x, joint->axis.y, joint->axis.z);

          const Scalar infty = std::numeric_limits<Scalar>::infinity();

          switch(joint->type)
          {
            case ::urdf::Joint::FLOATING:
              joint_info << "joint FreeFlyer";

              max_effort   = Vector::Constant(6, infty);
              max_velocity = Vector::Constant(6, infty);
              min_config   = Vector::Constant(7,-infty);
              max_config   = Vector::Constant(7, infty);
              min_config.tail<4>().setConstant(-1.01);
              max_config.tail<4>().setConstant( 1.01);
              
              friction = Vector::Constant(6, 0.);
              damping = Vector::Constant(6, 0.);

              model.addJointAndBody(UrdfVisitorBase::FLOATING, axis,
                                    parentFrameId,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,min_config,max_config,
                                    friction,damping);
              break;

            case ::urdf::Joint::REVOLUTE:
              joint_info << "joint REVOLUTE with axis";

              // TODO I think the URDF standard forbids REVOLUTE with no limits.
              assert(joint->limits);
              if(joint->limits)
              {
                max_effort << joint->limits->effort;
                max_velocity << joint->limits->velocity;
                min_config << joint->limits->lower;
                max_config << joint->limits->upper;
              }
              
              if(joint->dynamics)
              {
                friction << joint->dynamics->friction;
                damping << joint->dynamics->damping;
              }

              model.addJointAndBody(UrdfVisitorBase::REVOLUTE, axis,
                                    parentFrameId,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,min_config,max_config,
                                    friction,damping);
              break;

            case ::urdf::Joint::CONTINUOUS: // Revolute joint with no joint limits
              joint_info << "joint CONTINUOUS with axis";

              min_config.resize(2);
              max_config.resize(2);
              min_config << -1.01, -1.01;
              max_config <<  1.01,  1.01;

              if(joint->limits)
              {
                max_effort << joint->limits->effort;
                max_velocity << joint->limits->velocity;
              }
              else
              {
                max_effort << infty;
                max_velocity << infty;
              }
              
              if(joint->dynamics)
              {
                friction << joint->dynamics->friction;
                damping << joint->dynamics->damping;
              }

              model.addJointAndBody(UrdfVisitorBase::CONTINUOUS, axis,
                                    parentFrameId,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,min_config,max_config,
                                    friction,damping);
              break;

            case ::urdf::Joint::PRISMATIC:
              joint_info << "joint PRISMATIC with axis";

              // TODO I think the URDF standard forbids REVOLUTE with no limits.
              assert(joint->limits);
              if (joint->limits)
              {
                max_effort << joint->limits->effort;
                max_velocity << joint->limits->velocity;
                min_config << joint->limits->lower;
                max_config << joint->limits->upper;
              }
              
              if(joint->dynamics)
              {
                friction << joint->dynamics->friction;
                damping << joint->dynamics->damping;
              }

              model.addJointAndBody(UrdfVisitorBase::PRISMATIC, axis,
                                    parentFrameId,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,min_config,max_config,
                                    friction,damping);
              break;

            case ::urdf::Joint::PLANAR:
              joint_info << "joint PLANAR with normal axis along Z";

              max_effort   = Vector::Constant(3, infty);
              max_velocity = Vector::Constant(3, infty);
              min_config   = Vector::Constant(4,-infty);
              max_config   = Vector::Constant(4, infty);
              min_config.tail<2>().setConstant(-1.01);
              max_config.tail<2>().setConstant( 1.01);
              
              friction = Vector::Constant(3, 0.);
              damping = Vector::Constant(3, 0.);

              model.addJointAndBody(UrdfVisitorBase::PLANAR, axis,
                                    parentFrameId,jointPlacement,joint->name,
                                    Y,link->name,
                                    max_effort,max_velocity,min_config,max_config,
                                    friction,damping);
              break;

            case ::urdf::Joint::FIXED:
              // In case of fixed joint, if link has inertial tag:
              //    -add the inertia of the link to his parent in the model
              // Otherwise do nothing.
              // In all cases:
              //    -let all the children become children of parent
              //    -inform the parser of the offset to apply
              //    -add fixed body in model to display it in gepetto-viewer
              
              joint_info << "fixed joint";
              model.addFixedJointAndBody(parentFrameId, jointPlacement,
                  joint_name, Y, link_name);
              break;

            default:
              throw std::invalid_argument("The type of joint " + joint_name + " is not supported.");
              break;
          }

          model
            << "Adding Body" << '\n'
            << '\"' << link_name << "\" connected to \"" << parent_link_name << "\" through joint \"" << joint_name << "\"\n"
            << "joint type: " << joint_info.str() << '\n'
            << "joint placement:\n" << jointPlacement << '\n'
            << "body info: " << '\n'
            << "  mass: " << Y.mass() << '\n'
            << "  lever: " << Y.lever().transpose() << '\n'
            << "  inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): " << Y.inertia().data().transpose() << '\n' << '\n';
        }
        else if (link->getParent())
          throw std::invalid_argument(link->name + " - joint information missing.");

        BOOST_FOREACH(::urdf::LinkConstSharedPtr child, link->child_links)
        {
          parseTree(child, model);
        }
      }

      ///
      /// \brief Parse a tree with a specific root joint linking the model to the environment.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current URDF link.
      /// \param[in] model The model where the link must be added.
      ///
      void parseRootTree(const ::urdf::ModelInterface * urdfTree,
                         UrdfVisitorBase& model)
      {
        model.setName(urdfTree->getName());

        ::urdf::LinkConstSharedPtr root_link = urdfTree->getRoot();
        model.addRootJoint(convertFromUrdf(root_link->inertial), root_link->name);

        BOOST_FOREACH(::urdf::LinkConstSharedPtr child, root_link->child_links)
        {
          parseTree(child, model);
        }
      }

      void parseRootTree(const std::string & filename,
                         UrdfVisitorBase& model)
      {
        ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile (filename);
        if (urdfTree)
          return parseRootTree (urdfTree.get(), model);
        else
          throw std::invalid_argument("The file " + filename + " does not "
              "contain a valid URDF model.");
      }

      void parseRootTreeFromXML(const std::string & xmlString,
                                UrdfVisitorBase& model)
      {
        ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlString);
        if (urdfTree)
          return parseRootTree (urdfTree.get(), model);
        else
          throw std::invalid_argument("The XML stream does not contain a valid "
              "URDF model.");
      }
    } // namespace details
  } // namespace urdf
} // namespace pinocchio
