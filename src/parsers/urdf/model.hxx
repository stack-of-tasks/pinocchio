//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_parsers_urdf_model_hxx__
#define __pinocchio_multibody_parsers_urdf_model_hxx__

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/utils.hpp"
#include "pinocchio/multibody/model.hpp"

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
      const FrameType JOINT_OR_FIXED_JOINT = (FrameType) (JOINT | FIXED_JOINT);
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      FrameIndex getParentJointFrame(const ::urdf::LinkConstSharedPtr link,
                                     const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::Frame Frame;
        typedef typename Model::FrameIndex FrameIndex;
        
        assert(link && link->getParent());
        
        FrameIndex id;
        if (!link->getParent()->parent_joint)
        {
          if (model.existFrame("root_joint",  JOINT_OR_FIXED_JOINT))
            id = model.getFrameId("root_joint", JOINT_OR_FIXED_JOINT);
          else
            id = 0;
        }
        else
        {
          if (model.existFrame(link->getParent()->parent_joint->name, JOINT_OR_FIXED_JOINT))
            id = model.getFrameId (link->getParent()->parent_joint->name, JOINT_OR_FIXED_JOINT);
          else
            throw std::invalid_argument("Model does not have any joints named "
                                        + link->getParent()->parent_joint->name);
        }
        
        const Frame & f = model.frames[id];
        if (f.type == JOINT || f.type == FIXED_JOINT)
          return id;
        throw std::invalid_argument("Parent frame is not a JOINT neither a FIXED_JOINT");
      }

      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      void appendBodyToJoint(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const FrameIndex fid,
                             const ::urdf::InertialConstSharedPtr Y_ptr,
                             const typename ModelTpl<Scalar,Options,JointCollectionTpl>::SE3 & placement,
                             const std::string & body_name)
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::Frame Frame;
        typedef typename Model::SE3 SE3;
        
        const Frame & frame = model.frames[fid];
        const SE3 & p = frame.placement * placement;
        if(frame.parent > 0
           && Y_ptr
           && Y_ptr->mass > Eigen::NumTraits<double>::epsilon())
        {
          model.appendBodyToJoint(frame.parent, convertFromUrdf(*Y_ptr), p);
        }
        
        model.addBodyFrame(body_name, frame.parent, p, (int)fid);
        // Reference to model.frames[fid] can has changed because the vector
        // may have been reallocated.
        if (model.frames[fid].parent > 0)
        {
          assert (   !hasNaN(model.inertias[model.frames[fid].parent].lever())
                  && !hasNaN(model.inertias[model.frames[fid].parent].inertia().data()));
        }
      }
      
      ///
      /// \brief Shortcut for adding a joint and directly append a body to it.
      ///
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename JointModel>
      void addJointAndBody(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const JointModelBase<JointModel> & jmodel,
                           const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex & parentFrameId,
                           const typename ModelTpl<Scalar,Options,JointCollectionTpl>::SE3 & joint_placement,
                           const std::string & joint_name,
                           const ::urdf::InertialConstSharedPtr Y,
                           const std::string & body_name,
                           const typename JointModel::TangentVector_t & max_effort,
                           const typename JointModel::TangentVector_t & max_velocity,
                           const typename JointModel::ConfigVector_t  & min_config,
                           const typename JointModel::ConfigVector_t  & max_config)
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::Frame Frame;
        typedef typename Model::SE3 SE3;
        typedef typename Model::JointIndex JointIndex;
        typedef typename Model::FrameIndex FrameIndex;
        
        JointIndex idx;
        const Frame & frame = model.frames[parentFrameId];
        
        idx = model.addJoint(frame.parent,jmodel,
                             frame.placement * joint_placement,
                             joint_name,
                             max_effort,max_velocity,min_config,max_config
                             );
        int res (model.addJointFrame(idx, (int)parentFrameId));
        if (res == -1) {
          std::ostringstream oss;
          oss << joint_name << " already inserted as a frame. Current frames "
          "are [";
          for (typename container::aligned_vector<Frame>::const_iterator it =
               model.frames.begin (); it != model.frames.end (); ++it) {
            oss << "\"" << it->name << "\",";
          }
          oss << "]";
          throw std::invalid_argument(oss.str().c_str());
        }
        
        FrameIndex jointFrameId = (FrameIndex) res; // C-style cast to remove polluting compilation warning. This is Bad practice. See issue #323 (rework indexes)
        appendBodyToJoint(model, jointFrameId, Y, SE3::Identity(), body_name);
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename JointModel>
      void addJointAndBody(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const JointModelBase<JointModel> & jmodel,
                           const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex & parentFrameId,
                           const typename ModelTpl<Scalar,Options,JointCollectionTpl>::SE3 & joint_placement,
                           const std::string & joint_name,
                           const ::urdf::InertialConstSharedPtr Y,
                           const std::string & body_name)
      {
        const Scalar infty = std::numeric_limits<Scalar>::infinity();
        
        const typename JointModel::TangentVector_t max_effort   = JointModel::TangentVector_t::Constant(jmodel.nv(), infty);
        const typename JointModel::TangentVector_t max_velocity = JointModel::TangentVector_t::Constant(jmodel.nv(), infty);
        const typename JointModel::ConfigVector_t  min_config   = JointModel::ConfigVector_t ::Constant(jmodel.nq(),-infty);
        const typename JointModel::ConfigVector_t  max_config   = JointModel::ConfigVector_t ::Constant(jmodel.nq(), infty);
        
        addJointAndBody(model,jmodel.derived(),parentFrameId,joint_placement,joint_name,Y,body_name,
                        max_effort,max_velocity,min_config,max_config);
      }
      
      ///
      /// \brief Shortcut for adding a fixed joint and directly append a body to it.
      ///
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      void addFixedJointAndBody(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex & parentFrameId,
                                const typename ModelTpl<Scalar,Options,JointCollectionTpl>::SE3 & joint_placement,
                                const std::string & joint_name,
                                const ::urdf::InertialConstSharedPtr Y,
                                const std::string & body_name)
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::Frame Frame;
        
        const Frame & frame = model.frames[parentFrameId];
        
        int fid = model.addFrame(Frame(joint_name, frame.parent, parentFrameId,
                                       frame.placement * joint_placement, FIXED_JOINT)
                                 );
        if (fid < 0)
          throw std::invalid_argument("Fixed joint " + joint_name + " could not be added.");
        
        appendBodyToJoint(model, (FrameIndex)fid, Y, SE3::Identity(), body_name);
      }

      ///
      /// \brief Recursive procedure for reading the URDF tree.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current URDF link.
      /// \param[in] model The model where the link must be added.
      ///
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      void parseTree(::urdf::LinkConstSharedPtr link,
                     ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     bool verbose)
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::JointCollection JointCollection;
        typedef typename Model::SE3 SE3;
        typedef typename Model::FrameIndex FrameIndex;

        // Parent joint of the current body
        const ::urdf::JointConstSharedPtr joint =
        ::urdf::const_pointer_cast< ::urdf::Joint>(link->parent_joint);

        if(joint) // if the link is not the root of the tree
        {
          assert(link->getParent());

          const std::string & joint_name = joint->name;
          const std::string & link_name = link->name;
          const std::string & parent_link_name = link->getParent()->name;
          std::ostringstream joint_info;

          FrameIndex parentFrameId = getParentJointFrame(link, model);

          // Transformation from the parent link to the joint origin
          const SE3 jointPlacement
          = convertFromUrdf(joint->parent_to_joint_origin_transform).template cast<Scalar>();

          const ::urdf::InertialSharedPtr Y =
          ::urdf::const_pointer_cast< ::urdf::Inertial>(link->inertial);

          switch(joint->type)
          {
            case ::urdf::Joint::FLOATING:
              joint_info << "joint FreeFlyer";
              addJointAndBody(model,typename JointCollection::JointModelFreeFlyer(),
                              parentFrameId,jointPlacement,joint->name,
                              Y,link->name);

              break;

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

              CartesianAxis axis = extractCartesianAxis(joint->axis);

              switch(axis)
              {
                case AXIS_X:
                  joint_info << " along X";
                  addJointAndBody(model, typename JointCollection::JointModelRX(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_Y:
                  joint_info << " along Y";
                  addJointAndBody(model, typename JointCollection::JointModelRY(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_Z:
                  joint_info << " along Z";
                  addJointAndBody(model, typename JointCollection::JointModelRZ(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_UNALIGNED:
                {
                  typename SE3::Vector3 joint_axis((Scalar)joint->axis.x,(Scalar)joint->axis.y,(Scalar)joint->axis.z);
                  joint_info << " unaligned along (" << joint_axis.transpose() << ")";

                  addJointAndBody(model, typename JointCollection::JointModelRevoluteUnaligned(joint_axis.normalized()),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }
                default:
                  assert(false && "The axis type of the revolute joint is of wrong type.");
                  break;
              }
              break;
              }

            case ::urdf::Joint::CONTINUOUS: // Revolute joint with no joint limits
              {
              joint_info << "joint CONTINUOUS with axis";

              typedef JointModelRUBX::ConfigVector_t ConfigVector_t;
              typedef JointModelRUBX::TangentVector_t TangentVector_t;

              TangentVector_t max_effort;
              TangentVector_t max_velocity;
              const ConfigVector_t::Scalar u = 1.01;
              ConfigVector_t lower_position(-u, -u);
              ConfigVector_t upper_position( u,  u);

              if(joint->limits)
              {
                max_effort << joint->limits->effort;
                max_velocity << joint->limits->velocity;
              }

              CartesianAxis axis = extractCartesianAxis(joint->axis);

              switch(axis)
              {
                case AXIS_X:
                  joint_info << " along X";
                  addJointAndBody(model, typename JointCollection::JointModelRUBX(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_Y:
                  joint_info << " along Y";
                  addJointAndBody(model, typename JointCollection::JointModelRUBY(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_Z:
                  joint_info << " along Z";
                  addJointAndBody(model, typename JointCollection::JointModelRUBZ(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_UNALIGNED:
                {
                  typename SE3::Vector3 joint_axis((Scalar)joint->axis.x,(Scalar)joint->axis.y,(Scalar)joint->axis.z);
                  joint_info << " unaligned along (" << joint_axis.transpose() << ")";

                  typedef typename JointCollection::JointModelRevoluteUnaligned::ConfigVector_t ConfigVector_t;

                  const Scalar infty = std::numeric_limits<Scalar>::infinity();
                  ConfigVector_t lower_position(ConfigVector_t::Constant(-infty));
                  ConfigVector_t upper_position(ConfigVector_t::Constant(infty));

                  addJointAndBody(model, typename JointCollection::JointModelRevoluteUnaligned(joint_axis.normalized()),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position,upper_position);
                  break;
                }

                default:
                  assert(false && "The axis type of the revolute joint is of wrong type.");
                  break;
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
                  joint_info << " along X";
                  addJointAndBody(model, typename JointCollection::JointModelPX(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_Y:

                  joint_info << " along Y";
                  addJointAndBody(model, typename JointCollection::JointModelPY(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_Z:
                  joint_info << " along Z";
                  addJointAndBody(model, typename JointCollection::JointModelPZ(),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;

                case AXIS_UNALIGNED:
                {
                  typename SE3::Vector3 joint_axis((Scalar)joint->axis.x,(Scalar)joint->axis.y,(Scalar)joint->axis.z);
                  joint_info << " unaligned along (" << joint_axis.transpose() << ")";

                  addJointAndBody(model, typename JointCollection::JointModelPrismaticUnaligned(joint_axis.normalized()),
                                  parentFrameId,jointPlacement,joint->name,
                                  Y,link->name,
                                  max_effort,max_velocity,
                                  lower_position, upper_position);
                  break;
                }

                default:
                  assert(false && "The axis type of the prismatic joint is of wrong type.");
                  break;
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

              addJointAndBody(model, typename JointCollection::JointModelPlanar(),
                              parentFrameId,jointPlacement,joint->name,
                              Y,link->name,
                              max_effort,max_velocity,
                              lower_position, upper_position);

              }
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
              addFixedJointAndBody(model, parentFrameId, jointPlacement,
                                   joint_name, Y, link_name);
              
              break;
              
            default:
              {
                const std::string exception_message("The type of joint " + joint_name + " is not supported.");
                throw std::invalid_argument(exception_message);
                break;
              }
          }

          if(verbose)
          {
            const Inertia YY = (!Y) ? Inertia::Zero() : convertFromUrdf(*Y);
            std::cout << "Adding Body" << std::endl;
            std::cout << "\"" << link_name << "\" connected to " << "\"" << parent_link_name << "\" throw joint " << "\"" << joint_name << "\"" << std::endl;
            std::cout << "joint type: " << joint_info.str() << std::endl;
            std::cout << "joint placement:\n" << jointPlacement;
            std::cout << "body info: " << std::endl;
            std::cout << "  " << "mass: " << YY.mass() << std::endl;
            std::cout << "  " << "lever: " << YY.lever().transpose() << std::endl;
            std::cout << "  " << "inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): " << YY.inertia().data().transpose() << std::endl << std::endl;
          }
        }
        else if (link->getParent())
        {
          const std::string exception_message (link->name + " - joint information missing.");
          throw std::invalid_argument(exception_message);
        }

        BOOST_FOREACH(::urdf::LinkConstSharedPtr child, link->child_links)
        {
          parseTree(child, model, verbose);
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
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      void parseRootTree(::urdf::LinkConstSharedPtr root_link,
                         ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         const bool verbose)
      {
        addFixedJointAndBody(model, 0, SE3::Identity(), "root_joint",
                             root_link->inertial, root_link->name);
        
        BOOST_FOREACH(::urdf::LinkConstSharedPtr child, root_link->child_links)
        {
          parseTree(child, model, verbose);
        }
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
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename JointModel>
      void parseRootTree(::urdf::LinkConstSharedPtr root_link,
                         ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         const JointModelBase<JointModel> & root_joint,
                         const bool verbose)
      {
        addJointAndBody(model,root_joint,
                        0,SE3::Identity(),"root_joint",
                        root_link->inertial,root_link->name);

        BOOST_FOREACH(::urdf::LinkConstSharedPtr child, root_link->child_links)
        {
          parseTree(child, model, verbose);
        }
      }
    } // namespace details
              
    ///
    /// \brief Call parse root tree templated function
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    struct ParseRootTreeVisitor : public boost::static_visitor<>
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      ::urdf::LinkConstSharedPtr m_root_link;
      Model & m_model;
      const bool m_verbose;
     
      ParseRootTreeVisitor(::urdf::LinkConstSharedPtr root_link,
                           Model & model,
                           const bool verbose)
      : m_root_link(root_link)
      , m_model(model)
      , m_verbose(verbose)
      {}
      
      template<typename JointModel>
      void operator()(const JointModelBase<JointModel> & root_joint) const
      {
        details::parseRootTree(m_root_link,m_model,root_joint,m_verbose);
      }
      
      static void run(::urdf::LinkConstSharedPtr root_link,
                      Model & model,
                      const typename Model::JointModel & root_joint,
                      const bool verbose)
      {
        boost::apply_visitor(ParseRootTreeVisitor(root_link,model,verbose),root_joint);
      }
    }; // struct ParseRootTreeVisitor

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & root_joint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
     
    {
      ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile (filename);

      if (urdfTree)
      {
          return buildModel (urdfTree, root_joint, model, verbose);
      }
      else
      {
        const std::string exception_message ("The file " + filename + " does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
      }
      return model;
    }
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
   
    {
      ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
      if (urdfTree)
      {
          return buildModel(urdfTree, model, verbose);
      }
      else
      {
        const std::string exception_message("The file " + filename + " does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
      }
      
      return model;
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      const JointModelVariant & rootJoint,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose)
    {
      ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlStream);
      
      if (urdfTree)
        return buildModel(urdfTree, rootJoint, model, verbose);
      else
      {
        const std::string exception_message ("The XML stream does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
      }
      
      return model;
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose)
    {
      ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlStream);
      
      if (urdfTree)
        return buildModel(urdfTree, model, verbose);
      else
      {
        const std::string exception_message ("The XML stream does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
      }
      
      return model;
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const ::urdf::ModelInterfaceSharedPtr & urdfTree,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & root_joint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      assert(urdfTree);
      model.name = urdfTree->getName();
      ParseRootTreeVisitor<Scalar,Options,JointCollectionTpl>::run(urdfTree->getRoot(),model,root_joint,verbose);
      return model;
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const ::urdf::ModelInterfaceSharedPtr & urdfTree,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      assert(urdfTree);
      model.name = urdfTree->getName();
      details::parseRootTree(urdfTree->getRoot(),model,verbose);
      return model;
    }

  } // namespace urdf
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_parsers_urdf_model_hxx__
