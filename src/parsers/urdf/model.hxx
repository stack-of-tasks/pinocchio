//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_parsers_urdf_model_hxx__
#define __pinocchio_multibody_parsers_urdf_model_hxx__

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include <sstream>
#include <boost/foreach.hpp>
#include <limits>

namespace pinocchio
{
  namespace urdf
  {
    namespace details
    {
      typedef double urdf_value_type;

      template<typename _Scalar, int Options>
      class UrdfVisitorBaseTpl {
        public:
          enum JointType {
            REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR
          };

          typedef _Scalar Scalar;
          typedef SE3Tpl<Scalar,Options> SE3;
          typedef InertiaTpl<Scalar,Options> Inertia;

          typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
          typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
          typedef Eigen::Ref<Vector> VectorRef;
          typedef Eigen::Ref<const Vector> VectorConstRef;

          virtual void setName (const std::string& name) = 0;

          virtual void addRootJoint (const Inertia& Y, const std::string & body_name) = 0;

          virtual void addJointAndBody(
              JointType type,
              const Vector3& axis,
              const FrameIndex & parentFrameId,
              const SE3 & placement,
              const std::string & joint_name,
              const Inertia& Y,
              const std::string & body_name,
              const VectorConstRef& max_effort,
              const VectorConstRef& max_velocity,
              const VectorConstRef& min_config,
              const VectorConstRef& max_config,
              const VectorConstRef& friction,
              const VectorConstRef& damping
                                       ) = 0;

          virtual void addFixedJointAndBody(
              const FrameIndex & parentFrameId,
              const SE3 & joint_placement,
              const std::string & joint_name,
              const Inertia& Y,
              const std::string & body_name) = 0;

          virtual void appendBodyToJoint(
              const FrameIndex fid,
              const Inertia& Y,
              const SE3 & placement,
              const std::string & body_name) = 0;

          virtual FrameIndex getBodyId (
              const std::string& frame_name) const = 0;

          UrdfVisitorBaseTpl () : log (NULL) {}

          template <typename T>
          UrdfVisitorBaseTpl& operator<< (const T& t)
          {
            if (log != NULL) *log << t;
            return *this;
          }

          std::ostream* log;
      };

      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      class UrdfVisitor : public UrdfVisitorBaseTpl<Scalar, Options>
      {
        public:
          typedef UrdfVisitorBaseTpl<Scalar, Options> Base;
          typedef typename Base::JointType      JointType;
          typedef typename Base::Vector3        Vector3;
          typedef typename Base::VectorConstRef VectorConstRef;
          typedef typename Base::SE3            SE3;
          typedef typename Base::Inertia        Inertia;

          typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
          typedef typename Model::JointCollection JointCollection;
          typedef typename Model::JointModel JointModel;
          typedef typename Model::Frame Frame;

          Model& model;

          UrdfVisitor (Model& model) : model(model) {}

          void setName (const std::string& name)
          {
            model.name = name;
          }

          virtual void addRootJoint(const Inertia& Y, const std::string & body_name)
          {
            addFixedJointAndBody(0, SE3::Identity(), "root_joint", Y, body_name);
//            appendBodyToJoint(0,Y,SE3::Identity(),body_name); TODO: change for the correct behavior, see https://github.com/stack-of-tasks/pinocchio/pull/1102 for discussions on the topic

              
          }

          void addJointAndBody(
              JointType type,
              const Vector3& axis,
              const FrameIndex & parentFrameId,
              const SE3 & placement,
              const std::string & joint_name,
              const Inertia& Y,
              const std::string & body_name,
              const VectorConstRef& max_effort,
              const VectorConstRef& max_velocity,
              const VectorConstRef& min_config,
              const VectorConstRef& max_config,
              const VectorConstRef& friction,
              const VectorConstRef& damping)
          {
            JointIndex joint_id;
            const Frame & frame = model.frames[parentFrameId];

            switch (type) {
              case Base::FLOATING:
                joint_id = model.addJoint(frame.parent,
                                          typename JointCollection::JointModelFreeFlyer(),
                                          frame.placement * placement,
                                          joint_name,
                                          max_effort,max_velocity,min_config,max_config,
                                          friction,damping
                                          );
                break;
              case Base::REVOLUTE:
                joint_id = addJoint<
                  typename JointCollection::JointModelRX,
                  typename JointCollection::JointModelRY,
                  typename JointCollection::JointModelRZ,
                  typename JointCollection::JointModelRevoluteUnaligned> (
                      axis, frame, placement, joint_name,
                      max_effort, max_velocity, min_config, max_config,
                      friction, damping);
                break;
              case Base::CONTINUOUS:
                joint_id = addJoint<
                  typename JointCollection::JointModelRUBX,
                  typename JointCollection::JointModelRUBY,
                  typename JointCollection::JointModelRUBZ,
                  typename JointCollection::JointModelRevoluteUnboundedUnaligned>(
                      axis, frame, placement, joint_name,
                      max_effort, max_velocity, min_config, max_config,
                      friction, damping);
                break;
              case Base::PRISMATIC:
                joint_id = addJoint<
                  typename JointCollection::JointModelPX,
                  typename JointCollection::JointModelPY,
                  typename JointCollection::JointModelPZ,
                  typename JointCollection::JointModelPrismaticUnaligned> (
                      axis, frame, placement, joint_name,
                      max_effort, max_velocity, min_config, max_config,
                      friction, damping);
                break;
              case Base::PLANAR:
                joint_id = model.addJoint(frame.parent,
                    typename JointCollection::JointModelPlanar(),
                    frame.placement * placement,
                    joint_name,
                    max_effort,max_velocity,min_config,max_config,
                    friction, damping
                    );
                break;
              default:
                PINOCCHIO_CHECK_INPUT_ARGUMENT(false, "The joint type is not correct.");
            };

            FrameIndex jointFrameId = model.addJointFrame(joint_id, (int)parentFrameId);
            appendBodyToJoint(jointFrameId, Y, SE3::Identity(), body_name);
          }

          void addFixedJointAndBody(
              const FrameIndex & parentFrameId,
              const SE3 & joint_placement,
              const std::string & joint_name,
              const Inertia& Y,
              const std::string & body_name)
          {
            const Frame & frame = model.frames[parentFrameId];

            FrameIndex fid = model.addFrame(Frame(joint_name, frame.parent, parentFrameId,
                  frame.placement * joint_placement, FIXED_JOINT)
                );

            appendBodyToJoint((FrameIndex)fid, Y, SE3::Identity(), body_name);
          }

          void appendBodyToJoint(
              const FrameIndex fid,
              const Inertia& Y,
              const SE3 & placement,
              const std::string & body_name)
          {
            const Frame & frame = model.frames[fid];
            const SE3 & p = frame.placement * placement;
            assert(frame.parent >= 0);
            if(!Y.isZero(Scalar(0)))
            {
              model.appendBodyToJoint(frame.parent, Y, p);
            }

            model.addBodyFrame(body_name, frame.parent, p, (int)fid);
            // Reference to model.frames[fid] can has changed because the vector
            // may have been reallocated.
            assert (model.frames[fid].parent >= 0);
            {
              assert (   !hasNaN(model.inertias[model.frames[fid].parent].lever())
                  && !hasNaN(model.inertias[model.frames[fid].parent].inertia().data()));
            }
          }

          FrameIndex getBodyId (const std::string& frame_name) const
          {
            if (model.existFrame(frame_name, BODY)) {
              FrameIndex fid = model.getFrameId (frame_name, BODY);
              assert(model.frames[fid].type == BODY);
              return fid;
            } else
              throw std::invalid_argument("Model does not have any body named "
                  + frame_name);
          }

        private:
          ///
          /// \brief The four possible cartesian types of an 3D axis.
          ///
          enum CartesianAxis { AXIS_X=0, AXIS_Y=1, AXIS_Z=2, AXIS_UNALIGNED };

          ///
          /// \brief Extract the cartesian property of a particular 3D axis.
          ///
          /// \param[in] axis The input URDF axis.
          ///
          /// \return The property of the particular axis pinocchio::urdf::CartesianAxis.
          ///
          static inline CartesianAxis extractCartesianAxis (const Vector3 & axis)
          {
            if( axis == Vector3(1., 0., 0.))
              return AXIS_X;
            else if( axis == Vector3(0., 1., 0.))
              return AXIS_Y;
            else if( axis == Vector3(0., 0., 1.))
              return AXIS_Z;
            else
              return AXIS_UNALIGNED;
          }

          template <typename TypeX, typename TypeY, typename TypeZ,
                   typename TypeUnaligned>
          JointIndex addJoint(
              const Vector3& axis,
              const Frame & frame,
              const SE3 & placement,
              const std::string & joint_name,
              const VectorConstRef& max_effort,
              const VectorConstRef& max_velocity,
              const VectorConstRef& min_config,
              const VectorConstRef& max_config,
              const VectorConstRef& friction,
              const VectorConstRef& damping)
          {
            CartesianAxis axisType = extractCartesianAxis(axis);
            switch (axisType)
            {
              case AXIS_X:
                return model.addJoint(frame.parent, TypeX(),
                    frame.placement * placement, joint_name,
                    max_effort,max_velocity,min_config,max_config,
                    friction, damping);
                break;

              case AXIS_Y:
                return model.addJoint(frame.parent, TypeY(),
                    frame.placement * placement, joint_name,
                    max_effort,max_velocity,min_config,max_config,
                    friction, damping);
                break;

              case AXIS_Z:
                return model.addJoint(frame.parent, TypeZ(),
                    frame.placement * placement, joint_name,
                    max_effort,max_velocity,min_config,max_config,
                    friction, damping);
                break;

              case AXIS_UNALIGNED:
                return model.addJoint(frame.parent, TypeUnaligned (axis.normalized()),
                    frame.placement * placement, joint_name,
                    max_effort,max_velocity,min_config,max_config,
                    friction, damping);
                break;
              default:
                PINOCCHIO_CHECK_INPUT_ARGUMENT(false, "The axis type of the joint is of wrong type.");
                break;
            }
          }
      };

      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      class UrdfVisitorWithRootJoint : public UrdfVisitor<Scalar, Options, JointCollectionTpl>
      {
        public:
          typedef UrdfVisitor<Scalar, Options, JointCollectionTpl> Base;
          typedef typename Base::Inertia        Inertia;
          using Base::model;
          using Base::appendBodyToJoint;

          typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
          typedef typename Model::JointCollection JointCollection;
          typedef typename Model::JointModel JointModel;

          JointModel root_joint;

          UrdfVisitorWithRootJoint (Model& model, const JointModelBase<JointModel> & root_joint)
            : Base (model), root_joint (root_joint.derived()) {}

          void addRootJoint(const Inertia & Y, const std::string & body_name)
          {
            const Frame & frame = model.frames[0];
              
            PINOCCHIO_THROW(!model.existJointName("root_joint"),
                            std::invalid_argument,
                            "root_joint already exists as a joint in the kinematic tree.");
            
            JointIndex idx = model.addJoint(frame.parent, root_joint,
                SE3::Identity(), "root_joint"
                //TODO ,max_effort,max_velocity,min_config,max_config
                );

            FrameIndex jointFrameId = model.addJointFrame(idx, 0);
            appendBodyToJoint(jointFrameId, Y, SE3::Identity(), body_name);
          }
      };

      typedef UrdfVisitorBaseTpl<double, 0> UrdfVisitorBase;

      void PINOCCHIO_DLLAPI parseRootTree(const ::urdf::ModelInterface * urdfTree,
                                             UrdfVisitorBase & model);

      void PINOCCHIO_DLLAPI parseRootTree(const std::string & filename,
                                             UrdfVisitorBase & model);

      void PINOCCHIO_DLLAPI parseRootTreeFromXML(const std::string & xmlString,
                                                    UrdfVisitorBase & model);
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & root_joint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl> visitor (model, root_joint);
      if (verbose) visitor.log = &std::cout;
      parseRootTree(filename, visitor);
      return model;
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      if (verbose) visitor.log = &std::cout;
      parseRootTree(filename, visitor);
      return model;
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose)
    {
      details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl> visitor (model, rootJoint);
      if (verbose) visitor.log = &std::cout;
      parseRootTreeFromXML(xmlStream, visitor);
      return model;
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose)
    {
      details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      if (verbose) visitor.log = &std::cout;
      parseRootTreeFromXML(xmlStream, visitor);
      return model;
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const boost::shared_ptr< ::urdf::ModelInterface> urdfTree,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(urdfTree != NULL);
      details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl> visitor (model, rootJoint);
      if (verbose) visitor.log = &std::cout;
      parseRootTree(urdfTree.get(), visitor);
      return model;
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const boost::shared_ptr< ::urdf::ModelInterface> urdfTree,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(urdfTree != NULL);
      details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      if (verbose) visitor.log = &std::cout;
      parseRootTree(urdfTree.get(), visitor);
      return model;
    }

#ifdef PINOCCHIO_WITH_CXX11_SUPPORT
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::shared_ptr< ::urdf::ModelInterface> urdfTree,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(urdfTree != NULL);
      details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl> visitor (model, rootJoint);
      if (verbose) visitor.log = &std::cout;
      parseRootTree(urdfTree.get(), visitor);
      return model;
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::shared_ptr< ::urdf::ModelInterface> urdfTree,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(urdfTree != NULL);
      details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      if (verbose) visitor.log = &std::cout;
      parseRootTree(urdfTree.get(), visitor);
      return model;
    }
#endif

  } // namespace urdf
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_parsers_urdf_model_hxx__
