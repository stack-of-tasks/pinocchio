//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_multibody_parsers_sdf_model_hxx__
#define __pinocchio_multibody_parsers_sdf_model_hxx__

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/contact-info.hpp"


#include <sdf/sdf.hh>
#include <ignition/math.hh>
#include <sstream>
#include <boost/foreach.hpp>
#include <limits>
#include <iostream>

namespace pinocchio
{
  namespace sdf
  {
    namespace details
    {

      static SE3 convertFromPose3d(const ignition::math::Pose3d& posePlacement)
      {
        const ignition::math::Quaterniond& q = posePlacement.Rot();
        const ignition::math::Vector3d& p = posePlacement.Pos();
        return SE3(SE3::Quaternion(q.W(),q.X(),q.Y(),q.Z()).matrix(),
                   SE3::Vector3(p.X(),p.Y(),p.Z()));
      }
      
      ///
      /// \brief Convert SDF Inertial quantity to Spatial Inertia.
      ///
      /// \param[in] Y The input URDF Inertia.
      ///
      /// \return The converted Spatial Inertia pinocchio::Inertia.
      ///
      static Inertia convertInertiaFromSdf(const ::sdf::ElementPtr inertial)
      {
        
        const ignition::math::Pose3d& pose =
          inertial->template Get<ignition::math::Pose3d>("pose");
        const double& mass = inertial->template Get<double>("mass");

        const ::sdf::ElementPtr inertiaElem = inertial->GetElement("inertia");
        const double ixx = inertiaElem->template Get<double>("ixx");
        const double ixy = inertiaElem->template Get<double>("ixy");
        const double ixz = inertiaElem->template Get<double>("ixz");
        const double iyy = inertiaElem->template Get<double>("iyy");
        const double iyz = inertiaElem->template Get<double>("iyz");
        const double izz = inertiaElem->template Get<double>("izz");

        const Inertia::Vector3 com(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        const Inertia::Matrix3 & R =
          Eigen::Quaterniond(pose.Rot().W(),pose.Rot().X(),
                             pose.Rot().Y(),pose.Rot().Z()).matrix();

        Inertia::Matrix3 I;
        I << ixx,ixy,ixz,
             ixy,iyy,iyz,
             ixz,iyz,izz;
        
        return Inertia(mass,com,R*I*R.transpose());
      }      

      
      struct SdfGraph
      {
      public:
        typedef std::map<std::string, ::sdf::ElementPtr> ElementMap_t;
        typedef std::map<std::string, std::vector<std::string> > StringVectorMap_t;
        
        ElementMap_t mapOfLinks, mapOfJoints;
        StringVectorMap_t childrenOfLinks;
        std::string modelName;
        std::vector<std::string> childToBeAdded;

        typedef ::pinocchio::urdf::details::UrdfVisitorBase UrdfVisitorBase;
        UrdfVisitorBase& urdfVisitor;
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models;

        SdfGraph(::pinocchio::urdf::details::UrdfVisitorBase& urdfVisitor,
                 PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models)
          : urdfVisitor(urdfVisitor),
            contact_models(contact_models)
        {}

        void parseGraph(const std::string & filename)
        {
          // load and check sdf file
          ::sdf::SDFPtr sdfElement(new ::sdf::SDF());
          ::sdf::init(sdfElement);
          if (!::sdf::readFile(filename, sdfElement))
          {
            throw std::invalid_argument("The file " + filename + " does not "
                                        "contain a valid SDF model 1.");
          }
          
          // start parsing model
          const ::sdf::ElementPtr rootElement = sdfElement->Root();
          
          if (!rootElement->HasElement("model"))
          {
            throw std::invalid_argument("The file " + filename + " does not "
                                        "contain a valid SDF model 2.");
          }
          
          const ::sdf::ElementPtr modelElement = rootElement->GetElement("model");
          
          modelName = modelElement->template Get<std::string>("name");

          // parse model links
          ::sdf::ElementPtr linkElement = modelElement->GetElement("link");
          while (linkElement)
          {
            const std::string linkName = linkElement->Get<std::string>("name");
            //Inserting data in std::map
            mapOfLinks.insert(std::make_pair(linkName, linkElement));
            childrenOfLinks.insert(std::make_pair(linkName, std::vector<std::string>()));
            linkElement = linkElement->GetNextElement("link");
          }
          
          childrenOfLinks.insert(std::make_pair("world", std::vector<std::string>()));
          
          // parse model joints
          ::sdf::ElementPtr jointElement = modelElement->GetElement("joint");
          while (jointElement)
          {
            const std::string jointName = jointElement->template Get<std::string>("name");
            std::string parentLinkName =
              jointElement->GetElement("parent")->template Get<std::string>();
            //Inserting data in std::map
            mapOfJoints.insert(std::make_pair(jointName, jointElement));
            //Create data of children of links
            childrenOfLinks.find(parentLinkName)->second.push_back(jointName);
            jointElement = jointElement->GetNextElement("joint");
          }
        }

        static bool existConstraint(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                               const std::string& jointName)
        {
          for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)::const_iterator
                cm = std::begin(contact_models);
              cm != std::end(contact_models); ++cm)
            {
              if(cm->name == "jointName")
                return true;
            }
          return false;
        }

        static bool existChildName(const std::vector<std::string>& listOfNames,
                                   const std::string& childName)
        {
          for(std::vector<std::string>::const_iterator cm = std::begin(listOfNames);
              cm != std::end(listOfNames); ++cm)
          {
            if((*cm) == childName)
              return true;
          }
          return false;
        }

        
        static int getConstraintId(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                                           const std::string& jointName)
        {
          std::size_t i = 0;
          for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)::const_iterator
                cm = std::begin(contact_models);
              cm != std::end(contact_models); ++cm)
          {
            if(cm->name == "jointName")
              return i;
            i++;
            }
          return -1;
        }


        int getConstraintIdFromChild(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                                     const std::string& childName)
        {
          std::size_t i = 0;
          for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)::const_iterator
                cm = std::begin(contact_models);
              cm != std::end(contact_models); ++cm)
          {
            const std::string childFromMap =
              mapOfJoints.find(cm->name)->second->GetElement("child")->Get<std::string>();
            if(childFromMap == childName)
              return i;
            i++;
            }
          return -1;
        }
        
        ///
        /// \brief Recursive procedure for reading the SDF tree.
        ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
        ///
        /// \param[in] link The current SDF link.
        /// \param[in] model The model where the link must be added.
        ///
        void recursiveFillModel(const ::sdf::ElementPtr jointElement)
        {
          typedef UrdfVisitorBase::Scalar Scalar;
          typedef UrdfVisitorBase::SE3 SE3;
          typedef UrdfVisitorBase::Vector Vector;
          typedef UrdfVisitorBase::Vector3 Vector3;
          const std::string& jointName = jointElement->template Get<std::string>("name");

          std::ostringstream joint_info;
          const std::string parentName = jointElement->GetElement("parent")->Get<std::string>();
          const std::string childName =
            jointElement->GetElement("child")->Get<std::string>();          
          const ::sdf::ElementPtr childElement =  mapOfLinks.find(childName)->second;
          const ::sdf::ElementPtr parentElement = mapOfLinks.find(parentName)->second;
          const ignition::math::Pose3d parentPlacement =
            parentElement->template Get<ignition::math::Pose3d>("pose");
          const ignition::math::Pose3d childPlacement =
            childElement->template Get<ignition::math::Pose3d>("pose");

          const JointIndex parentJointId = urdfVisitor.getParentId(parentName);
          const std::string& parentJointName = urdfVisitor.getJointName(parentJointId);

          
          SE3 cMj(SE3::Identity()), pMjp(SE3::Identity());

          if (jointElement->HasElement("pose"))
          {
            const ignition::math::Pose3d cMj_ig =
              jointElement->template Get<ignition::math::Pose3d>("pose");
            cMj = ::pinocchio::sdf::details::convertFromPose3d(cMj_ig);
          }

          if (parentJointName != "root_joint") {
            const ::sdf::ElementPtr parentJointElement = mapOfJoints.find(parentJointName)->second;
          
            if (parentJointElement->HasElement("pose"))
            {

              const ignition::math::Pose3d parentcMj_ig =
                parentJointElement->template Get<ignition::math::Pose3d>("pose");
              pMjp = ::pinocchio::sdf::details::convertFromPose3d(parentcMj_ig);
            }
          }
          
          const SE3 oMp = ::pinocchio::sdf::details::convertFromPose3d(parentPlacement);
          const SE3 oMc = ::pinocchio::sdf::details::convertFromPose3d(childPlacement);
          const SE3 jointPlacement = pMjp.inverse() * oMp.inverse() * oMc * cMj;
          
          joint_info << "Joint " << jointName << " connects parent " << parentName
                    << " link to child " << childName << " link" << " with joint type "
                    << jointElement->template Get<std::string>("type")<<std::endl;

          
          //TODO: Check left-knee-shin-joint axis
          //if (urdfVisitor.existFrame(childName, BODY)) {
          if (jointElement->template Get<std::string>("type") == "ball") {
            JointIndex existingParentJointId;
            if (urdfVisitor.existFrame(childName, BODY))
            {
              if (! existConstraint(contact_models, jointName))
              {
                std::cout<<childName<<" already exists"<<std::endl;
                existingParentJointId = urdfVisitor.getParentId(childName);
                const ::sdf::ElementPtr prevJointElement =
                  mapOfJoints.find(urdfVisitor.getJointName(existingParentJointId))->second;
                std::cout<<"connected by joint "
                         <<urdfVisitor.getJointName(existingParentJointId)<<std::endl;
                SE3 cMj1(SE3::Identity());
                if (prevJointElement->HasElement("pose"))
                {
                  const ignition::math::Pose3d prevcMj_ig =
                    prevJointElement->template Get<ignition::math::Pose3d>("pose");
                  cMj1 = ::pinocchio::sdf::details::convertFromPose3d(prevcMj_ig);
                }

                ::pinocchio::RigidConstraintModel rcm (::pinocchio::CONTACT_3D,
                                                    parentJointId,
                                                    jointPlacement,
                                                    existingParentJointId,
                                                    cMj1.inverse() * cMj);
                rcm.name = jointName;
                contact_models.push_back(rcm);
                
              }
              else
              {
                const int i = getConstraintId(contact_models, jointName);
                if(i != -1) {
                  contact_models[i].joint2_id = parentJointId;
                  contact_models[i].joint2_placement = jointPlacement;
                }
                else
                {
                  throw std::invalid_argument("Unknown error with sdf parsing");
                }
              }
            }
            else
            {
              std::cout<<childName<<" not yet added to model"<<std::endl;
              std::cout<<jointName<<" corresponds to pending link"<<childName<<std::endl;
              existingParentJointId = -1;
              ::pinocchio::RigidConstraintModel rcm (::pinocchio::CONTACT_3D,
                                                  parentJointId,
                                                  jointPlacement,
                                                  existingParentJointId,
                                                  cMj);
              rcm.name = jointName;
              childToBeAdded.push_back(childName);
              contact_models.push_back(rcm);
            }
          }
          else {            
            //childElement is the link. 
            const ::sdf::ElementPtr inertialElem = childElement->GetElement("inertial");
            const Inertia Y = ::pinocchio::sdf::details::convertInertiaFromSdf(inertialElem);
                        
            FrameIndex parentFrameId = urdfVisitor.getBodyId(parentName);
            Vector max_effort(1), max_velocity(1), min_config(1), max_config(1);
            Vector spring_stiffness(1), spring_reference(1);
            Vector friction(Vector::Constant(1,0.)), damping(Vector::Constant(1,0.));
            ignition::math::Vector3d axis_ignition;
            Vector3 axis;
            
            const Scalar infty = std::numeric_limits<Scalar>::infinity();
            
            if (jointElement->HasElement("axis"))
            {
              const ::sdf::ElementPtr axisElem = jointElement->GetElement("axis");
              
              axis_ignition =
                axisElem->Get<ignition::math::Vector3d>("xyz");
              axis << axis_ignition.X(), axis_ignition.Y(), axis_ignition.Z();
              
              if (axisElem->HasElement("limit"))
              {
                const ::sdf::ElementPtr limitElem = axisElem->GetElement("limit");
                if (limitElem->HasElement("upper"))
                {
                  max_config[0] = limitElem->Get<double>("upper");
                }
                if (limitElem->HasElement("lower"))
                {
                  min_config[0] = limitElem->Get<double>("lower");
                }
                if (limitElem->HasElement("effort"))
                {
                  max_effort[0] = limitElem->Get<double>("effort");
                }
                if (limitElem->HasElement("velocity"))
                {
                  max_velocity[0] = limitElem->Get<double>("velocity");
                }
              }
              if (axisElem->HasElement("dynamics"))
              {
                const ::sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");
                if (dynamicsElem->HasElement("spring_reference"))
                {
                  spring_reference[0] = dynamicsElem->Get<double>("spring_reference");
                }
                if (dynamicsElem->HasElement("spring_stiffness"))
                {
                  spring_stiffness[0] = dynamicsElem->Get<double>("spring_stiffness");
                }
                if (dynamicsElem->HasElement("damping"))
                {
                  damping[0] = dynamicsElem->Get<double>("damping");
                }
              }
            }
            
            if (jointElement->template Get<std::string>("type") == "universal")
            {
            }
            else if (jointElement->template Get<std::string>("type") == "revolute")
            {
              joint_info << "joint REVOLUTE with axis"<< axis.transpose();
              urdfVisitor.addJointAndBody(UrdfVisitorBase::REVOLUTE, axis,
                                          parentFrameId, jointPlacement, jointName,
                                          Y, cMj.inverse(), childName,
                                          max_effort, max_velocity, min_config, max_config,
                                          friction,damping);
            }
            else if (jointElement->template Get<std::string>("type") == "gearbox")
            {
              joint_info << "joint GEARBOX with axis";
              urdfVisitor.addFixedJointAndBody(parentFrameId, jointPlacement, jointName,
                                               Y, childName);
            }
            else if (jointElement->template Get<std::string>("type") == "ball")
            {
              max_effort   = Vector::Constant(3, infty);
              max_velocity = Vector::Constant(3, infty);
              min_config   = Vector::Constant(4,-infty);
              max_config   = Vector::Constant(4, infty);
              min_config.setConstant(-1.01);
              max_config.setConstant( 1.01);
              friction = Vector::Constant(3, 0.);
              damping = Vector::Constant(3, 0.);
              
              joint_info << "joint BALL";
              //std::cerr<<"TODO: Fix BALL JOINT"<<std::endl;
              urdfVisitor.addJointAndBody(UrdfVisitorBase::SPHERICAL, axis,
                                          parentFrameId, jointPlacement, jointName,
                                          Y, cMj.inverse(), childName,
                                          max_effort, max_velocity, min_config, max_config,
                                          friction,damping);
            }
            else
            {
              std::cerr<<"This type is yet to be implemented "<<jointElement->template Get<std::string>("type")<<std::endl;
              
            }


            if (existChildName(childToBeAdded, childName)) {
              int constraintId = getConstraintIdFromChild(contact_models, childName);
              if (constraintId != -1)
              {
                contact_models[constraintId].joint2_id = urdfVisitor.getJointId(jointName);
              }
              else {
                throw std::invalid_argument("Something wrong here");
              }
            }
            
            const std::vector<std::string>& childrenOfLink =
              childrenOfLinks.find(childName)->second;
            
            for(std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
                childOfChild != std::end(childrenOfLink); ++childOfChild)
            {
              const ::sdf::ElementPtr childOfChildElement =
                mapOfJoints.find(*childOfChild)->second;
              recursiveFillModel(childOfChildElement);
            }
          }
        }
      };
      
      void PINOCCHIO_DLLAPI parseRootTree(SdfGraph& graph);
      }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & root_joint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
               const bool verbose)
    {
      ::pinocchio::urdf::details::UrdfVisitorWithRootJoint<Scalar, Options,
                                                           JointCollectionTpl> visitor (model, root_joint);

      ::pinocchio::sdf::details::SdfGraph graph (visitor, contact_models);
      
      if (verbose) visitor.log = &std::cout;

      //Create maps from the SDF Graph
      graph.parseGraph(filename);
      //Use the SDF graph to create the model
      details::parseRootTree(graph);
      return model;
    }
    
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
               const bool verbose)
    {
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      ::pinocchio::sdf::details::SdfGraph graph (visitor, contact_models);
      
      if (verbose) visitor.log = &std::cout;

      //Create maps from the SDF Graph
      graph.parseGraph(filename);
      //Use the SDF graph to create the model
      details::parseRootTree(graph);
      return model;
    }
  }
}

#endif // ifndef __pinocchio_parsers_sdf_hpp__
