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

      
      template<typename Scalar, int Options>
      struct ContactDetailsTpl
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        typedef SE3Tpl<Scalar,Options> SE3;
        typedef pinocchio::JointIndex JointIndex;

        /// \brief Name of the contact.
        std::string name;
        
        /// \brief Type of the contact.
        ContactType type;
        
        /// \brief Index of the first joint in the model tree
        JointIndex joint1_id;
        
        /// \brief Index of the second joint in the model tree
        JointIndex joint2_id;
        
        /// \brief Relative placement with respect to the frame of joint1.
        SE3 joint1_placement;
        
        /// \brief Relative placement with respect to the frame of joint2.
        SE3 joint2_placement;
        
        /// \brief Reference frame where the constraint is expressed (LOCAL_WORLD_ALIGNED or LOCAL)
        ReferenceFrame reference_frame;
        
        
        ContactDetailsTpl(const ContactType type,
                          const JointIndex joint1_id,
                          const SE3 & joint1_placement,
                          const JointIndex joint2_id,
                          const SE3 & joint2_placement,
                          const ReferenceFrame & reference_frame = LOCAL)
          : type(type)
          , joint1_id(joint1_id)
          , joint2_id(joint2_id)
          , joint1_placement(joint1_placement)
          , joint2_placement(joint2_placement)
          , reference_frame(reference_frame)
        {}
      };
      
      struct SdfGraph
      {
      public:
        typedef std::map<std::string, ::sdf::ElementPtr> ElementMap_t;
        typedef std::map<std::string, std::vector<std::string> > StringVectorMap_t;
        typedef ContactDetailsTpl<double, 0> ContactDetails;
        
        ElementMap_t mapOfLinks, mapOfJoints;
        StringVectorMap_t childrenOfLinks;
        std::string modelName;
        std::vector<std::string> childToBeAdded;

        typedef pinocchio::urdf::details::UrdfVisitor<double, 0, ::pinocchio::JointCollectionDefaultTpl > UrdfVisitor;
        UrdfVisitor& urdfVisitor;
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails) contact_details;

        SdfGraph(UrdfVisitor& urdfVisitor)
          : urdfVisitor(urdfVisitor)
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

        static bool existConstraint(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)& contact_details,
                               const std::string& jointName)
        {
          for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)::const_iterator
                cm = std::begin(contact_details);
              cm != std::end(contact_details); ++cm)
            {
              if(cm->name == jointName)
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

        
        static int getConstraintId(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)& contact_details,
                                           const std::string& jointName)
        {
          std::size_t i = 0;
          for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)::const_iterator
                cm = std::begin(contact_details);
              cm != std::end(contact_details); ++cm)
          {
            if(cm->name == jointName)
              return i;
            i++;
            }
          return -1;
        }


        int getConstraintIdFromChild(const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)& contact_details,
                                     const std::string& childName)
        {
          std::size_t i = 0;
          for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)::const_iterator
                cm = std::begin(contact_details);
              cm != std::end(contact_details); ++cm)
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
          typedef UrdfVisitor::Scalar Scalar;
          typedef UrdfVisitor::SE3 SE3;
          typedef UrdfVisitor::Vector Vector;
          typedef UrdfVisitor::Vector3 Vector3;
          const std::string& jointName = jointElement->template Get<std::string>("name");
          bool is_constraint = false;
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

          if (parentJointName != "root_joint" && parentJointName != "universe" ) {
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

          FrameIndex parentFrameId = urdfVisitor.getBodyId(parentName);
          Vector max_effort(1), max_velocity(1), min_config(1), max_config(1);
          Vector spring_stiffness(1), spring_reference(1);
          Vector friction(Vector::Constant(1,0.)), damping(Vector::Constant(1,0.));
          ignition::math::Vector3d axis_ignition;
          Vector3 axis;
          const Scalar infty = std::numeric_limits<Scalar>::infinity();
          
          if (jointElement->HasElement("axis")) {
            const ::sdf::ElementPtr axisElem = jointElement->GetElement("axis");
            
            axis_ignition =
              axisElem->Get<ignition::math::Vector3d>("xyz");
            axis << axis_ignition.X(), axis_ignition.Y(), axis_ignition.Z();
            
            if (axisElem->HasElement("limit")) {
              const ::sdf::ElementPtr limitElem = axisElem->GetElement("limit");
              if (limitElem->HasElement("upper")) {
                max_config[0] = limitElem->Get<double>("upper");
              }
              if (limitElem->HasElement("lower")) {
                min_config[0] = limitElem->Get<double>("lower");
              }
              if (limitElem->HasElement("effort")) {
                max_effort[0] = limitElem->Get<double>("effort");
              }
              if (limitElem->HasElement("velocity")) {
                max_velocity[0] = limitElem->Get<double>("velocity");
              }
            }
            if (axisElem->HasElement("dynamics")) {
              const ::sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");
              if (dynamicsElem->HasElement("spring_reference")) {
                spring_reference[0] = dynamicsElem->Get<double>("spring_reference");
              }
              if (dynamicsElem->HasElement("spring_stiffness")) {
                spring_stiffness[0] = dynamicsElem->Get<double>("spring_stiffness");
              }
              if (dynamicsElem->HasElement("damping")) {
                damping[0] = dynamicsElem->Get<double>("damping");
              }
            }
          }

          const ::sdf::ElementPtr inertialElem = childElement->GetElement("inertial");
          const Inertia Y_c = ::pinocchio::sdf::details::convertInertiaFromSdf(inertialElem);
          Inertia Y = Inertia::Zero();
          JointIndex existingJointId = -1;
          if (urdfVisitor.existFrame(childName, BODY))
          { // Child link exists, thus loop constraint should be active.
            //No Inertial Information is needed, as it would have already been added.
            is_constraint = true;

            // Find existing joint before adding new one.
            existingJointId = urdfVisitor.getParentId(childName);
            Y = Y_c.se3Action(cMj);
            Y.mass() *= 0.5;
            Y.inertia() *= 0.5;

            //TODO: ADD CONSTRAINT DEFINITION HERE
          }
          else {
            //childElement is the link that is new and should be added.
            Y = Y_c.se3Action(cMj);
          }
            
          if (jointElement->template Get<std::string>("type") == "universal") {
          }
          else if (jointElement->template Get<std::string>("type") == "revolute") {
            joint_info << "joint REVOLUTE with axis"<< axis.transpose();
            urdfVisitor.addJointAndBody(UrdfVisitor::REVOLUTE, axis,
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
            //joint_info<<"TODO: Fix BALL JOINT"<<std::endl;
            urdfVisitor.addJointAndBody(UrdfVisitor::SPHERICAL, axis,
                                        parentFrameId, jointPlacement, jointName,
                                        Y, cMj.inverse(), childName,
                                        max_effort, max_velocity, min_config, max_config,
                                        friction,damping);
          }
          else
          {
            joint_info<<"This type is yet to be implemented "<<jointElement->template Get<std::string>("type")<<std::endl;
          }

          if (not is_constraint) {
            // Add a recursion to the remaining children of the link just added.
            const std::vector<std::string>& childrenOfLink =
              childrenOfLinks.find(childName)->second;
            
            for(std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
                childOfChild != std::end(childrenOfLink); ++childOfChild) {
              const ::sdf::ElementPtr childOfChildElement =
                mapOfJoints.find(*childOfChild)->second;
              recursiveFillModel(childOfChildElement);
            }
          }
          else { // If it is a constraint, add the constraint and half the previous inertia.
            // Since Y_p is already present, halve it.
            //TODO: For multiple joints connected to the same link, halving might be a problem.
            //TODO: Handle inertias after full model is parsed.

            
            Inertia & Y_p = urdfVisitor.model.inertias[existingJointId];
            Y_p.mass() *= 0.5;
            Y_p.inertia() *= 0.5;

            //Get cMj of previous joint.
            const ::sdf::ElementPtr prevJointElement =
              mapOfJoints.find(urdfVisitor.getJointName(existingJointId))->second;
            SE3 cMj1(SE3::Identity());
            if (prevJointElement->HasElement("pose"))
            {
              const ignition::math::Pose3d prevcMj_ig =
                prevJointElement->template Get<ignition::math::Pose3d>("pose");
              cMj1 = ::pinocchio::sdf::details::convertFromPose3d(prevcMj_ig);
            }

            //Get joint Id that was just added:
            JointIndex currentJointId = urdfVisitor.getJointId(jointName);           
            ContactDetails rcm (::pinocchio::CONTACT_6D,
                                currentJointId,
                                cMj.inverse(),
                                existingJointId,
                                cMj1.inverse());
            rcm.name = childName+"_"+std::to_string(currentJointId)+"_"+std::to_string(existingJointId);
            contact_details.push_back(rcm);
          }
        }
      }; //Struct sdfGraph

      void PINOCCHIO_DLLAPI parseRootTree(SdfGraph& graph, const std::string& rootLinkName);
    } //namespace details

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & root_joint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
               const std::string rootLinkName,
               const bool verbose)
    {
      ::pinocchio::urdf::details::UrdfVisitorWithRootJoint<Scalar, Options,
                                                           JointCollectionTpl> visitor (model, root_joint);

      typedef ::pinocchio::sdf::details::SdfGraph SdfGraph;

      SdfGraph graph (visitor);
      if (verbose) visitor.log = &std::cout;

      //Create maps from the SDF Graph
      graph.parseGraph(filename);
      //Use the SDF graph to create the model
      details::parseRootTree(graph, rootLinkName);
      for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(SdfGraph::ContactDetails)::const_iterator
            cm = std::begin(graph.contact_details); cm != std::end(graph.contact_details); ++cm)
      {
        RigidConstraintModel rcm(cm->type, model, cm->joint1_id, cm->joint1_placement,
                                 cm->joint2_id, cm->joint2_placement, cm->reference_frame);
        rcm.name = cm->name;
        contact_models.push_back(rcm);
      }

      return model;
    }
    
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
               const std::string rootLinkName,
               const bool verbose)
    {
      typedef ::pinocchio::sdf::details::SdfGraph SdfGraph;
      
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
      SdfGraph graph (visitor);
      
      if (verbose) visitor.log = &std::cout;

      //Create maps from the SDF Graph
      graph.parseGraph(filename);
      //Use the SDF graph to create the model
      details::parseRootTree(graph, rootLinkName);

      for(PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(SdfGraph::ContactDetails)::const_iterator
            cm = std::begin(graph.contact_details); cm != std::end(graph.contact_details); ++cm)
      {
        RigidConstraintModel rcm(cm->type, model, cm->joint1_id, cm->joint1_placement,
                                 cm->joint2_id, cm->joint2_placement, cm->reference_frame);
        rcm.name = cm->name;
        contact_models.push_back(rcm);
      }
      
      return model;
    }
  }
}

#endif // ifndef __pinocchio_parsers_sdf_hpp__
