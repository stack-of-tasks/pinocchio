//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_multibody_parsers_sdf_model_hxx__
#define __pinocchio_multibody_parsers_sdf_model_hxx__

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/parsers/config.hpp"
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

      static SE3 convertFromPose3d(const ignition::math::Pose3d & posePlacement)
      {
        const ignition::math::Quaterniond & q = posePlacement.Rot();
        const ignition::math::Vector3d & p = posePlacement.Pos();
        return SE3(
          SE3::Quaternion(q.W(), q.X(), q.Y(), q.Z()).matrix(), SE3::Vector3(p.X(), p.Y(), p.Z()));
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

        const ignition::math::Pose3d & pose =
          inertial->template Get<ignition::math::Pose3d>("pose");
        const double & mass = inertial->template Get<double>("mass");

        const ::sdf::ElementPtr inertiaElem = inertial->GetElement("inertia");
        const double ixx = inertiaElem->template Get<double>("ixx");
        const double ixy = inertiaElem->template Get<double>("ixy");
        const double ixz = inertiaElem->template Get<double>("ixz");
        const double iyy = inertiaElem->template Get<double>("iyy");
        const double iyz = inertiaElem->template Get<double>("iyz");
        const double izz = inertiaElem->template Get<double>("izz");

        const Inertia::Vector3 com(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        const Inertia::Matrix3 & R =
          Eigen::Quaterniond(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z())
            .matrix();

        Inertia::Matrix3 I;
        I << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;

        return Inertia(mass, com, R * I * R.transpose());
      }

      template<typename Scalar, int Options>
      struct ContactDetailsTpl
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef SE3Tpl<Scalar, Options> SE3;
        typedef pinocchio::JointIndex JointIndex;

        ///  \brief Name of the contact.
        std::string name;

        ///  \brief Type of the contact.
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

        ContactDetailsTpl(
          const ContactType type,
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
        {
        }
      };

      struct SdfGraph
      {
      public:
        typedef std::map<std::string, ::sdf::ElementPtr> ElementMap_t;
        typedef std::map<
          std::string,
          SE3,
          std::less<std::string>,
          Eigen::aligned_allocator<std::pair<const std::string, SE3>>>
          ChildPoseMap;

        typedef std::map<std::string, std::vector<std::string>> StringVectorMap_t;
        typedef std::vector<std::string> VectorOfStrings;
        typedef std::vector<JointIndex> VectorOfIndexes;
        typedef ContactDetailsTpl<double, 0> ContactDetails;

        ElementMap_t mapOfLinks, mapOfJoints;
        StringVectorMap_t childrenOfLinks;
        StringVectorMap_t parentOfLinks;
        VectorOfStrings linksWithMultipleParents;
        VectorOfStrings parentGuidance;
        VectorOfIndexes parentOrderWithGuidance;
        ChildPoseMap childPoseMap;
        std::string modelName;

        typedef pinocchio::urdf::details::
          UrdfVisitor<double, 0, ::pinocchio::JointCollectionDefaultTpl>
            UrdfVisitor;
        UrdfVisitor & urdfVisitor;
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails) contact_details;

        SdfGraph(UrdfVisitor & urdfVisitor)
        : urdfVisitor(urdfVisitor)
        {
        }

        void setParentGuidance(const VectorOfStrings & parentGuidance_in)
        {
          parentGuidance = parentGuidance_in;
        }

        void parseGraphFromXML(const std::string & xmlString)
        {
          // load and check sdf file
          ::sdf::SDFPtr sdfElement(new ::sdf::SDF());
          ::sdf::init(sdfElement);
          if (!::sdf::readString(xmlString, sdfElement))
          {
            throw std::invalid_argument("The xml string does not "
                                        "contain a valid SDF model");
          }
          parseGraph(sdfElement);
        }

        void parseGraphFromFile(const std::string & filename)
        {
          // load and check sdf file
          ::sdf::SDFPtr sdfElement(new ::sdf::SDF());
          ::sdf::init(sdfElement);
          if (!::sdf::readFile(filename, sdfElement))
          {
            throw std::invalid_argument(
              "The file " + filename
              + " does not "
                "contain a valid SDF model");
          }
          parseGraph(sdfElement);
        }

        void parseGraph(::sdf::SDFPtr sdfElement)
        {
          // start parsing model
          const ::sdf::ElementPtr rootElement = sdfElement->Root();

          if (!rootElement->HasElement("model"))
          {
            throw std::invalid_argument("The sdf model does not "
                                        "contain model element");
          }

          const ::sdf::ElementPtr modelElement = rootElement->GetElement("model");

          modelName = modelElement->template Get<std::string>("name");
          urdfVisitor.setName(modelName);

          // parse model links
          ::sdf::ElementPtr linkElement = modelElement->GetElement("link");
          while (linkElement)
          {
            const std::string linkName = linkElement->Get<std::string>("name");
            // Inserting data in std::map
            mapOfLinks.insert(std::make_pair(linkName, linkElement));
            childrenOfLinks.insert(std::make_pair(linkName, std::vector<std::string>()));
            parentOfLinks.insert(std::make_pair(linkName, std::vector<std::string>()));
            linkElement = linkElement->GetNextElement("link");
          }

          // parse model joints
          ::sdf::ElementPtr jointElement = modelElement->GetElement("joint");
          while (jointElement)
          {
            const std::string jointName = jointElement->template Get<std::string>("name");
            std::string parentLinkName =
              jointElement->GetElement("parent")->template Get<std::string>();
            std::string childLinkName =
              jointElement->GetElement("child")->template Get<std::string>();
            // Inserting data in std::map
            StringVectorMap_t::const_iterator parent_link = childrenOfLinks.find(parentLinkName);
            if (parent_link == childrenOfLinks.end())
            {
              const std::string msg = "Parent of " + jointName + " doesn't exist";
              throw std::invalid_argument(msg);
            }

            mapOfJoints.insert(std::make_pair(jointName, jointElement));
            // Create data of children of links
            childrenOfLinks.find(parentLinkName)->second.push_back(jointName);
            parentOfLinks.find(childLinkName)->second.push_back(jointName);
            jointElement = jointElement->GetNextElement("joint");
          }

          for (StringVectorMap_t::const_iterator linkMap = parentOfLinks.begin();
               linkMap != parentOfLinks.end(); ++linkMap)
          {
            const std::string linkName = linkMap->first;
            const VectorOfStrings & parents = linkMap->second;
            if (parents.size() >= 2)
            {
              linksWithMultipleParents.push_back(linkName);
              urdfVisitor << linkName << " has " << parents.size() << " parents" << '\n';

              // Find which parent would become the chain creator:
              JointIndex parentOrder = 0;
              for (VectorOfStrings::const_iterator parentJoint = std::begin(parents);
                   parentJoint != std::end(parents); ++parentJoint)
              {
                VectorOfStrings::const_iterator p_it =
                  std::find(parentGuidance.cbegin(), parentGuidance.cend(), *parentJoint);
                if (p_it != parentGuidance.end())
                {
                  parentOrder =
                    static_cast<JointIndex>(std::distance(parents.cbegin(), parentJoint));
                  break;
                }
              }
              parentOrderWithGuidance.push_back(parentOrder);
            }
          }
        }

        static bool existConstraint(
          const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails) & contact_details,
          const std::string & jointName)
        {
          for (PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)::const_iterator cm =
                 std::begin(contact_details);
               cm != std::end(contact_details); ++cm)
          {
            if (cm->name == jointName)
              return true;
          }
          return false;
        }

        static bool
        existChildName(const std::vector<std::string> & listOfNames, const std::string & childName)
        {
          for (std::vector<std::string>::const_iterator cm = std::begin(listOfNames);
               cm != std::end(listOfNames); ++cm)
          {
            if ((*cm) == childName)
              return true;
          }
          return false;
        }

        static int getConstraintId(
          const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails) & contact_details,
          const std::string & jointName)
        {
          std::size_t i = 0;
          for (PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)::const_iterator cm =
                 std::begin(contact_details);
               cm != std::end(contact_details); ++cm)
          {
            if (cm->name == jointName)
              return static_cast<int>(i);
            i++;
          }
          return -1;
        }

        int getConstraintIdFromChild(
          const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails) & contact_details,
          const std::string & childName)
        {
          std::size_t i = 0;
          for (PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactDetails)::const_iterator cm =
                 std::begin(contact_details);
               cm != std::end(contact_details); ++cm)
          {
            const std::string childFromMap =
              mapOfJoints.find(cm->name)->second->GetElement("child")->Get<std::string>();
            if (childFromMap == childName)
              return static_cast<int>(i);
            i++;
          }
          return -1;
        }

        ///
        /// \brief Recursive procedure for reading the SDF tree.
        ///        The function returns an exception as soon as a necessary Inertia or Joint
        ///        information are missing.
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
          const std::string & jointName = jointElement->template Get<std::string>("name");

          urdfVisitor << jointName << " being parsed." << '\n';

          const std::string & parentName = jointElement->GetElement("parent")->Get<std::string>();
          const std::string & childNameOrig = jointElement->GetElement("child")->Get<std::string>();

          bool multiple_parents = false;
          bool make_parent = true;
          int nParents = 1;

          JointIndex parentOrderId, link_index, currentJointOrderId;
          // Find is the link has multiple parents
          // If yes, one parent would create chain, while other parents would create constraints
          // If there is guidance, use guidance to choose parent which creates chain
          // If there is no guidance, use first element to create chain.

          VectorOfStrings::const_iterator linkHasParents = std::find(
            linksWithMultipleParents.cbegin(), linksWithMultipleParents.cend(), childNameOrig);
          if (linkHasParents != linksWithMultipleParents.end())
          {
            link_index = static_cast<JointIndex>(
              std::distance(linksWithMultipleParents.cbegin(), linkHasParents));
            parentOrderId = (parentOrderWithGuidance.at(link_index));
            multiple_parents = true;
            const VectorOfStrings & parentsOfChild = parentOfLinks.find(childNameOrig)->second;

            VectorOfStrings::const_iterator currentJointIt =
              std::find(parentsOfChild.cbegin(), parentsOfChild.cend(), jointName);
            currentJointOrderId =
              static_cast<JointIndex>(std::distance(parentsOfChild.cbegin(), currentJointIt));

            if (jointName == parentsOfChild.at(parentOrderId))
            {
              make_parent = true;
            }
            else
            {
              make_parent = false;
            }
            nParents = static_cast<int>(parentsOfChild.size());
          }

          std::string childName = childNameOrig;
          if (not make_parent and multiple_parents)
          {
            childName += "_" + jointName;
          }

          const ::sdf::ElementPtr childElement = mapOfLinks.find(childNameOrig)->second;
          const ::sdf::ElementPtr parentElement = mapOfLinks.find(parentName)->second;
          const ::sdf::ElementPtr parentLinkPoseElem = parentElement->GetElement("pose");
          const ::sdf::ElementPtr childLinkPoseElem = childElement->GetElement("pose");
          const ::sdf::ElementPtr jointPoseElem = jointElement->GetElement("pose");

          const ignition::math::Pose3d parentLinkPlacement_ig =
            parentElement->template Get<ignition::math::Pose3d>("pose");

          const ignition::math::Pose3d childLinkPlacement_ig =
            childElement->template Get<ignition::math::Pose3d>("pose");

          const ignition::math::Pose3d curJointPlacement_ig =
            jointElement->template Get<ignition::math::Pose3d>("pose");

          const SE3 parentLinkPlacement =
            ::pinocchio::sdf::details::convertFromPose3d(parentLinkPlacement_ig);
          const SE3 childLinkPlacement =
            ::pinocchio::sdf::details::convertFromPose3d(childLinkPlacement_ig);
          const SE3 curJointPlacement =
            ::pinocchio::sdf::details::convertFromPose3d(curJointPlacement_ig);

          const JointIndex parentJointId = urdfVisitor.getParentId(parentName);
          const std::string & parentJointName = urdfVisitor.getJointName(parentJointId);

          SE3 cMj(SE3::Identity()), pMjp(SE3::Identity()), oMc(SE3::Identity()),
            pMj(SE3::Identity());

          // Find pose of parent link w.r.t. parent joint.
          if (parentJointName != "root_joint" && parentJointName != "universe")
          {
            const ::sdf::ElementPtr parentJointElement = mapOfJoints.find(parentJointName)->second;

            const ::sdf::ElementPtr parentJointPoseElem = parentJointElement->GetElement("pose");

            const ignition::math::Pose3d parentJointPoseElem_ig =
              parentJointElement->template Get<ignition::math::Pose3d>("pose");

            const std::string relativeFrame =
              parentJointPoseElem->template Get<std::string>("relative_to");
            const std::string parentJointParentName =
              parentJointElement->GetElement("parent")->Get<std::string>();

            if (not relativeFrame.compare(parentJointParentName))
            { // If they are equal

              // Pose is relative to Parent joint's parent. Search in parent link instead.
              const std::string & parentLinkRelativeFrame =
                parentLinkPoseElem->template Get<std::string>("relative_to");

              // If the pMjp is not found, throw
              PINOCCHIO_THROW(
                not parentLinkRelativeFrame.compare(parentJointName), std::logic_error,
                parentName + " pose is not defined w.r.t. parent joint");

              pMjp = parentLinkPlacement.inverse();
            }
            else
            { // If the relative_to is not the parent
              // The joint pose is defined w.r.t to the child, as per the SDF standard < 1.7
              pMjp = ::pinocchio::sdf::details::convertFromPose3d(parentJointPoseElem_ig);
            }
          }

          // Find Pose of current joint w.r.t. child link, e.t. cMj;
          const std::string & curJointRelativeFrame =
            jointPoseElem->template Get<std::string>("relative_to");
          const std::string & childLinkRelativeFrame =
            childLinkPoseElem->template Get<std::string>("relative_to");

          if (not curJointRelativeFrame.compare(parentName))
          { // If they are equal
            pMj = curJointPlacement;
          }
          else
          {
            cMj = curJointPlacement;
          }

          if (not childLinkRelativeFrame.compare(jointName))
          { // If they are equal
            cMj = childLinkPlacement.inverse();
          }
          else
          {
            oMc = childLinkPlacement;
            pMj = parentLinkPlacement.inverse() * childLinkPlacement * cMj;
          }

          // const SE3 jointPlacement = pMjp.inverse() * pMj;

          urdfVisitor << "Joint " << jointName << " connects parent " << parentName << " link"
                      << " with parent joint " << parentJointName << " to child " << childNameOrig
                      << " link"
                      << " with joint type " << jointElement->template Get<std::string>("type")
                      << '\n';
          const Scalar infty = std::numeric_limits<Scalar>::infinity();
          FrameIndex parentFrameId = urdfVisitor.getBodyId(parentName);
          Vector max_effort(Vector::Constant(1, infty)), max_velocity(Vector::Constant(1, infty)),
            min_config(Vector::Constant(1, -infty)), max_config(Vector::Constant(1, infty));
          Vector spring_stiffness(1), spring_reference(1);
          Vector friction(Vector::Constant(1, 0.)), damping(Vector::Constant(1, 0.));
          ignition::math::Vector3d axis_ignition;
          Vector3 axis;
          bool axis_found = false;

          if (jointElement->HasElement("axis"))
          {
            axis_found = true;
            const ::sdf::ElementPtr axisElem = jointElement->GetElement("axis");
            const ::sdf::ElementPtr xyzElem = axisElem->GetElement("xyz");
            axis_ignition = axisElem->Get<ignition::math::Vector3d>("xyz");
            axis << axis_ignition.X(), axis_ignition.Y(), axis_ignition.Z();

            // if use_parent_model_frame has been set to true
            if (xyzElem->HasAttribute("expressed_in"))
            {
              const std::string parentModelFrame =
                xyzElem->template Get<std::string>("expressed_in");
              if (parentModelFrame == "__model__")
              {
                axis = childLinkPlacement.rotation().inverse() * axis;
              }
            }

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

          const ::sdf::ElementPtr inertialElem = childElement->GetElement("inertial");
          Inertia Y = ::pinocchio::sdf::details::convertInertiaFromSdf(inertialElem);

          Y.mass() *= 1.0 / (double)nParents;
          Y.inertia() *= 1.0 / (double)nParents;

          if (jointElement->template Get<std::string>("type") == "universal")
          {
          }
          else if (jointElement->template Get<std::string>("type") == "revolute")
          {
            if (not axis_found)
            {
              const std::string msg("Axis information missing in joint " + jointName);
              throw std::invalid_argument(msg);
            }

            urdfVisitor << "joint REVOLUTE with axis" << axis.transpose() << '\n';
            urdfVisitor.addJointAndBody(
              UrdfVisitor::REVOLUTE, axis, parentFrameId, pMj, jointName, Y, cMj.inverse(),
              childName, max_effort, max_velocity, min_config, max_config, friction, damping);
          }
          else if (jointElement->template Get<std::string>("type") == "gearbox")
          {
            urdfVisitor << "joint GEARBOX with axis" << '\n';
            urdfVisitor.addJointAndBody(
              UrdfVisitor::REVOLUTE, axis, parentFrameId, pMj, jointName, Y, cMj.inverse(),
              childName, max_effort, max_velocity, min_config, max_config, friction, damping);
          }
          else if (jointElement->template Get<std::string>("type") == "prismatic")
          {
            if (not axis_found)
            {
              const std::string msg("Axis information missing in joint " + jointName);
              throw std::invalid_argument(msg);
            }

            urdfVisitor << "joint prismatic with axis" << '\n';
            urdfVisitor.addJointAndBody(
              UrdfVisitor::PRISMATIC, axis, parentFrameId, pMj, jointName, Y, cMj.inverse(),
              childName, max_effort, max_velocity, min_config, max_config, friction, damping);
          }
          else if (jointElement->template Get<std::string>("type") == "fixed")
          {
            urdfVisitor << "joint fixed" << '\n';
            urdfVisitor.addFixedJointAndBody(parentFrameId, pMj, jointName, Y, childName);
          }
          else if (jointElement->template Get<std::string>("type") == "ball")
          {
            max_effort = Vector::Constant(3, infty);
            max_velocity = Vector::Constant(3, infty);
            min_config = Vector::Constant(4, -infty);
            max_config = Vector::Constant(4, infty);
            min_config.setConstant(-1.01);
            max_config.setConstant(1.01);
            friction = Vector::Constant(3, 0.);
            damping = Vector::Constant(3, 0.);

            urdfVisitor << "joint BALL" << '\n';
            urdfVisitor.addJointAndBody(
              UrdfVisitor::SPHERICAL, axis, parentFrameId, pMj, jointName, Y, cMj.inverse(),
              childName, max_effort, max_velocity, min_config, max_config, friction, damping);
          }
          else
          {
            const std::string msg = "This type is yet to be implemented "
                                    + jointElement->template Get<std::string>("type");
            urdfVisitor << msg << '\n';
            throw std::invalid_argument(msg);
          }

          SE3 cMj1(SE3::Identity());
          JointIndex existingJointId = 0;
          std::string constraint_name;
          // Get joint Id that was just added:

          if (make_parent)
          {
            if (multiple_parents)
            {
              childPoseMap.insert(std::make_pair(childNameOrig, cMj));
            }
            // Add a recursion to the remaining children of the link just added.
            const std::vector<std::string> & childrenOfLink =
              childrenOfLinks.find(childNameOrig)->second;

            for (std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
                 childOfChild != std::end(childrenOfLink); ++childOfChild)
            {
              const ::sdf::ElementPtr childOfChildElement = mapOfJoints.find(*childOfChild)->second;
              recursiveFillModel(childOfChildElement);
            }
          }
          else
          {

            // If there are multiple parents, use parent guidance to choose which parent to use.
            // By default, it uses the first parent that is parsed in parentsOfLink.
            // The inertia values are already divided by nParents before addJointAndBody.
            // One parent creates the kinematic chain, while all other parents create constraints

            if (not multiple_parents)
            {
              assert(true && "Should not happen.");
            }
            const JointIndex currentAddedJointId = urdfVisitor.getJointId(jointName);
            ContactDetails rcm(
              ::pinocchio::CONTACT_6D, currentAddedJointId, cMj.inverse(), existingJointId,
              cMj1.inverse());
            rcm.name = childNameOrig;
            contact_details.push_back(rcm);
          }
        }
      }; // Struct sdfGraph

      PINOCCHIO_PARSERS_DLLAPI void
      parseRootTree(SdfGraph & graph, const std::string & rootLinkName);
      PINOCCHIO_PARSERS_DLLAPI void parseContactInformation(
        const SdfGraph & graph,
        const urdf::details::UrdfVisitorBase & visitor,
        const Model & model,
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models);

      /**
       * @brief Find the parent of all elements, the root link, and return it.
       *
       * @param[in] filename     SDF rootLinkName
       *
       */
      PINOCCHIO_PARSERS_DLLAPI const std::string findRootLink(const SdfGraph & graph);

    } // namespace details

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & root_joint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName,
      const std::vector<std::string> & parentGuidance,
      const bool verbose,
      const std::string jointRootName)
    {
      ::pinocchio::urdf::details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl>
        visitor(model, root_joint, jointRootName);

      typedef ::pinocchio::sdf::details::SdfGraph SdfGraph;

      SdfGraph graph(visitor);
      if (verbose)
        visitor.log = &std::cout;

      graph.setParentGuidance(parentGuidance);

      // Create maps from the SDF Graph
      graph.parseGraphFromXML(xmlStream);

      if (rootLinkName == "")
      {
        const_cast<std::string &>(rootLinkName) = details::findRootLink(graph);
      }

      // Use the SDF graph to create the model
      details::parseRootTree(graph, rootLinkName);
      details::parseContactInformation(graph, visitor, model, contact_models);

      return model;
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & root_joint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName,
      const std::vector<std::string> & parentGuidance,
      const bool verbose,
      const std::string jointRootName)
    {
      ::pinocchio::urdf::details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl>
        visitor(model, root_joint, jointRootName);

      typedef ::pinocchio::sdf::details::SdfGraph SdfGraph;

      SdfGraph graph(visitor);
      if (verbose)
        visitor.log = &std::cout;

      graph.setParentGuidance(parentGuidance);

      // Create maps from the SDF Graph
      graph.parseGraphFromFile(filename);

      if (rootLinkName == "")
      {
        const_cast<std::string &>(rootLinkName) = details::findRootLink(graph);
      }

      // Use the SDF graph to create the model
      details::parseRootTree(graph, rootLinkName);
      details::parseContactInformation(graph, visitor, model, contact_models);

      return model;
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName,
      const std::vector<std::string> & parentGuidance,
      const bool verbose)
    {
      typedef ::pinocchio::sdf::details::SdfGraph SdfGraph;

      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor(model);
      SdfGraph graph(visitor);

      graph.setParentGuidance(parentGuidance);

      if (verbose)
        visitor.log = &std::cout;

      // Create maps from the SDF Graph
      graph.parseGraphFromXML(xmlStream);

      if (rootLinkName == "")
      {
        const_cast<std::string &>(rootLinkName) = details::findRootLink(graph);
      }

      // Use the SDF graph to create the model
      details::parseRootTree(graph, rootLinkName);
      details::parseContactInformation(graph, visitor, model, contact_models);

      return model;
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName,
      const std::vector<std::string> & parentGuidance,
      const bool verbose)
    {
      typedef ::pinocchio::sdf::details::SdfGraph SdfGraph;

      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor(model);
      SdfGraph graph(visitor);

      graph.setParentGuidance(parentGuidance);

      if (verbose)
        visitor.log = &std::cout;

      // Create maps from the SDF Graph
      graph.parseGraphFromFile(filename);

      if (rootLinkName == "")
      {
        const_cast<std::string &>(rootLinkName) = details::findRootLink(graph);
      }

      // Use the SDF graph to create the model
      details::parseRootTree(graph, rootLinkName);
      details::parseContactInformation(graph, visitor, model, contact_models);

      return model;
    }
  } // namespace sdf
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_sdf_hpp__
