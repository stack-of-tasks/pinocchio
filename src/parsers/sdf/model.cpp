//
// Copyright (c) 2021-2022 INRIA
//

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

      const std::string findRootLink(const SdfGraph & graph)
      {
        std::string trial_link = graph.mapOfLinks.begin()->first;
        bool search_for_parent = true;
        while (search_for_parent)
        {
          const std::vector<std::string> & parents_of_links =
            graph.parentOfLinks.find(trial_link)->second;
          if (parents_of_links.size() == 0)
          {
            search_for_parent = false;
            return trial_link;
          }
          else
          {
            ::sdf::ElementPtr joint_element = graph.mapOfJoints.find(parents_of_links[0])->second;
            trial_link = joint_element->GetElement("parent")->template Get<std::string>();
          }
        }
        return std::string("");
      }

      void parseContactInformation(
        const SdfGraph & graph,
        const urdf::details::UrdfVisitorBase & visitor,
        const Model & model,
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models)
      {
        for (PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(
               SdfGraph::ContactDetails)::const_iterator cm = std::cbegin(graph.contact_details);
             cm != std::cend(graph.contact_details); ++cm)
        {
          // Get Link Name, and Link Pose, and set the values here:
          const JointIndex joint_id = visitor.getParentId(cm->name);
          const SE3 & cMj = graph.childPoseMap.find(cm->name)->second;

          RigidConstraintModel rcm(
            cm->type, model, cm->joint1_id, cm->joint1_placement, joint_id, cMj.inverse(),
            cm->reference_frame);

          contact_models.push_back(rcm);
        }
      }

      void parseRootTree(SdfGraph & graph, const std::string & rootLinkName)
      {
        // First joint connecting universe
        const ::sdf::ElementPtr rootLinkElement = graph.mapOfLinks.find(rootLinkName)->second;
        const ::sdf::ElementPtr inertialElem = rootLinkElement->GetElement("inertial");

        graph.urdfVisitor.addRootJoint(convertInertiaFromSdf(inertialElem), rootLinkName);
        const std::vector<std::string> & childrenOfLink =
          graph.childrenOfLinks.find(rootLinkName)->second;
        for (std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
             childOfChild != std::end(childrenOfLink); ++childOfChild)
        {
          graph.recursiveFillModel(graph.mapOfJoints.find(*childOfChild)->second);
        }
      }
    } // namespace details
  } // namespace sdf
} // namespace pinocchio
