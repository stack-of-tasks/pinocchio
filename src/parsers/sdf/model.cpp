//
// Copyright (c) 2021 CNRS
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

      const std::string findRootLink(const SdfGraph& graph)
      {
        std::string trial_link = graph.mapOfLinks.begin()->first;
        bool search_for_parent = true;
        while(search_for_parent) {
          const std::vector<std::string>& parents_of_links =
            graph.parentOfLinks.find(trial_link)->second;
          if (parents_of_links.size() ==0) {
            search_for_parent = false;
            std::cerr<<"Link "<<trial_link<<" is the root link"<<std::endl;
            return trial_link;
          }
          else {
            ::sdf::ElementPtr joint_element = graph.mapOfJoints.find(parents_of_links[0])->second;
            trial_link =
              joint_element->GetElement("parent")->template Get<std::string>();
          }
        }
      }
      
      void parseRootTree(SdfGraph& graph,
                         const std::string& rootLinkName)
      {
        //First joint connecting universe
        const ::sdf::ElementPtr rootLinkElement = graph.mapOfLinks.find(rootLinkName)->second;
        const ::sdf::ElementPtr inertialElem = rootLinkElement->GetElement("inertial");
        const Inertia Y = ::pinocchio::sdf::details::convertInertiaFromSdf(inertialElem);

        graph.urdfVisitor.addRootJoint(convertInertiaFromSdf(inertialElem), rootLinkName);
        const std::vector<std::string>& childrenOfLink =
          graph.childrenOfLinks.find(rootLinkName)->second;
        for(std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
            childOfChild != std::end(childrenOfLink); ++childOfChild)
        {
          graph.recursiveFillModel(graph.mapOfJoints.find(*childOfChild)->second);
        }
      }
    }
  }
}
