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
