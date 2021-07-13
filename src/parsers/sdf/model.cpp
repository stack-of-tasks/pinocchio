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
#ifdef PINOCCHIO_WITH_HPP_FCL
      void parseRootTree(SdfGraph& graph)
      {
        //First joint connecting universe
        const ::sdf::ElementPtr jointElement = graph.mapOfJoints.find("static")->second;
        const std::string childName =
          jointElement->GetElement("child")->Get<std::string>();;
        const ::sdf::ElementPtr childElement = graph.mapOfLinks.find(childName)->second;
        const ::sdf::ElementPtr inertialElem = childElement->GetElement("inertial");
        const Inertia Y = ::pinocchio::sdf::details::convertInertiaFromSdf(inertialElem);

        graph.urdfVisitor.addRootJoint(convertInertiaFromSdf(inertialElem), childName);
        const std::vector<std::string>& childrenOfLink =
          graph.childrenOfLinks.find(childName)->second;
        for(std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
            childOfChild != std::end(childrenOfLink); ++childOfChild)
        {
          graph.recursiveFillModel(graph.mapOfJoints.find(*childOfChild)->second);
        }
      }
#endif
    }
  }
}
