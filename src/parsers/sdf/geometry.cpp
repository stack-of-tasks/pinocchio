//
// Copyright (c) 2021 CNRS
//

#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/parsers/utils.hpp"

#include <sstream>
#include <iomanip>
#include <boost/shared_ptr.hpp>

#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/mesh_loader/assimp.h>


namespace pinocchio
{
  namespace sdf
  {
    namespace details
    {

      void addLinkGeometryToGeomModel(const SdfGraph& graph,
                                      ::hpp::fcl::MeshLoaderPtr& meshLoader,
                                      const ::sdf::ElementPtr link,
                                      GeometryModel & geomModel,
                                      const std::vector<std::string> & package_dirs,
                                      const GeometryType type)
      {
        const std::string& linkName = link->Get<std::string>("name");
        switch(type)
        {
          case COLLISION:
            addLinkGeometryToGeomModel< ::pinocchio::COLLISION >(graph, meshLoader, link,
                                                                 geomModel, package_dirs);
            break;
          case VISUAL:
            addLinkGeometryToGeomModel< ::pinocchio::VISUAL >(graph, meshLoader, link,
                                                              geomModel, package_dirs);
            break;
          default:
            break;
        }
      }
      
      void parseTreeForGeom(const Model& model,
                            const SdfGraph& graph,
                            GeometryModel & geomModel,
                            const std::string& rootLinkName,
                            const GeometryType type,
                            const std::vector<std::string> & package_dirs,
                            ::hpp::fcl::MeshLoaderPtr meshLoader)
      {
        std::vector<std::string> hint_directories(package_dirs);
        std::vector<std::string> ros_pkg_paths = rosPaths();
        hint_directories.insert(hint_directories.end(), ros_pkg_paths.begin(),
                                ros_pkg_paths.end());
        
        if (!meshLoader) meshLoader = fcl::MeshLoaderPtr(new fcl::MeshLoader);

        const ::sdf::ElementPtr rootElement = graph.mapOfLinks.find(rootLinkName)->second;

        addLinkGeometryToGeomModel(graph, meshLoader, rootElement,
                                   geomModel, hint_directories, type);
        
        for(std::vector<std::string>::const_iterator joint_name =
              std::begin(model.names);
            joint_name != std::end(model.names); ++joint_name)
        {
          if (graph.mapOfJoints.find(*joint_name) == graph.mapOfJoints.end())
          {
            continue;
          }
          const ::sdf::ElementPtr childJointElement =
            graph.mapOfJoints.find(*joint_name)->second;
          const std::string childLinkName =
            childJointElement->GetElement("child")->template Get<std::string>();
          const ::sdf::ElementPtr childLinkElement =
            graph.mapOfLinks.find(childLinkName)->second;

          addLinkGeometryToGeomModel(graph, meshLoader, childLinkElement,
                                     geomModel, hint_directories,type);
        }        
      }
    } // namespace details    
  } // namespace sdf
} // namespace pinocchio
