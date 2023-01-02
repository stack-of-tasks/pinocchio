//
// Copyright (c) 2015-2023 CNRS INRIA
//

#ifndef __pinocchio_multibody_parsers_urdf_geometry_hxx__
#define __pinocchio_multibody_parsers_urdf_geometry_hxx__

#include "pinocchio/parsers/urdf.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include <hpp/fcl/mesh_loader/loader.h>
  #include <hpp/fcl/mesh_loader/assimp.h>
#endif // PINOCCHIO_WITH_HPP_FCL

namespace pinocchio
{
  namespace urdf
  {
    namespace details
    {
      struct UrdfGeomVisitorBase
      {
        typedef FrameTpl<urdf_scalar_type, 0> Frame;

        virtual Frame getBodyFrame (const std::string& name, FrameIndex& fid) const = 0;
      };

      template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
      struct UrdfGeomVisitor : UrdfGeomVisitorBase
      {
        typedef ModelTpl<_Scalar,_Options,JointCollectionTpl> Model;
        const Model& model;

        UrdfGeomVisitor (const Model& model) : model(model) {}

        Frame getBodyFrame (const std::string& link_name, FrameIndex& fid) const
        {
          if (!model.existFrame(link_name, BODY))
          {
            throw std::invalid_argument("No link " + link_name + " in model");
          }
          fid = model.getFrameId(link_name, BODY);
          PINOCCHIO_CHECK_INPUT_ARGUMENT(model.frames[fid].type == BODY);
          return model.frames[fid].template cast<urdf_scalar_type>();
        }
      };

      /**
       * @brief      Recursive procedure for reading the URDF tree, looking for geometries
       *             This function fill the geometric model whith geometry objects retrieved from the URDF tree
       *
       * @param[in]  tree           The URDF kinematic tree
       * @param[in]  meshLoader     The FCL mesh loader to avoid duplications of already loaded geometries
       * @param[in]  link           The current URDF link
       * @param      model          The model to which is the GeometryModel associated
       * @param      geomModel      The GeometryModel where the Collision Objects must be added
       * @param[in]  package_dirs   A vector containing the different directories where to search for packages
       * @param[in]  type           The type of objects that must be loaded ( can be VISUAL or COLLISION)
       *
       */
      PINOCCHIO_DLLAPI void parseTreeForGeom(UrdfGeomVisitorBase& visitor,
                                             const std::istream& xmlStream,
                                             const GeometryType type,
                                             GeometryModel & geomModel,
                                             const std::vector<std::string> & package_dirs,
                                             ::hpp::fcl::MeshLoaderPtr meshLoader);
      
      } // namespace details
      
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      GeometryModel& buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const std::string & filename,
                               const GeometryType type,
                               GeometryModel & geomModel,
                               const std::vector<std::string> & package_dirs,
                               ::hpp::fcl::MeshLoaderPtr meshLoader)
      {
        std::ifstream xmlStream(filename.c_str());
        if (! xmlStream.is_open())
        {
          const std::string exception_message (filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
        return buildGeom (model, xmlStream, type, geomModel, package_dirs, meshLoader);
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      GeometryModel& buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const std::istream& xmlStream,
                               const GeometryType type,
                               GeometryModel & geomModel,
                               const std::vector<std::string> & package_dirs,
                               ::hpp::fcl::MeshLoaderPtr meshLoader)
      {
        details::UrdfGeomVisitor<Scalar, Options, JointCollectionTpl> visitor (model);
        details::parseTreeForGeom (visitor, xmlStream, type, geomModel,
            package_dirs, meshLoader);
        return geomModel;
      }

  } // namespace urdf
} // namespace pinocchio
            
#endif // ifndef __pinocchio_multibody_parsers_urdf_geometry_hxx__
