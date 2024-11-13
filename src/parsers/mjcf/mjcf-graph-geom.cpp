//
// Copyright (c) 2016-2024 CNRS INRIA
//

#include "pinocchio/parsers/mjcf/mjcf-graph.hpp"
#ifdef PINOCCHIO_WITH_HPP_FCL

  #include <hpp/fcl/mesh_loader/loader.h>
  #include <hpp/fcl/mesh_loader/assimp.h>

#endif // PINOCCHIO_WITH_HPP_FCL

namespace pinocchio
{
  namespace mjcf
  {
    namespace details
    {
      static double computeVolume(const Eigen::VectorXd & size, const std::string & geomType)
      {
        double pi = boost::math::constants::pi<double>();
        if (geomType == "box")
        {
          return size.prod();
        }
        else if (geomType == "cylinder")
        {
          return 2 * pi * std::pow(size(0), 2) * size(1);
        }
        else if (geomType == "sphere")
        {
          return 4.0 / 3 * pi * std::pow(size(0), 3);
        }
        else if (geomType == "capsule")
        {
          return 4.0 / 3 * pi * std::pow(size(0), 3) + 2 * pi * std::pow(size(0), 2) * size(1);
        }
        else if (geomType == "ellipsoid")
        {
          return 4.0 / 3 * pi * size.prod();
        }
        else
        {
          throw std::invalid_argument("geometry type does not exist");
        }
      }

      bool isType(const MjcfGeom & geom, const GeometryType & type)
      {
        switch (type)
        {
        case ::pinocchio::COLLISION:
          return geom.geomKind != MjcfGeom::VISUAL;
        default:
        case ::pinocchio::VISUAL:
          return geom.geomKind != MjcfGeom::COLLISION;
        }
      }

      /**
       * @brief      Get a fcl::CollisionObject from an mjcf geometry, searching
       *             for it in specified package directories
       *
       * @param[in]  geom  A mjcf geometry object
       * @return     A shared pointer on the geometry converted as a fcl::CollisionGeometry
       */
#ifdef PINOCCHIO_WITH_HPP_FCL
      static std::shared_ptr<fcl::CollisionGeometry> retrieveCollisionGeometry(
        ::hpp::fcl::MeshLoaderPtr & meshLoader,
        const MjcfGeom & geom,
        const MjcfGraph & currentGraph,
        std::string & meshPath,
        Eigen::Vector3d & meshScale)
      {
        if (geom.geomType == "mesh")
        {
          MjcfMesh currentMesh = currentGraph.mapOfMeshes.at(geom.meshName);
          meshPath = currentMesh.filePath;
          meshScale = currentMesh.scale;
          hpp::fcl::BVHModelPtr_t bvh = meshLoader->load(meshPath, meshScale);
          return bvh;
        }
        else if (geom.geomType == "cylinder")
        {
          meshPath = "CYLINDER";
          meshScale << 1, 1, 1;
          double radius = geom.size(0);
          double height = geom.size(1) * 2;
          return std::make_shared<fcl::Cylinder>(radius, height);
        }
        else if (geom.geomType == "box")
        {
          meshPath = "BOX";
          meshScale << 1, 1, 1;
          double x = geom.size(0);
          double y = geom.size(1);
          double z = geom.size(2);
          return std::make_shared<fcl::Box>(x, y, z);
        }
        else if (geom.geomType == "sphere")
        {
          meshPath = "SPHERE";
          meshScale << 1, 1, 1;
          double radius = geom.size(0);
          return std::make_shared<fcl::Sphere>(radius);
        }
        else if (geom.geomType == "ellipsoid")
        {
          meshPath = "ELLIPSOID";
          meshScale << 1, 1, 1;
          double rx = geom.size(0);
          double ry = geom.size(1);
          double rz = geom.size(2);
          return std::make_shared<fcl::Ellipsoid>(rx, ry, rz);
        }
        else if (geom.geomType == "capsule")
        {
          meshPath = "CAPSULE";
          meshScale << 1, 1, 1;
          double radius = geom.size(0);
          double height = geom.size(1) * 2;
          return std::make_shared<fcl::Capsule>(radius, height);
        }
        else
          throw std::invalid_argument("Unknown geometry type :"); // Missing plane, hfield and sdf
      }
#else
      static std::shared_ptr<fcl::CollisionGeometry> retrieveCollisionGeometry(
        ::hpp::fcl::MeshLoaderPtr &,
        const MjcfGeom &,
        const MjcfGraph &,
        std::string &,
        Eigen::Vector3d &)
      {
        return std::make_shared<fcl::CollisionGeometry>();
      }
#endif

      /// @brief Add all geometries associated with a body
      /// @param currentBody Body from which geometries will be collected
      /// @param type Type of geometry to parse (COLLISION or VISUAL)
      /// @param geomModel geometry model to fill
      /// @param meshLoader mesh loader from hpp::fcl
      static void addLinksToGeomModel(
        const MjcfBody & currentBody,
        const MjcfGraph & currentGraph,
        GeometryModel & geomModel,
        ::hpp::fcl::MeshLoaderPtr & meshLoader,
        const GeometryType & type)
      {
        const std::string & bodyName = currentBody.bodyName;
        FrameIndex frame_id = currentGraph.urdfVisitor.getBodyId(bodyName);
        Frame frame = currentGraph.urdfVisitor.getBodyFrame(bodyName);

        const SE3 & bodyPlacement = frame.placement;

        std::size_t objectId = 0;
        for (const auto & geom : currentBody.geomChildren)
        {
          std::string meshPath = "";
          Eigen::Vector3d meshScale(Eigen::Vector3d::Ones());
          if (isType(geom, type))
          {
            const GeometryObject::CollisionGeometryPtr geometry =
              retrieveCollisionGeometry(meshLoader, geom, currentGraph, meshPath, meshScale);

            bool overrideMaterial = false;
            Eigen::Vector4d meshColor = geom.rgba;
            std::string texturePath;
            if (!geom.materialName.empty())
            {
              MjcfMaterial mat = currentGraph.mapOfMaterials.at(geom.materialName);
              meshColor = mat.rgba;
              overrideMaterial = true;
              if (!mat.texture.empty())
              {
                MjcfTexture text = currentGraph.mapOfTextures.at(mat.texture);
                texturePath = text.filePath;
              }
            }
            const SE3 geomPlacement = bodyPlacement * geom.geomPlacement;
            std::ostringstream geometry_object_suffix;
            geometry_object_suffix << "_" << objectId;
            std::string geometry_object_name = std::string(bodyName + geometry_object_suffix.str());

            GeometryObject geometry_object(
              geometry_object_name, frame.parentJoint, frame_id, geomPlacement, geometry, meshPath,
              meshScale, overrideMaterial, meshColor, texturePath);
            geomModel.addGeometryObject(geometry_object);
            ++objectId;
          }
        }
      }

      void MjcfMaterial::goThroughElement(const ptree & el)
      {
        // texture name
        auto text_s = el.get_optional<std::string>("<xmlattr>.texture");
        if (text_s)
          texture = *text_s;

        // Emission
        auto single_v = el.get_optional<float>("<xmlattr>.emission");
        if (single_v)
          emission = *single_v;

        // Specular
        if ((single_v = el.get_optional<float>("<xmlattr>.specular")))
          specular = *single_v;

        // Shininess
        if ((single_v = el.get_optional<float>("<xmlattr>.shininess")))
          shininess = *single_v;

        if ((single_v = el.get_optional<float>("<xmlattr>.reflectance")))
          reflectance = *single_v;

        auto rgba_ = el.get_optional<std::string>("<xmlattr>.rgba");
        if (rgba_)
          rgba = internal::getVectorFromStream<4>(*rgba_);
      }

      void MjcfGeom::goThroughElement(const ptree & el, const MjcfGraph & currentGraph)
      {
        if (el.get_child_optional("<xmlattr>.pos") && el.get_child_optional("<xmlattr>.fromto"))
          throw std::invalid_argument("Both pos and fromto are defined in geom object");

        // Placement
        geomPlacement = currentGraph.convertPosition(el);

        // density
        auto value_v = el.get_optional<double>("<xmlattr>.density");
        if (value_v)
          density = *value_v;

        // contype
        auto val_v = el.get_optional<int>("<xmlattr>.contype");
        if (val_v)
          contype = *val_v;
        // condim
        if ((val_v = el.get_optional<int>("<xmlattr>.conaffinity")))
          conaffinity = *val_v;
        // group
        if ((val_v = el.get_optional<int>("<xmlattr>.group")))
          group = *val_v;

        // mass
        massGeom = el.get_optional<double>("<xmlattr>.mass");

        // shellinertia
        auto value_s = el.get_optional<std::string>("<xmlattr>.shellinertia");
        if (value_s)
          throw std::invalid_argument("Shell inertia computation is not supported yet.");

        // type
        if ((value_s = el.get_optional<std::string>("<xmlattr>.type")))
          geomType = *value_s;

        // material name
        if ((value_s = el.get_optional<std::string>("<xmlattr>.material")))
          materialName = *value_s;

        if (geomType == "mesh")
        {
          value_s = el.get_optional<std::string>("<xmlattr>.mesh");
          if (value_s)
            meshName = *value_s;
        }

        // rgba
        if ((value_s = el.get_optional<std::string>("<xmlattr>.rgba")))
          rgba = internal::getVectorFromStream<4>(*value_s);

        // size
        if ((value_s = el.get_optional<std::string>("<xmlattr>.size")))
          sizeS = *value_s;
        // fromto
        if (!fromtoS)
          fromtoS = el.get_optional<std::string>("<xmlattr>.fromto");
        if (fromtoS)
        {
          Eigen::VectorXd poses = internal::getVectorFromStream<6>(*fromtoS);
          geomPlacement.translation() = (poses.head(3) + poses.tail(3)) / 2;

          Eigen::Vector3d zaxis = poses.tail(3) - poses.head(3);
          // Compute the rotation matrix that maps z_axis to unit z
          geomPlacement.rotation() =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), zaxis).toRotationMatrix();
        }
      }

      void MjcfGeom::findKind()
      {
        // Common mechanism to set visual only geometry
        if (contype == 0 && conaffinity == 0)
          geomKind = VISUAL;
        else if (group > 2) // Mechanism for Collision only geometries
          geomKind = COLLISION;
        else
          geomKind = BOTH;
      }

      void MjcfGeom::computeSize()
      {
        std::vector<std::string> forbiddenListFromTo = {"plane", "hfield", "mesh", "sphere"};
        if (
          fromtoS
          && std::find(forbiddenListFromTo.begin(), forbiddenListFromTo.end(), geomType)
               != forbiddenListFromTo.end())
          throw std::invalid_argument(
            "fromto tag can only be used with capsules, boxes, cylinders and ellipsoids");

        // First deal with cases that do not work with fromto
        if (geomType == "sphere")
          size = internal::getVectorFromStream<1>(sizeS);
        else if (geomType == "plane")
          size = internal::getVectorFromStream<3>(sizeS);
        else if (geomType == "box")
        {
          if (!fromtoS)
            size = internal::getVectorFromStream<3>(sizeS) * 2; // half x, half y, half z
          else
          {
            size = Eigen::Vector3d::Zero();
            size(0) = internal::getVectorFromStream<1>(sizeS)(0) * 2;
            size(1) = size(0);
            Eigen::VectorXd poses = internal::getVectorFromStream<6>(*fromtoS);
            Eigen::Vector3d zaxis = poses.tail(3) - poses.head(3);
            size(2) = zaxis.norm();
          }
        }
        else if (geomType == "ellipsoid")
        {
          if (!fromtoS)
            size = internal::getVectorFromStream<3>(sizeS); // radius and half length
          else
          {
            size = Eigen::Vector3d::Zero();
            size(0) = internal::getVectorFromStream<1>(sizeS)(0);
            size(1) = size(0);
            Eigen::VectorXd poses = internal::getVectorFromStream<6>(*fromtoS);
            Eigen::Vector3d zaxis = poses.tail(3) - poses.head(3);
            size(2) = zaxis.norm() / 2; // to get radius;
          }
        }
        else if (geomType == "cylinder" || geomType == "capsule")
        {
          if (!fromtoS)
            size = internal::getVectorFromStream<2>(sizeS);
          else
          {
            size = Eigen::Vector2d::Zero();
            size(0) = internal::getVectorFromStream<1>(sizeS)(0);
            Eigen::VectorXd poses = internal::getVectorFromStream<6>(*fromtoS);
            Eigen::Vector3d zaxis = poses.tail(3) - poses.head(3);
            size(1) = zaxis.norm() / 2; // to get half height
          }
        }
        else if (geomType == "mesh")
          return;
        else
          throw std::invalid_argument("geomType does not exist");
      }

      void MjcfGeom::computeInertia()
      {
        if (geomType == "mesh" || geomType == "plane" || geomType == "hfield")
          return;

        double mass;
        if (massGeom)
          mass = *massGeom;
        else
          mass = computeVolume(size, geomType) * density;

        if (geomType == "box")
        {
          geomInertia = Inertia::FromBox(mass, size(0), size(1), size(2));
        }
        else if (geomType == "cylinder")
        {
          geomInertia = Inertia::FromCylinder(mass, size(0), size(1) * 2);
        }
        else if (geomType == "ellipsoid")
        {
          geomInertia = Inertia::FromEllipsoid(mass, size(0), size(1), size(2));
        }
        else if (geomType == "sphere")
        {
          geomInertia = Inertia::FromSphere(mass, size(0));
        }
        else if (geomType == "capsule")
        {
          geomInertia = Inertia::FromCapsule(mass, size(0), size(1) * 2);
        }
        else
        {
          throw std::invalid_argument("Unsupported geometry type");
        }
      }

      void
      MjcfGeom::fill(const ptree & el, const MjcfBody & currentBody, const MjcfGraph & currentGraph)
      {
        // Name
        auto name_s = el.get_optional<std::string>("<xmlattr>.name");
        if (name_s)
          geomName = *name_s;
        else
          geomName =
            currentBody.bodyName + "Geom_" + std::to_string(currentBody.geomChildren.size());

        // default < ChildClass < Class < Real Joint
        if (currentGraph.mapOfClasses.find("mujoco_default") != currentGraph.mapOfClasses.end())
        {
          const MjcfClass & classD = currentGraph.mapOfClasses.at("mujoco_default");
          if (auto geom_p = classD.classElement.get_child_optional("geom"))
            goThroughElement(*geom_p, currentGraph);
        }
        //  childClass
        if (currentBody.childClass != "")
        {
          const MjcfClass & classE = currentGraph.mapOfClasses.at(currentBody.childClass);
          if (auto geom_p = classE.classElement.get_child_optional("geom"))
            goThroughElement(*geom_p, currentGraph);
        }

        // Class
        auto cl_s = el.get_optional<std::string>("<xmlattr>.class");
        if (cl_s)
        {
          std::string className = *cl_s;
          const MjcfClass & classE = currentGraph.mapOfClasses.at(className);
          if (auto geom_p = classE.classElement.get_child_optional("geom"))
            goThroughElement(*geom_p, currentGraph);
        }

        // Geom
        goThroughElement(el, currentGraph);
        if (geomType == "mesh" && meshName.empty())
          throw std::invalid_argument("Type is mesh but no mesh file were provided");
        // Compute Kind of geometry
        findKind();

        computeSize();

        // Compute Mass and inertia of geom object
        computeInertia();
      }

      void MjcfSite::goThroughElement(const ptree & el, const MjcfGraph & currentGraph)
      {
        if (el.get_child_optional("<xmlattr>.pos") && el.get_child_optional("<xmlattr>.fromto"))
          throw std::invalid_argument("Both pos and fromto are defined in site object");

        // Placement
        sitePlacement = currentGraph.convertPosition(el);

        auto fromtoS = el.get_optional<std::string>("<xmlattr>.fromto");
        if (fromtoS)
        {
          Eigen::VectorXd poses = internal::getVectorFromStream<6>(*fromtoS);
          sitePlacement.translation() = (poses.head(3) + poses.tail(3)) / 2;

          Eigen::Vector3d zaxis = poses.tail(3) - poses.head(3);
          // Compute the rotation matrix that maps z_axis to unit z
          sitePlacement.rotation() =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), zaxis).toRotationMatrix();
        }
      }

      void
      MjcfSite::fill(const ptree & el, const MjcfBody & currentBody, const MjcfGraph & currentGraph)
      {
        // Name
        auto name_s = el.get_optional<std::string>("<xmlattr>.name");
        if (name_s)
          siteName = *name_s;
        else
          siteName =
            currentBody.bodyName + "Site_" + std::to_string(currentBody.siteChildren.size());

        // default < ChildClass < Class < Real Joint
        if (currentGraph.mapOfClasses.find("mujoco_default") != currentGraph.mapOfClasses.end())
        {
          const MjcfClass & classD = currentGraph.mapOfClasses.at("mujoco_default");
          if (auto site_p = classD.classElement.get_child_optional("site"))
            goThroughElement(*site_p, currentGraph);
        }
        //  childClass
        if (currentBody.childClass != "")
        {
          const MjcfClass & classE = currentGraph.mapOfClasses.at(currentBody.childClass);
          if (auto site_p = classE.classElement.get_child_optional("site"))
            goThroughElement(*site_p, currentGraph);
        }

        // Class
        auto cl_s = el.get_optional<std::string>("<xmlattr>.class");
        if (cl_s)
        {
          std::string className = *cl_s;
          const MjcfClass & classE = currentGraph.mapOfClasses.at(className);
          if (auto site_p = classE.classElement.get_child_optional("site"))
            goThroughElement(*site_p, currentGraph);
        }

        // Site
        goThroughElement(el, currentGraph);
      }

      void MjcfGraph::parseGeomTree(
        const GeometryType & type,
        GeometryModel & geomModel,
        ::hpp::fcl::MeshLoaderPtr & meshLoader)
      {
#ifdef PINOCCHIO_WITH_HPP_FCL
        if (!meshLoader)
          meshLoader = std::make_shared<fcl::MeshLoader>(fcl::MeshLoader());
#endif // ifdef PINOCCHIO_WITH_HPP_FCL

        for (const auto & entry : bodiesList)
        {
          const MjcfBody & currentBody = mapOfBodies.at(entry);

          addLinksToGeomModel(currentBody, *this, geomModel, meshLoader, type);
        }
      }
    } // namespace details
  } // namespace mjcf
} // namespace pinocchio
