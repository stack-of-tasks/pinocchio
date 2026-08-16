//
// Copyright (c) 2016-2024 CNRS INRIA
//
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/filesystem/path.hpp>
#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/variant/get.hpp>

#include "pinocchio/constraints.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial.hpp"

namespace pinocchio
{
  namespace mjcf
  {
    namespace details
    {
      typedef boost::property_tree::ptree ptree;

      // supported elements from mjcf
      static const std::array<std::string, 3> ELEMENTS = {"joint", "geom", "site"};

      /// @brief Copy the value of ptree src into dst
      /// @param src ptree to copy
      /// @param dst ptree where copy is made
      static void copyPtree(const ptree & src, ptree & dst)
      {
        for (const ptree::value_type & v : src)
          dst.put(ptree::path_type(v.first), v.second.data());
      }

      /// @brief Update class Element in order to have all info of parent classes
      /// @param current current class
      /// @param dst parent class
      static void updateClassElement(ptree & current, const ptree & parent)
      {
        for (const std::string & el : ELEMENTS)
        {
          std::string path = el + ".<xmlattr>";
          if (parent.get_child_optional(path))
          {
            const ptree default_value = ptree();
            ptree attr_parent = parent.get_child(path, default_value);
            const ptree & attr_current = current.get_child(path, default_value);
            // To only copy non existing attribute in current, we copy all current
            // attribute (replacing) into a parent copy then we replace current with the new
            // ptree
            copyPtree(attr_current, attr_parent);
            current.put_child(path, attr_parent);
          }
        }
      }

      static std::string getName(const ptree & el, const boost::filesystem::path & filePath)
      {
        auto n = el.get_optional<std::string>("<xmlattr>.name");
        if (n)
          return *n;
        else
        {
          if (filePath.extension().empty())
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Cannot find extension for one of the mesh/texture");

          auto st = filePath.stem();
          if (!st.empty())
            return st.string();
          else
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Cannot find a name for one of the mesh.texture");
        }
      }

      static boost::filesystem::path updatePath(
        bool strippath,
        const std::string & dir,
        const std::string & modelPath,
        const boost::filesystem::path & filePath)
      {
        namespace fs = boost::filesystem;

        // Check if filename still has Absolute path, like said in mujoco doc
        if (filePath.is_absolute() && !strippath)
          return filePath;
        else
        {
          auto filename = filePath;
          if (strippath)
            filename = filePath.filename();

          fs::path meshPath(dir);
          if (meshPath.is_absolute())
            return (meshPath / filename);
          else
          {
            fs::path mainPath(modelPath);
            return (mainPath.parent_path() / meshPath / filename);
          }
        }
      }

      double MjcfCompiler::convertAngle(const double & angle_) const
      {
        return angle_ * angle_converter;
      }

      Eigen::Matrix3d MjcfCompiler::convertEuler(const Eigen::Vector3d & angles) const
      {
        Eigen::Matrix3d aa1 =
          Eigen::AngleAxisd(convertAngle(angles(0)), mapEulerAngles.col(0)).toRotationMatrix();
        Eigen::Matrix3d aa2 =
          Eigen::AngleAxisd(convertAngle(angles(1)), mapEulerAngles.col(1)).toRotationMatrix();
        Eigen::Matrix3d aa3 =
          Eigen::AngleAxisd(convertAngle(angles(2)), mapEulerAngles.col(2)).toRotationMatrix();

        return aa1 * aa2 * aa3;
      }

      template<int Nq, int Nv>
      RangeJoint RangeJoint::setDimension() const
      {
        typedef MjcfVisitor::Vector Vector;
        const double infty = std::numeric_limits<double>::infinity();

        RangeJoint ret;
        ret.minEffort = Vector::Constant(Nv, -infty);
        ret.maxEffort = Vector::Constant(Nv, infty);
        ret.minVel = Vector::Constant(Nv, -infty);
        ret.maxVel = Vector::Constant(Nv, infty);
        ret.maxConfig = Vector::Constant(Nq, 1.01);
        ret.minConfig = Vector::Constant(Nq, -1.01);
        ret.configLimitMargin = Vector::Constant(Nq, 0.);
        ret.maxDryFriction = Vector::Constant(Nv, 0.);
        ret.minDryFriction = Vector::Constant(Nv, 0.);
        ret.damping = Vector::Constant(Nv, 0.);
        ret.armature = Vector::Constant(Nv, armature[0]);
        ret.springStiffness = springStiffness;
        ret.springReference = springReference;
        return ret;
      }

      template<int Nq, int Nv>
      RangeJoint RangeJoint::concatenate(const RangeJoint & range) const
      {
        assert(range.maxEffort.size() == Nv);
        assert(range.minConfig.size() == Nq);
        assert(range.configLimitMargin.size() == Nq);

        RangeJoint ret(*this);
        ret.minEffort.conservativeResize(minEffort.size() + Nv);
        ret.minEffort.tail(Nv) = range.minEffort;
        ret.maxEffort.conservativeResize(maxEffort.size() + Nv);
        ret.maxEffort.tail(Nv) = range.maxEffort;
        ret.minVel.conservativeResize(minVel.size() + Nv);
        ret.minVel.tail(Nv) = range.minVel;
        ret.maxVel.conservativeResize(maxVel.size() + Nv);
        ret.maxVel.tail(Nv) = range.maxVel;

        ret.minConfig.conservativeResize(minConfig.size() + Nq);
        ret.minConfig.tail(Nq) = range.minConfig;
        ret.maxConfig.conservativeResize(maxConfig.size() + Nq);
        ret.maxConfig.tail(Nq) = range.maxConfig;
        ret.configLimitMargin.conservativeResize(configLimitMargin.size() + Nq);
        ret.configLimitMargin.tail(Nq) = range.configLimitMargin;

        ret.damping.conservativeResize(damping.size() + Nv);
        ret.damping.tail(Nv) = range.damping;
        ret.minDryFriction.conservativeResize(minDryFriction.size() + Nv);
        ret.minDryFriction.tail(Nv) = range.minDryFriction;
        ret.maxDryFriction.conservativeResize(maxDryFriction.size() + Nv);
        ret.maxDryFriction.tail(Nv) = range.maxDryFriction;

        ret.springReference.conservativeResize(springReference.size() + 1);
        ret.springReference.tail(1) = range.springReference;
        ret.springStiffness.conservativeResize(springStiffness.size() + 1);
        ret.springStiffness.tail(1) = range.springStiffness;

        ret.armature.conservativeResize(armature.size() + Nv);
        ret.armature.tail(Nv) = range.armature;

        return ret;
      }

      void MjcfJoint::goThroughElement(
        const ptree & el, bool use_limits, const MjcfCompiler & currentCompiler)
      {

        if (!use_limits && el.get_optional<std::string>("<xmlattr>.range"))
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Range limit is specified but it was not specify to use it.");

        // Type
        auto type_s = el.get_optional<std::string>("<xmlattr>.type");
        if (type_s)
          jointType = *type_s;

        // Axis
        auto ax = el.get_optional<std::string>("<xmlattr>.axis");
        if (ax)
          axis = internal::getVectorFromStream<3>(*ax);

        // Config limits (upper/lower)
        auto range_ = el.get_optional<std::string>("<xmlattr>.range");
        bool has_range_limits = false;
        if (range_)
        {
          Eigen::Vector2d rangeT = internal::getVectorFromStream<2>(*range_);
          range.minConfig[0] = currentCompiler.convertAngle(rangeT(0));
          range.maxConfig[0] = currentCompiler.convertAngle(rangeT(1));
          has_range_limits = true;
        }

        // Config limit margins
        auto margin_ = el.get_optional<double>("<xmlattr>.margin");
        if (margin_)
        {
          PINOCCHIO_THROW_PRETTY_IF(
            *margin_ < 0, std::invalid_argument, "Negative joint limit margin.");
          range.configLimitMargin.array() = currentCompiler.convertAngle(*margin_);
        }

        // Effort limit
        range_ = el.get_optional<std::string>("<xmlattr>.actuatorfrcrange");
        if (range_)
        {
          Eigen::Vector2d rangeT = internal::getVectorFromStream<2>(*range_);
          range.minEffort[0] = rangeT(0);
          range.maxEffort[0] = rangeT(1);
        }

        // Spring
        auto value = el.get_optional<double>("<xmlattr>.springref");
        if (value)
          range.springReference[0] = *value;

        // damping
        value = el.get_optional<double>("<xmlattr>.damping");
        if (value)
          range.damping[0] = *value;

        value = el.get_optional<double>("<xmlattr>.armature");
        if (value)
          range.armature[0] = *value;

        // friction loss
        value = el.get_optional<double>("<xmlattr>.frictionloss");
        if (value)
        {
          range.maxDryFriction.array() = *value;
          range.minDryFriction = -range.maxDryFriction;
        }
        value = el.get_optional<double>("<xmlattr>.ref");
        if (value)
        {
          bool has_pos_ref = false;
          if (jointType == "slide")
          {
            posRef = *value;
            has_pos_ref = true;
          }
          else if (jointType == "hinge")
          {
            posRef = currentCompiler.convertAngle(*value);
            has_pos_ref = true;
          }
          else
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Reference position can only be used with hinge or slide joints.");
          if (has_range_limits && has_pos_ref)
          {
            range.minConfig[0] -= posRef;
            range.maxConfig[0] -= posRef;
          }
        }
      }

      void MjcfJoint::fill(
        const ptree & el, const MjcfBody & currentBody, const MjcfGraph & currentGraph)
      {
        bool use_limit = true;
        // Name
        auto name_s = el.get_optional<std::string>("<xmlattr>.name");
        if (name_s)
          jointName = *name_s;
        else
          jointName =
            currentBody.bodyName + "Joint_" + std::to_string(currentBody.jointChildren.size());

        // Check if we need to check for limited argument
        if (!currentGraph.compilerInfo.autolimits)
        {
          use_limit = false;
          auto use_ls = el.get_optional<std::string>("<xmlattr>.limited");
          use_limit = *use_ls == "true";
        }

        // Placement
        jointPlacement = currentGraph.convertPosition(el);

        // default < ChildClass < Class < Real Joint
        if (currentGraph.mapOfClasses.find("mujoco_default") != currentGraph.mapOfClasses.end())
        {
          const MjcfClass & classD = currentGraph.mapOfClasses.at("mujoco_default");
          if (auto joint_p = classD.classElement.get_child_optional("joint"))
            goThroughElement(*joint_p, use_limit, currentGraph.compilerInfo);
        }
        //  childClass
        if (currentBody.childClass != "")
        {
          const MjcfClass & classE = currentGraph.mapOfClasses.at(currentBody.childClass);
          if (auto joint_p = classE.classElement.get_child_optional("joint"))
            goThroughElement(*joint_p, use_limit, currentGraph.compilerInfo);
        }
        // Class
        auto cl_s = el.get_optional<std::string>("<xmlattr>.class");
        if (cl_s)
        {
          std::string className = *cl_s;
          const MjcfClass & classE = currentGraph.mapOfClasses.at(className);
          if (auto joint_p = classE.classElement.get_child_optional("joint"))
            goThroughElement(*joint_p, use_limit, currentGraph.compilerInfo);
        }
        // Joint
        goThroughElement(el, use_limit, currentGraph.compilerInfo);
      }

      SE3 MjcfGraph::convertPosition(const ptree & el) const
      {
        SE3 placement;
        placement.setIdentity();

        // position
        auto pos = el.get_optional<std::string>("<xmlattr>.pos");
        if (pos)
          placement.translation() = internal::getVectorFromStream<3>(*pos);

        /////////// Rotation
        // Quaternion (w, x, y, z)
        auto rot_s = el.get_optional<std::string>("<xmlattr>.quat");
        if (rot_s)
        {
          Eigen::Vector4d quat = internal::getVectorFromStream<4>(*rot_s);

          Eigen::Quaterniond quaternion(quat(0), quat(1), quat(2), quat(3));
          quaternion.normalize();
          placement.rotation() = quaternion.toRotationMatrix();
        }
        // Axis Angle
        else if ((rot_s = el.get_optional<std::string>("<xmlattr>.axisangle")))
        {
          Eigen::Vector4d axis_angle = internal::getVectorFromStream<4>(*rot_s);

          double angle = axis_angle(3);

          Eigen::AngleAxisd angleAxis(compilerInfo.convertAngle(angle), axis_angle.head(3));
          placement.rotation() = angleAxis.toRotationMatrix();
        }
        // Euler Angles
        else if ((rot_s = el.get_optional<std::string>("<xmlattr>.euler")))
        {
          Eigen::Vector3d angles = internal::getVectorFromStream<3>(*rot_s);

          placement.rotation() = compilerInfo.convertEuler(angles);
        }
        else if ((rot_s = el.get_optional<std::string>("<xmlattr>.xyaxes")))
        {
          Eigen::Matrix<double, 6, 1> xyaxes = internal::getVectorFromStream<6>(*rot_s);

          Eigen::Vector3d xAxis = xyaxes.head(3);
          xAxis.normalize();
          Eigen::Vector3d yAxis = xyaxes.tail(3);

          // make y axis orthogonal to x axis, normalize
          double d = xAxis.dot(yAxis);
          yAxis -= xAxis * d;
          yAxis.normalize();

          Eigen::Vector3d zAxis = xAxis.cross(yAxis);
          zAxis.normalize();

          Eigen::Matrix3d rotation;
          rotation.col(0) = xAxis;
          rotation.col(1) = yAxis;
          rotation.col(2) = zAxis;

          placement.rotation() = rotation;
        }
        else if ((rot_s = el.get_optional<std::string>("<xmlattr>.zaxis")))
        {
          Eigen::Vector3d zaxis = internal::getVectorFromStream<3>(*rot_s);
          // Compute the rotation matrix that maps z_axis to unit z
          placement.rotation() =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), zaxis).toRotationMatrix();
        }
        return placement;
      }

      Inertia MjcfGraph::convertInertiaFromMjcf(const ptree & el) const
      {
        double mass = std::max(el.get<double>("<xmlattr>.mass"), compilerInfo.boundMass);
        ;
        if (mass < 0)
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Mass of body is not supposed to be negative");

        Inertia::Vector3 com;
        auto com_s = el.get_optional<std::string>("<xmlattr>.pos");
        if (com_s)
          com = internal::getVectorFromStream<3>(*com_s);
        else
          com = Inertia::Vector3::Zero();

        const Inertia::Matrix3 R = convertPosition(el).rotation();

        Inertia::Matrix3 I = Eigen::Matrix3d::Identity();

        auto inertia_s = el.get_optional<std::string>("<xmlattr>.diaginertia");
        if (inertia_s)
        {
          Eigen::Vector3d inertiaDiag = internal::getVectorFromStream<3>(*inertia_s);
          I = inertiaDiag.asDiagonal();
        }

        else if ((inertia_s = el.get_optional<std::string>("<xmlattr>.fullinertia")))
        {
          // M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3)
          std::istringstream inertiaStream = internal::getConfiguredStringStream(*inertia_s);
          inertiaStream >> I(0, 0);
          inertiaStream >> I(1, 1);
          inertiaStream >> I(2, 2);
          inertiaStream >> I(0, 1);
          inertiaStream >> I(0, 2);
          inertiaStream >> I(1, 2);

          I(1, 0) = I(0, 1);
          I(2, 0) = I(0, 2);
          I(2, 1) = I(1, 2);
        }

        // Extract the diagonal elements as a vector
        for (int i = 0; i < 3; i++)
          I(i, i) = std::max(I(i, i), compilerInfo.boundInertia);

        return Inertia(mass, com, R * I * R.transpose());
      }

      void MjcfGraph::parseJointAndBody(
        const ptree & el,
        const boost::optional<std::string> & childClass,
        const std::string & parentName)
      {
        MjcfBody currentBody;
        auto chcl_s = childClass;
        // if inertiafromgeom is false and inertia does not exist - throw
        if (!compilerInfo.inertiafromgeom && !el.get_child_optional("inertial"))
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Cannot get inertia from geom and no inertia was found");

        bool usegeominertia = false;
        if (compilerInfo.inertiafromgeom)
          usegeominertia = true;
        else if (
          boost::indeterminate(compilerInfo.inertiafromgeom) && !el.get_child_optional("inertial"))
          usegeominertia = true;

        for (const ptree::value_type & v : el)
        {
          // Current body node
          if (v.first == "<xmlattr>")
          {
            // Name
            auto name_s = v.second.get_optional<std::string>("name");
            if (name_s)
              currentBody.bodyName = *name_s;
            else
              currentBody.bodyName = parentName + "Bis";

            currentBody.bodyParent = parentName;
            currentBody.bodyPlacement = convertPosition(el);

            bodiesList.push_back(currentBody.bodyName);

            if (auto chcl_st = v.second.get_optional<std::string>("childclass"))
            {
              chcl_s = chcl_st;
              currentBody.childClass = *chcl_s;
            }
            else if (childClass)
              currentBody.childClass = *chcl_s;

            // Class
            auto cl_s = v.second.get_optional<std::string>("class");
            if (cl_s)
              currentBody.bodyClassName = *cl_s;

            // Still need to deal with gravcomp and figure out if we need mocap, and user param...
          }
          // Inertia
          if (v.first == "inertial" && !usegeominertia)
            currentBody.bodyInertia = convertInertiaFromMjcf(v.second);

          // Geom
          if (v.first == "geom")
          {
            MjcfGeom currentGeom;
            currentGeom.fill(v.second, currentBody, *this);
            currentBody.geomChildren.push_back(currentGeom);
          }

          // Joint
          if (v.first == "joint")
          {
            MjcfJoint currentJoint;
            currentJoint.fill(v.second, currentBody, *this);
            currentBody.jointChildren.push_back(currentJoint);
          }
          if (v.first == "freejoint")
          {
            MjcfJoint currentJoint;
            currentJoint.jointType = "free";
            auto jointName = v.second.get_optional<std::string>("<xmlattr>.name");
            if (jointName)
              currentJoint.jointName = *jointName;
            else
              currentJoint.jointName = currentBody.bodyName + "_free";

            currentBody.jointChildren.push_back(currentJoint);
          }
          if (v.first == "site")
          {
            MjcfSite currentSite;
            currentSite.fill(v.second, currentBody, *this);
            currentBody.siteChildren.push_back(currentSite);
          }
          if (v.first == "body")
          {
            parseJointAndBody(v.second, chcl_s, currentBody.bodyName);
          }
        }
        // Add all geom inertias if needed
        if (usegeominertia)
        {
          Inertia inert_temp(Inertia::Zero());
          for (const auto & geom : currentBody.geomChildren)
          {
            if (geom.geomKind != MjcfGeom::VISUAL)
              inert_temp += geom.geomPlacement.act(geom.geomInertia);
          }
          currentBody.bodyInertia = inert_temp;
        }
        mapOfBodies.insert(std::make_pair(currentBody.bodyName, currentBody));
      }

      void MjcfGraph::parseTexture(const ptree & el)
      {
        namespace fs = boost::filesystem;
        MjcfTexture text;
        auto file = el.get_optional<std::string>("<xmlattr>.file");
        auto name_ = el.get_optional<std::string>("<xmlattr>.name");
        auto type = el.get_optional<std::string>("<xmlattr>.type");
        auto builtin = el.get_optional<std::string>("<xmlattr>.builtin");

        std::string name;
        if (name_)
          name = *name_;
        else if (type && *type == "skybox")
          name = *type;
        if (!builtin || *builtin == "none")
        {
          if (!file)
          {
            std::cout << "Warning - Only texture with files are supported" << std::endl;
            if (name.empty())
              PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Textures need a name.");
          }
          else
          {
            fs::path filePath(*file);
            name = getName(el, filePath);

            text.filePath =
              updatePath(compilerInfo.strippath, compilerInfo.texturedir, modelPath, filePath)
                .string();
          }
        }
        else
        {
          if (name.empty())
            PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Textures need a name.");
        }
        auto str_v = el.get_optional<std::string>("<xmlattr>.type");
        if (str_v)
          text.textType = *str_v;

        if ((str_v = el.get_optional<std::string>("<xmlattr>.gridsize")))
          text.gridsize = internal::getVectorFromStream<2>(*str_v);

        mapOfTextures.insert(std::make_pair(name, text));
      }

      void MjcfGraph::parseMaterial(const ptree & el)
      {
        std::string name;
        MjcfMaterial mat;
        auto n = el.get_optional<std::string>("<xmlattr>.name");
        if (n)
          name = *n;
        else
          PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Material was given without a name");

        // default < Class < Attributes
        if (mapOfClasses.find("mujoco_default") != mapOfClasses.end())
        {
          const MjcfClass & classD = mapOfClasses.at("mujoco_default");
          if (auto mat_p = classD.classElement.get_child_optional("material"))
            mat.goThroughElement(*mat_p);
        }
        auto cl_s = el.get_optional<std::string>("<xmlattr>.class");
        if (cl_s)
        {
          std::string className = *cl_s;
          const MjcfClass & classE = mapOfClasses.at(className);
          if (auto mat_p = classE.classElement.get_child_optional("material"))
            mat.goThroughElement(*mat_p);
        }

        mat.goThroughElement(el);

        mapOfMaterials.insert(std::make_pair(name, mat));
      }

      void MjcfGraph::parseMesh(const ptree & el)
      {
        namespace fs = boost::filesystem;

        MjcfMesh mesh;
        auto file = el.get_optional<std::string>("<xmlattr>.file");
        auto scale = el.get_optional<std::string>("<xmlattr>.scale");
        if (scale)
          mesh.scale = internal::getVectorFromStream<3>(*scale);
        if (file)
        {
          fs::path filePath(*file);
          std::string name = getName(el, filePath);

          mesh.filePath =
            updatePath(compilerInfo.strippath, compilerInfo.meshdir, modelPath, filePath).string();
          mapOfMeshes.insert(std::make_pair(name, mesh));
          return;
        }

        // Handle vertex-based mesh
        auto vertex = el.get_optional<std::string>("<xmlattr>.vertex");
        if (!vertex)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Only meshes with files/vertices are supported.")
        }

        auto name = el.get_optional<std::string>("<xmlattr>.name");
        if (!name)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Mesh with vertices without a name is not supported");
        }

        // Parse and validate vertices
        Eigen::VectorXd meshVertices = internal::getUnknownSizeVectorFromStream(*vertex);
        if (meshVertices.size() % 3 != 0)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Number of vertices is not a multiple of 3");
        }

        // Convert to 3D vertex matrix
        const auto numVertices = meshVertices.size() / 3;
        Eigen::MatrixX3d vertices(numVertices, 3);
        for (auto i = 0; i < numVertices; ++i)
        {
          vertices.row(i) = meshVertices.segment<3>(3 * i).transpose();
        }
        mesh.vertices = vertices;
        mapOfMeshes.insert(std::make_pair(*name, mesh));
      }

      void MjcfGraph::parseAsset(const ptree & el)
      {
        for (const ptree::value_type & v : el)
        {
          if (v.first == "mesh")
            parseMesh(v.second);

          if (v.first == "material")
            parseMaterial(v.second);

          if (v.first == "texture")
            parseTexture(v.second);

          if (v.first == "hfield")
            PINOCCHIO_THROW_PRETTY(std::invalid_argument, "hfields are not supported yet");
        }
      }

      void MjcfGraph::parseDefault(ptree & el, const ptree & parent, const std::string & parentTag)
      {
        boost::optional<std::string> nameClass;
        // Classes
        for (ptree::value_type & v : el)
        {
          if (v.first == "<xmlattr>")
          {
            nameClass = v.second.get_optional<std::string>("class");
            if (nameClass)
            {
              MjcfClass classDefault;
              classDefault.className = *nameClass;
              updateClassElement(el, parent);
              classDefault.classElement = el;
              mapOfClasses.insert(std::make_pair(*nameClass, classDefault));
            }
            else
              PINOCCHIO_THROW_PRETTY(
                std::invalid_argument, "Class does not have a name. Cannot parse model.");
          }
          else if (v.first == "default")
            parseDefault(v.second, el, v.first);
          else if (parentTag == "mujoco" && v.first != "<xmlattr>")
          {
            MjcfClass classDefault;
            classDefault.className = "mujoco_default";
            classDefault.classElement = el;
            mapOfClasses.insert(std::make_pair("mujoco_default", classDefault));
          }
        }
      }

      void MjcfGraph::parseCompiler(const ptree & el)
      {
        // get autolimits
        auto auto_s = el.get_optional<std::string>("<xmlattr>.autolimits");
        if (auto_s)
          if (*auto_s == "true")
            compilerInfo.autolimits = true;

        // strip path
        auto strip_s = el.get_optional<std::string>("<xmlattr>.strippath");
        if (strip_s)
          if (*strip_s == "true")
            compilerInfo.strippath = true;

        // get dir to mesh and texture
        auto dir = el.get_optional<std::string>("<xmlattr>.assetdir");
        if (dir)
        {
          compilerInfo.meshdir = *dir;
          compilerInfo.texturedir = *dir;
        }

        if ((dir = el.get_optional<std::string>("<xmlattr>.meshdir")))
          compilerInfo.meshdir = *dir;

        if ((dir = el.get_optional<std::string>("<xmlattr>.texturedir")))
          compilerInfo.texturedir = *dir;

        auto value_v = el.get_optional<double>("<xmlattr>.boundmass");
        if (value_v)
          compilerInfo.boundMass = *value_v;

        if ((value_v = el.get_optional<double>("<xmlattr>.boundinertia")))
          compilerInfo.boundInertia = *value_v;

        auto in_g = el.get_optional<std::string>("<xmlattr>.inertiafromgeom");
        if (in_g)
        {
          if (*in_g == "true")
            compilerInfo.inertiafromgeom = true;
          else if (*in_g == "false")
            compilerInfo.inertiafromgeom = false;
        }

        // angle radian or degree
        auto angle_s = el.get_optional<std::string>("<xmlattr>.angle");
        if (angle_s)
          if (*angle_s == "radian")
            compilerInfo.angle_converter = 1;

        auto eulerS = el.get_optional<std::string>("<xmlattr>.eulerseq");
        if (eulerS)
        {
          std::string eulerseq = *eulerS;
          if (eulerseq.find_first_not_of("xyzXYZ") != std::string::npos || eulerseq.size() != 3)
          {
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Model tried to use euler angles but euler sequence is wrong");
          }
          else
          {
            // get index combination
            for (std::size_t i = 0; i < eulerseq.size(); i++)
            {
              auto ci = static_cast<Eigen::Index>(i);
              switch (eulerseq[i])
              {
              case 'x':
                compilerInfo.mapEulerAngles.col(ci) = Eigen::Vector3d::UnitX();
                break;
              case 'X':
                compilerInfo.mapEulerAngles.col(ci) = Eigen::Vector3d::UnitX();
                break;
              case 'y':
                compilerInfo.mapEulerAngles.col(ci) = Eigen::Vector3d::UnitY();
                break;
              case 'Y':
                compilerInfo.mapEulerAngles.col(ci) = Eigen::Vector3d::UnitY();
                break;
              case 'z':
                compilerInfo.mapEulerAngles.col(ci) = Eigen::Vector3d::UnitZ();
                break;
              case 'Z':
                compilerInfo.mapEulerAngles.col(ci) = Eigen::Vector3d::UnitZ();
                break;
              default:
                PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Euler Axis does not exist");
                break;
              }
            }
          }
        }
      }

      void MjcfGraph::parseKeyFrame(const ptree & el)
      {
        for (const ptree::value_type & v : el)
        {
          if (v.first == "key")
          {
            auto nameKey = v.second.get_optional<std::string>("<xmlattr>.name");
            if (nameKey)
            {
              auto configVectorS = v.second.get_optional<std::string>("<xmlattr>.qpos");
              if (configVectorS)
              {
                Eigen::VectorXd configVector =
                  internal::getUnknownSizeVectorFromStream(*configVectorS);
                mapOfConfigs.insert(std::make_pair(*nameKey, configVector));
              }
            }
          }
        }
      }

      void MjcfGraph::parseEquality(const ptree & el)
      {
        for (const ptree::value_type & v : el)
        {
          std::string type = v.first;
          // List of supported constraints from mjcf description
          // equality -> connect

          // The constraints below are not supported and will be ignored with the following
          // warning: joint, flex, distance, weld
          if ((type != "connect") && (type != "weld"))
          {
            // TODO(jcarpent): support extra constraint types such as joint, flex, distance.
            continue;
          }

          MjcfEquality eq;
          eq.type = type;

          // get the name of first body
          auto body1 = v.second.get_optional<std::string>("<xmlattr>.body1");
          if (body1)
            eq.body1 = *body1;

          // get the name of second body
          auto body2 = v.second.get_optional<std::string>("<xmlattr>.body2");
          if (body2)
            eq.body2 = *body2;

          // get the name of first site
          auto site1 = v.second.get_optional<std::string>("<xmlattr>.site1");
          if (site1)
            eq.site1 = *site1;

          // get the name of second site
          auto site2 = v.second.get_optional<std::string>("<xmlattr>.site2");
          if (site2)
            eq.site2 = *site2;

          // get the name of the constraint (if it exists)
          auto name = v.second.get_optional<std::string>("<xmlattr>.name");
          if (name)
            eq.name = *name;

          // TODO: argument torqscale present in MJCF for energy base solve
          // get the anchor position
          auto anchor = v.second.get_optional<std::string>("<xmlattr>.anchor");
          if (anchor)
            eq.anchor = internal::getVectorFromStream<3>(*anchor);

          // get the relative pose
          auto relpose = v.second.get_optional<std::string>("<xmlattr>.relpose");
          if (relpose)
          {
            Eigen::Matrix<double, 7, 1> pos_quat = internal::getVectorFromStream<7>(*relpose);
            Eigen::Vector4d quat_vec = pos_quat.tail(4);
            Eigen::Quaterniond quaternion(pos_quat(3), pos_quat(4), pos_quat(5), pos_quat(6));
            if (quat_vec.isZero(0))
              // Weird default behavior from Mujoco
              // If quat is four 0, use the relpose in the reference configuration qref
              // then relpose is ignored
              // See: https://mujoco.readthedocs.io/en/latest/XMLreference.html#equality-weld
              eq.use_qref_relpose = true;
            else
            {
              // We will use relpose argument so we store it as SE3 placement
              eq.use_qref_relpose = false;
              quaternion.normalize();
              eq.relpose.translation() = pos_quat.head(3);
              eq.relpose.rotation() = quaternion.toRotationMatrix();
            }
          }

          mapOfEqualities.insert(
            std::make_pair(eq.name + eq.body1 + eq.site1 + eq.body2 + eq.site2, eq));
        }
      }

      void MjcfGraph::parseGraph()
      {
        boost::property_tree::ptree el;
        if (pt.get_child_optional("mujoco"))
          el = pt.get_child("mujoco");
        else
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "This is not a standard mujoco model. Cannot parse it.");

        for (const ptree::value_type & v : el)
        {
          // get model name
          if (v.first == "<xmlattr>")
          {
            auto n_s = v.second.get_optional<std::string>("model");
            if (n_s)
              modelName = *n_s;
            else
              PINOCCHIO_THROW_PRETTY(
                std::invalid_argument, "Model is missing a name. Cannot parse it");
          }

          if (v.first == "compiler")
            parseCompiler(el.get_child("compiler"));

          if (v.first == "default")
            parseDefault(el.get_child("default"), el, "mujoco");

          if (v.first == "asset")
            parseAsset(el.get_child("asset"));

          if (v.first == "keyframe")
            parseKeyFrame(el.get_child("keyframe"));

          if (v.first == "worldbody")
          {
            boost::optional<std::string> childClass;
            parseWorldBodyGeoms(el.get_child("worldbody"));
            parseJointAndBody(el.get_child("worldbody").get_child("body"), childClass);
          }

          if (v.first == "equality")
          {
            parseEquality(el.get_child("equality"));
          }
        }
      }

      void MjcfGraph::parseWorldBodyGeoms(const ptree & el)
      {
        // in pinocchio, the worldbody is called "universe"
        worldBody.bodyName = "universe";
        for (const ptree::value_type & v : el)
        {
          if (v.first == "geom")
          {
            MjcfGeom currentGeom;
            currentGeom.fill(v.second, worldBody, *this);
            worldBody.geomChildren.push_back(currentGeom);
          }
        }
      }

      void MjcfGraph::parseGraphFromXML(const std::string & xmlStr)
      {
        boost::property_tree::read_xml(xmlStr, pt);
        parseGraph();
      }

      template<typename TypeX, typename TypeY, typename TypeZ, typename TypeUnaligned>
      JointModel MjcfGraph::createJoint(const Eigen::Vector3d & axis)
      {
        if (axis.isApprox(Eigen::Vector3d::UnitX()))
          return TypeX();
        else if (axis.isApprox(Eigen::Vector3d::UnitY()))
          return TypeY();
        else if (axis.isApprox(Eigen::Vector3d::UnitZ()))
          return TypeZ();
        else
          return TypeUnaligned(axis.normalized());
      }

      void MjcfGraph::addSoloJoint(
        const MjcfJoint & joint, const MjcfBody & currentBody, SE3 & bodyInJoint)
      {

        FrameIndex parentFrameId = 0;
        if (!currentBody.bodyParent.empty())
          parentFrameId = mjcfVisitor.getBodyId(currentBody.bodyParent);

        // get body pose in body parent
        const SE3 bodyPose = currentBody.bodyPlacement;
        Inertia inert = currentBody.bodyInertia;
        SE3 jointInParent = bodyPose * joint.jointPlacement;
        bodyInJoint = joint.jointPlacement.inverse();
        JointType jType;

        RangeJoint range;
        if (joint.jointType == "free")
        {
          mjcfVisitor << "Free Joint " << '\n';
          range = joint.range.setDimension<7, 6>();
          jType = JointType::FLOATING;
        }
        else if (joint.jointType == "slide")
        {
          mjcfVisitor << "joint prismatic with axis " << joint.axis << '\n';
          range = joint.range;
          jType = JointType::PRISMATIC;
        }
        else if (joint.jointType == "ball")
        {
          mjcfVisitor << "Sphere Joint " << '\n';
          range = joint.range.setDimension<4, 3>();
          jType = JointType::SPHERICAL;
        }
        else if (joint.jointType == "hinge")
        {
          mjcfVisitor << "joint revolute with axis " << joint.axis << '\n';
          range = joint.range;
          jType = JointType::REVOLUTE;
        }
        else
          PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Unknown joint type");

        mjcfVisitor.addJointAndBody(
          jType, joint.axis, parentFrameId, jointInParent, joint.jointName, inert, bodyInJoint,
          currentBody.bodyName, range.minEffort, range.maxEffort, range.minVel, range.maxVel,
          range.minConfig, range.maxConfig, range.configLimitMargin, range.minDryFriction,
          range.maxDryFriction, range.damping);

        // Add armature info
        JointIndex j_id = mjcfVisitor.getJointId(joint.jointName);
        mjcfVisitor.model.armature.segment(
          mjcfVisitor.model.joints[j_id].idx_v(), mjcfVisitor.model.joints[j_id].nv()) =
          range.armature;
      }

      void MjcfGraph::fillModel(const std::string & nameOfBody)
      {
        typedef MjcfVisitor::SE3 SE3;

        if (mapOfBodies.find(nameOfBody) == mapOfBodies.end())
        {
          std::stringstream ss;
          ss << "Cannot find body " << nameOfBody;
          PINOCCHIO_THROW_PRETTY(std::invalid_argument, ss.str());
        }
        MjcfBody currentBody = mapOfBodies.at(nameOfBody);

        // get parent body frame
        FrameIndex parentFrameId = 0;
        if (!currentBody.bodyParent.empty())
          parentFrameId = mjcfVisitor.getBodyId(currentBody.bodyParent);

        Frame frame = mjcfVisitor.model.frames[parentFrameId];
        // get body pose in body parent
        const SE3 bodyPose = currentBody.bodyPlacement;
        Inertia inertia = currentBody.bodyInertia;

        // Fixed Joint
        if (currentBody.jointChildren.size() == 0)
        {
          if (currentBody.bodyParent.empty())
            return;

          std::string jointName = nameOfBody + "_fixed";
          mjcfVisitor << jointName << " being parsed." << '\n';

          mjcfVisitor.addFixedJointAndBody(parentFrameId, bodyPose, jointName, inertia, nameOfBody);

          // Attach child site frame to parent joint
          FrameIndex bodyId = mjcfVisitor.model.getFrameId(nameOfBody, BODY);
          JointIndex parentJoint = mjcfVisitor.model.frames[bodyId].parentJoint;
          for (const auto & site : currentBody.siteChildren)
          {
            // Frames are expressed in their supporting joint, which only matches bodyPose when
            // the body it hangs off sits at that joint's origin.
            SE3 placement = mjcfVisitor.model.frames[bodyId].placement * site.sitePlacement;
            mjcfVisitor.model.addFrame(
              Frame(site.siteName, parentJoint, bodyId, placement, OP_FRAME));
          }
          return;
        }

        bool composite = false;
        SE3 jointPlacement, firstJointPlacement, prevJointPlacement = SE3::Identity();

        RangeJoint rangeCompo;
        JointModelComposite jointM;
        std::string jointName;

        if (currentBody.jointChildren.size() > 1)
        {
          composite = true;

          MjcfJoint firstOne = currentBody.jointChildren.at(0);
          jointName = "Composite_" + firstOne.jointName;
        }

        fillReferenceConfig(currentBody);

        bool isFirst = true;
        SE3 bodyInJoint;

        if (!composite)
        {
          addSoloJoint(currentBody.jointChildren.at(0), currentBody, bodyInJoint);
        }
        else
        {
          for (const auto & joint : currentBody.jointChildren)
          {
            if (joint.jointType == "free")
              PINOCCHIO_THROW_PRETTY(
                std::invalid_argument, "Joint Composite trying to be created with a freeFlyer");

            SE3 jointInParent = bodyPose * joint.jointPlacement;
            bodyInJoint = joint.jointPlacement.inverse();
            if (isFirst)
            {
              firstJointPlacement = jointInParent;
              jointPlacement = SE3::Identity();
              isFirst = false;
            }
            else
              jointPlacement = prevJointPlacement.inverse() * jointInParent;
            if (joint.jointType == "slide")
            {
              jointM.addJoint(
                createJoint<JointModelPX, JointModelPY, JointModelPZ, JointModelPrismaticUnaligned>(
                  joint.axis),
                jointPlacement);

              rangeCompo = rangeCompo.concatenate<1, 1>(joint.range);
            }
            else if (joint.jointType == "ball")
            {
              jointM.addJoint(JointModelSpherical(), jointPlacement);
              rangeCompo = rangeCompo.concatenate<4, 3>(joint.range.setDimension<4, 3>());
            }
            else if (joint.jointType == "hinge")
            {
              jointM.addJoint(
                createJoint<JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned>(
                  joint.axis),
                jointPlacement);
              rangeCompo = rangeCompo.concatenate<1, 1>(joint.range);
            }
            else
              PINOCCHIO_THROW_PRETTY(
                std::invalid_argument, "Unknown joint type trying to be parsed.");

            prevJointPlacement = jointInParent;
          }
          JointIndex joint_id;

          joint_id = mjcfVisitor.model.addJoint(
            frame.parentJoint, jointM, frame.placement * firstJointPlacement, jointName,
            rangeCompo.minEffort, rangeCompo.maxEffort, rangeCompo.minVel, rangeCompo.maxVel,
            rangeCompo.minConfig, rangeCompo.maxConfig, rangeCompo.configLimitMargin,
            rangeCompo.minDryFriction, rangeCompo.maxDryFriction, rangeCompo.damping);
          FrameIndex jointFrameId = mjcfVisitor.model.addJointFrame(joint_id, (int)parentFrameId);
          mjcfVisitor.appendBodyToJoint(jointFrameId, inertia, bodyInJoint, nameOfBody);

          mjcfVisitor.model.armature.segment(
            mjcfVisitor.model.joints[joint_id].idx_v(), mjcfVisitor.model.joints[joint_id].nv()) =
            rangeCompo.armature;
        }

        // Attach child site frame to parent joint
        FrameIndex bodyId = mjcfVisitor.model.getFrameId(nameOfBody, BODY);
        frame = mjcfVisitor.model.frames[bodyId];
        for (const auto & site : currentBody.siteChildren)
        {
          SE3 placement = bodyInJoint * site.sitePlacement;
          mjcfVisitor.model.addFrame(
            Frame(site.siteName, frame.parentJoint, bodyId, placement, OP_FRAME));
        }
      }

      void MjcfGraph::fillReferenceConfig(const MjcfBody & currentBody)
      {
        for (const auto & joint : currentBody.jointChildren)
        {
          assert(qpos0.size() == referenceConfig.size());
          if (joint.jointType == "free")
          {
            referenceConfig.conservativeResize(referenceConfig.size() + 7);
            // In FK of mujoco, the placement of a freeflyer w.r.t its parent is ignored.
            // Instead, the freeflyer's components of the configuration vector are used directly in
            // the FK. For other joints, the placement w.r.t the parent is taken into consideration.
            // In pinocchio, the placement w.r.t the parent is always taken into consideration.
            // So for the special case of freeflyers, we apply the opposite transformation
            // of the mujoco's freeflyer.
            // Consequently, when we adjust the joint placements (see @ref
            // updateJointPlacementsFromReferenceConfig), we obtain the same result given a mujoco
            // configuration vector.
            const SE3::Quaternion q(currentBody.bodyPlacement.rotation());
            const SE3::Quaternion qinv = q.inverse();
            const Eigen::Vector3d t(-currentBody.bodyPlacement.translation());
            referenceConfig.tail(7) << t(0), t(1), t(2), qinv.x(), qinv.y(), qinv.z(), qinv.w();

            // We store qpos0 in the convention of mujoco.
            // The function `addKeyFrame` will convert this qpos0 to the pinocchio's convention.
            // The `addKeyFrame` function also deals with composite joints, which is something we
            // can't do in this function.
            qpos0.conservativeResize(qpos0.size() + 7);
            qpos0.tail(7) << -t(0), -t(1), -t(2), q.w(), q.x(), q.y(), q.z();
          }
          else if (joint.jointType == "ball")
          {
            referenceConfig.conservativeResize(referenceConfig.size() + 4);
            referenceConfig.tail(4) << Eigen::Vector4d(0, 0, 0, 1);

            qpos0.conservativeResize(qpos0.size() + 4);
            qpos0.tail(4) << Eigen::Vector4d(1, 0, 0, 0);
          }
          else if (joint.jointType == "slide" || joint.jointType == "hinge")
          {
            referenceConfig.conservativeResize(referenceConfig.size() + 1);
            referenceConfig.tail(1) << -joint.posRef;

            qpos0.conservativeResize(qpos0.size() + 1);
            qpos0.tail(1) << joint.posRef;
          }
        }
      }

      void MjcfGraph::addKeyFrame(const Eigen::VectorXd & keyframe, const std::string & keyName)
      {
        // Check config vectors and add them if size is right
        const int model_nq = mjcfVisitor.model.nq;

        PINOCCHIO_CHECK_INPUT_ARGUMENT(
          keyframe.size() == model_nq, "Keyframe size does not match model size");

        Eigen::VectorXd qpos(model_nq);
        for (std::size_t i = 1; i < mjcfVisitor.model.joints.size(); i++)
        {
          const auto & joint = mjcfVisitor.model.joints[i];
          int idx_q = joint.idx_q();
          int nq = joint.nq();

          Eigen::VectorXd qpos_j = keyframe.segment(idx_q, nq);
          if (joint.shortname() == "JointModelFreeFlyer")
          {
            Eigen::Vector4d new_quat(qpos_j(4), qpos_j(5), qpos_j(6), qpos_j(3));
            qpos_j.tail(4) = new_quat;
          }
          else if (joint.shortname() == "JointModelSpherical")
          {
            Eigen::Vector4d new_quat(qpos_j(1), qpos_j(2), qpos_j(3), qpos_j(0));
            qpos_j = new_quat;
          }
          else if (joint.shortname() == "JointModelComposite")
          {
            for (const auto & joint_ : boost::get<JointModelComposite>(joint.toVariant()).joints)
            {
              int idx_q_ = joint_.idx_q() - idx_q;
              int nq_ = joint_.nq();
              if (joint_.shortname() == "JointModelSpherical")
              {
                Eigen::Vector4d new_quat(
                  qpos_j(idx_q_ + 1), qpos_j(idx_q_ + 2), qpos_j(idx_q_ + 3), qpos_j(idx_q_));
                qpos_j.segment(idx_q_, nq_) = new_quat;
              }
            }
          }
          qpos.segment(idx_q, nq) = qpos_j;
        }

        // we normalize in case parsed qpos has numerical errors
        ::pinocchio::normalize(mjcfVisitor.model, qpos);
        mjcfVisitor.model.referenceConfigurations.insert(std::make_pair(keyName, qpos));
      }

      void MjcfGraph::parseContactInformation(
        const Model & model,
        std::vector<PointAnchorConstraintModel> & point_anchor_constraint_models,
        std::vector<FrameAnchorConstraintModel> & frame_anchor_constraint_models)
      {
        Data data(model);
        Eigen::VectorXd qref;
        if (!model.referenceConfigurations.empty())
        {
          // If possible, it's always preferable to select qpos0 to construct the point anchor
          // constraints as the mujoco equality constraints are defined w.r.t the default
          // configuration of the model (qpos0).
          if (model.referenceConfigurations.find("qpos0") != model.referenceConfigurations.end())
          {
            mjcfVisitor << "Using qpos0 (default configuration defined by the XML file) to "
                           "construct point anchor constraints.\n";
            qref = model.referenceConfigurations.at("qpos0");
          }
          else
          {
            mjcfVisitor << "Could not find qpos0 in referenceConfigurations. Using keyframe "
                        << model.referenceConfigurations.begin()->first
                        << " to construct point anchor constraints.\n";
            qref = model.referenceConfigurations.begin()->second;
          }
        }
        else
        {
          mjcfVisitor << "WARNING: Could not find qpos0 nor other keyframes in "
                         "referenceConfigurations.\n"
                      << "This is unexpected and may lead to issues for point anchor constraints.\n"
                      << "Using ::pinocchio::neutral to construct point anchor constraints.\n";
          qref = ::pinocchio::neutral(model);
        }
        ::pinocchio::forwardKinematics(model, data, qref);
        for (const auto & entry : mapOfEqualities)
        {
          const MjcfEquality & eq = entry.second;
          // Retireve parent joints and relative pose
          SE3 i1Mc, i2Mc;
          JointIndex joint1, joint2;
          if (eq.body1 == "")
          {
            if ((eq.site1 == "") || (eq.site2 == ""))
            {
              std::stringstream ss;
              ss << "Body 1 or both site1 and site2 must be specified for constraint"
                 << entry.first;
              PINOCCHIO_THROW_PRETTY(std::invalid_argument, ss.str());
            }
            // It is Mujoco site convention common for weld or connect
            const Frame & frame1 = model.frames[model.getFrameId(eq.site1)];
            const Frame & frame2 = model.frames[model.getFrameId(eq.site2)];
            joint1 = frame1.parentJoint;
            joint2 = frame2.parentJoint;
            i1Mc = frame1.placement;
            i2Mc = frame2.placement;
          }
          else
          {
            joint1 = mjcfVisitor.getParentId(eq.body1);
            const SE3 & oMi1 = data.oMi[joint1];
            // Body 2 default to world joint
            joint2 = (eq.body2 == "") ? 0 : mjcfVisitor.getParentId(eq.body2);
            const SE3 & oMi2 = data.oMi[joint2];
            if (eq.type == "connect")
            {
              // For connect, anchor is relative to joint1
              i1Mc.setIdentity();
              i1Mc.translation() = eq.anchor;
              // Constaint relative to joint 2 is obtaint thanks to qref pose
              i2Mc = (joint2 == 0) ? oMi1 * i1Mc : oMi2.inverse() * oMi1 * i1Mc;
            }
            else if (eq.type == "weld")
            {
              // For weld constraint, anchor is relative to joint2
              i2Mc.setIdentity();
              i2Mc.translation() = eq.anchor;
              // Constraint location relative to joint 1: i1Mc is calculated using i2Mc and given
              // the relative pose i1Mi2.
              // Using weird default behavior of mujoco, use qref relative pose if quat is four 0
              // and relpose argument otherwise
              SE3 i1Mi2 = eq.use_qref_relpose
                            ? ((joint2 == 0) ? oMi1.inverse() : oMi1.inverse() * oMi2)
                            : eq.relpose;
              i1Mc = i1Mi2 * i2Mc;
            }
          }

          // Create the constraints
          if (eq.type == "connect")
          {
            PointAnchorConstraintModel bpcm(model, joint1, i1Mc, joint2, i2Mc);
            point_anchor_constraint_models.push_back(bpcm);
          }
          else if (eq.type == "weld")
          {
            FrameAnchorConstraintModel wcm(model, joint1, i1Mc, joint2, i2Mc);
            frame_anchor_constraint_models.push_back(wcm);
          }
        }
      }

      void MjcfGraph::updateJointPlacementsFromReferenceConfig()
      {
        Data data(mjcfVisitor.model);
        ::pinocchio::forwardKinematics(mjcfVisitor.model, data, referenceConfig);
        for (std::size_t i = 1; i < mjcfVisitor.model.joints.size(); i++)
        {
          auto & joint = mjcfVisitor.model.joints[i];
          if (joint.shortname() == "JointModelComposite")
          {
            JointModelComposite & jmodel = boost::get<JointModelComposite>(joint.toVariant());
            JointDataComposite & jdata = boost::get<JointDataComposite>(data.joints[i]);
            for (std::size_t j = 1; j < jmodel.joints.size(); j++)
            {
              jmodel.jointPlacements[j] = jdata.pjMi[j];
            }
          }
          else
          {
            const std::size_t parenti = mjcfVisitor.model.parents[i];
            const ::pinocchio::SE3 oMparenti = data.oMi[parenti];
            const ::pinocchio::SE3 oMi = data.oMi[i];
            mjcfVisitor.model.jointPlacements[i] = oMparenti.inverse() * oMi;
          }
        }
      }

      void MjcfGraph::parseRootTree(
        const boost::optional<const JointModel &> rootJoint,
        const boost::optional<const std::string &> rootJointName)
      {
        mjcfVisitor.setName(modelName);
        // get name and inertia of first root link
        PINOCCHIO_THROW_PRETTY_IF(
          bodiesList.empty(), std::invalid_argument, "MJCF parser: no body found in parsed tree.");
        std::string rootLinkName = bodiesList.at(0);
        MjcfBody rootBody = mapOfBodies.find(rootLinkName)->second;
        if (rootBody.jointChildren.size() == 0)
        {
          // We only add the root joint if we have a fixed base
          // (first body doesn't have any joint). Otherwise, the root joint is ignored.
          mjcfVisitor.addRootJoint(
            rootBody.bodyInertia, rootLinkName, rootBody.bodyPlacement, referenceConfig, qpos0,
            rootJoint, rootJointName);
        }
        else
        {
          if (rootJoint.has_value())
          {
            PINOCCHIO_THROW_IF(
              !rootJointName.has_value(), std::invalid_argument,
              "if root_joint is provided, root_joint_name must be also be provided.");

            mjcfVisitor << "WARNING: trying to add root joint '" << rootJointName.get()
                        << "' to a model which doesn't have a fixed base."
                        << " The provided root joint is therefore ignored.\n";
          }
        }

        for (const auto & entry : bodiesList)
        {
          fillModel(entry);
        }

        // We store the default configuration, obtained by parsing the <worldbody/>.
        // Equality constraints are typically defined w.r.t this default configuration qpos0.
        mapOfConfigs.insert(std::make_pair("qpos0", qpos0));

        // We update the joint placements so their base placement is matching with the
        // referenceConfig
        updateJointPlacementsFromReferenceConfig();

        for (const auto & entry : mapOfConfigs)
        {
          addKeyFrame(entry.second, entry.first);
        }
      }
    } // namespace details
  } // namespace mjcf
} // namespace pinocchio
