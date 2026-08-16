//
// Copyright (c) 2015-2024 CNRS INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/parsers/mjcf.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/parsers/mjcf.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace mjcf
  {
    namespace details
    {
      struct MjcfGraph;
      struct MjcfJoint;
      struct MjcfGeom;
      struct MjcfSite;

      using JointType = ::pinocchio::urdf::details::JointType;

      class PINOCCHIO_PARSERS_DLLAPI MjcfVisitor : public ::pinocchio::urdf::details::UrdfVisitor
      {
      public:
        typedef ::pinocchio::urdf::details::UrdfVisitor Base;
        typedef Base::Model Model;
        typedef Base::JointModel JointModel;

        MjcfVisitor(Model & model)
        : Base(model)
        {
        }

        void addRootJoint(
          const Inertia & Y,
          const std::string & body_name,
          const SE3 & body_placement,
          Eigen::VectorXd & reference_config,
          Eigen::VectorXd & qpos0,
          const boost::optional<const JointModel &> root_joint,
          const boost::optional<const std::string &> root_joint_name)
        {
          // Base::addRootJoint folds Y into the universe inertia untransformed.
          const Inertia universe_inertia_before = Base::model.inertias[0];

          Base::addRootJoint(Y, body_name, root_joint, root_joint_name);

          if (root_joint.has_value())
          {
            Eigen::VectorXd qroot(root_joint->template lieGroup<LieGroupMap>().neutral());

            // update the reference_config with the size of the root joint
            reference_config.conservativeResize(reference_config.size() + qroot.size());
            reference_config.tail(qroot.size()) = qroot;

            // convert qroot to mujoco's convention for quaternions
            int qpos0_size = static_cast<int>(qpos0.size());
            qpos0.conservativeResize(qpos0_size + qroot.size());
            qpos0.tail(qroot.size()) = qroot;
            if (root_joint->shortname() == "JointModelFreeFlyer")
            {
              qpos0.tail(4) << qroot(6), qroot(3), qroot(4), qroot(5);
            }
            else if (root_joint->shortname() == "JointModelSpherical")
            {
              qpos0.tail(4) << qroot(3), qroot(0), qroot(1), qroot(2);
            }
            else if (root_joint->shortname() == "JointModelComposite")
            {
              JointModel root_joint_copy = root_joint.get();
              root_joint_copy.setIndexes(0, 0, 0);
              for (const auto & joint_ :
                   boost::get<JointModelComposite>(root_joint_copy.toVariant()).joints)
              {
                if (joint_.shortname() == "JointModelSpherical")
                {
                  int idx_q_ = joint_.idx_q();
                  qpos0.segment(qpos0_size + idx_q_, 4) << qroot(idx_q_ + 3), qroot(idx_q_ + 0),
                    qroot(idx_q_ + 1), qroot(idx_q_ + 2);
                }
              }
            }
          }
          else
          {
            // Unlike URDF, MJCF lets the fixed root body carry its own pos/quat, which
            // Base::addRootJoint ignores for both the body frame and the universe inertia.
            const FrameIndex bodyFrameId = Base::model.getFrameId(body_name, BODY);
            Base::model.inertias[0] = universe_inertia_before + body_placement.act(Y);
            Base::model.frames[bodyFrameId].placement = body_placement;
          }
        }
      };

      /// @brief Informations that are stocked in the XML tag compile.
      ///
      struct PINOCCHIO_PARSERS_DLLAPI MjcfCompiler
      {
      public:
        // Global attribute to use limit that are in the model or not
        bool autolimits = true;

        // Attribute to keep or not the full path of files specified in the model
        bool strippath = false;
        // Directory where all the meshes are (can be relative or absolute)
        std::string meshdir;
        // Directory where all the textures are (can be relative or absolute)
        std::string texturedir;

        // Value for angle conversion (Mujoco default - degrees)
        double angle_converter = boost::math::constants::pi<double>() / 180.0;
        // Euler Axis to use to convert angles representation to quaternion
        Eigen::Matrix3d mapEulerAngles = Eigen::Matrix3d::Identity();

        // Value to crop the mass (if mass < boundMass, mass = boundMass)
        double boundMass = 0;
        // Value to crop the diagonal of the inertia matrix (if mass < boundMass, mass = boundMass)
        double boundInertia = 0;

        // True, false or auto - auto = indeterminate
        boost::logic::tribool inertiafromgeom = boost::logic::indeterminate;

        /// @brief Convert the angle in radian if model was declared to use degree
        /// @param angle_ angle to convert
        /// @return converted angle
        double convertAngle(const double & angle_) const;

        /// @brief Convert the euler angles according to the convention declared in the compile tag.
        /// @param angles Euler angles
        /// @return Quaternion representation of the euler angles
        Eigen::Matrix3d convertEuler(const Eigen::Vector3d & angles) const;
      };

      /// @brief Structure to stock all default classes information
      struct MjcfClass
      {
      public:
        typedef boost::property_tree::ptree ptree;

        // name of the default class
        std::string className;
        // Ptree associated with the class name
        ptree classElement;
      };

      /// @brief All Bodies informations extracted from mjcf model
      struct MjcfBody
      {
      public:
        // Name of the body
        std::string bodyName;
        // Name of the parent
        std::string bodyParent;
        // Name of the default class used by this body (optional)
        std::string bodyClassName;
        // Special default class, that is common to all bodies and children if not specified
        // otherwise
        std::string childClass;

        // Position of the body wrt to the previous body
        SE3 bodyPlacement = SE3::Identity();
        // Body inertia
        Inertia bodyInertia = Inertia::Identity();

        // Vector of joints associated with the body
        std::vector<MjcfJoint> jointChildren;
        // Vector of geometries associated with the body
        std::vector<MjcfGeom> geomChildren;
        // Vector of sites
        std::vector<MjcfSite> siteChildren;
      };

      /// @brief All joint limits
      struct PINOCCHIO_PARSERS_DLLAPI RangeJoint
      {
        // Min effort
        Eigen::VectorXd minEffort;
        // Max effort
        Eigen::VectorXd maxEffort;
        // Min velocity
        Eigen::VectorXd minVel;
        // Max velocity
        Eigen::VectorXd maxVel;
        // Max position
        Eigen::VectorXd maxConfig;
        // Min position
        Eigen::VectorXd minConfig;
        // Position margin
        Eigen::VectorXd configLimitMargin;

        // Join Stiffness
        Eigen::VectorXd springStiffness;
        //  joint position or angle in which the joint spring (if any) achieves equilibrium
        Eigen::VectorXd springReference;

        // Min friction applied in this joint
        Eigen::VectorXd minDryFriction;
        // Max friction applied in this joint
        Eigen::VectorXd maxDryFriction;
        // Damping applied by this joint.
        Eigen::VectorXd damping;

        // Armature inertia created by this joint
        Eigen::VectorXd armature;
        // Dry friction.
        // double frictionLoss = 0.;

        RangeJoint() = default;
        explicit RangeJoint(double v)
        {
          const double infty = std::numeric_limits<double>::infinity();
          minVel = Eigen::VectorXd::Constant(1, -infty);
          maxVel = Eigen::VectorXd::Constant(1, infty);
          minEffort = Eigen::VectorXd::Constant(1, -infty);
          maxEffort = Eigen::VectorXd::Constant(1, infty);
          minConfig = Eigen::VectorXd::Constant(1, -infty);
          maxConfig = Eigen::VectorXd::Constant(1, infty);
          configLimitMargin = Eigen::VectorXd::Constant(1, 0);
          springStiffness = Eigen::VectorXd::Constant(1, v);
          springReference = Eigen::VectorXd::Constant(1, v);
          minDryFriction = Eigen::VectorXd::Constant(1, 0.);
          maxDryFriction = Eigen::VectorXd::Constant(1, 0.);
          damping = Eigen::VectorXd::Constant(1, 0.);
          armature = Eigen::VectorXd::Constant(1, 0.);
        }

        /// @brief Set dimension to the limits to match the joint nq and nv.
        /// @tparam Nq joint configuration
        /// @tparam Nv joint velocity
        /// @return Range with new dimension
        template<int Nq, int Nv>
        RangeJoint setDimension() const;

        /// @brief Concatenate 2 rangeJoint
        /// @tparam Nq old_range, joint configuration
        /// @tparam Nv old_range, joint velocity
        /// @param range to concatenate with
        /// @return Concatenated range.
        template<int Nq, int Nv>
        RangeJoint concatenate(const RangeJoint & range) const;
      };

      /// @brief All joint information parsed from the mjcf model
      struct PINOCCHIO_PARSERS_DLLAPI MjcfJoint
      {
      public:
        typedef boost::property_tree::ptree ptree;

        // Name of the joint
        std::string jointName = "free";
        // Placement of the joint wrt to its body - default Identity
        SE3 jointPlacement = SE3::Identity();

        // axis of the joint - default "0 0 1"
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        // Limits that applie to this joint
        RangeJoint range{1};

        // type of the joint (hinge, ball, slide, free) - default "hinge"
        std::string jointType = "hinge";

        double posRef = 0.; // only possible for hinge and slides

        /// @param el ptree joint node
        /// @param currentBody body to which the joint belongs to
        /// @param currentGraph current Mjcf graph (needed to get compiler information)
        void fill(const ptree & el, const MjcfBody & currentBody, const MjcfGraph & currentGraph);

        /// @brief Go through a joint node (default class or not) and parse info into the structure
        /// @param el ptree joint node
        /// @param use_limits whether to parse the limits or not
        void
        goThroughElement(const ptree & el, bool use_limits, const MjcfCompiler & currentCompiler);
      };
      /// @brief All informations related to a mesh are stored here
      struct MjcfMesh
      {
        // Scale of the mesh
        Eigen::Vector3d scale = Eigen::Vector3d::Constant(1);
        // Path to the mesh file
        std::string filePath;
        // Vertices of the mesh
        Eigen::MatrixX3d vertices;
      };

      /// @brief All informations related to a texture are stored here
      struct MjcfTexture
      {
        // [2d, cube, skybox], “cube”
        std::string textType = "cube";
        // Path to the texture file
        std::string filePath;
        // Size of the grid if needed
        Eigen::Vector2d gridsize = Eigen::Vector2d::Constant(1);
      };

      /// @brief All informations related to material are stored here
      struct PINOCCHIO_PARSERS_DLLAPI MjcfMaterial
      {
        typedef boost::property_tree::ptree ptree;
        // Color of the material
        Eigen::Vector4d rgba = Eigen::Vector4d::Constant(1);

        float reflectance = 0;

        float shininess = 0.5;

        float specular = 0.5;

        float emission = 0;
        // name of the texture to apply on the material
        std::string texture;

        /// @brief Go through a ptree node to look for material tag related
        /// @param el ptree material node
        void goThroughElement(const ptree & el);
      };

      struct PINOCCHIO_PARSERS_DLLAPI MjcfGeom
      {
      public:
        typedef boost::property_tree::ptree ptree;

        // Kind of possible geometry
        enum TYPE
        {
          VISUAL,
          COLLISION,
          BOTH
        };
        // name of the geometry object
        std::string geomName;

        // [plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh, sdf], “sphere”
        std::string geomType = "sphere";

        // Kind of the geometry object
        TYPE geomKind = BOTH;

        // Contact filtering and dynamic pair (used here to determine geometry kind)
        int contype = 1;
        int conaffinity = 1;
        // Geometry group (used to determine geometry kind)
        int group = 0;

        // String that hold size parameter
        std::string sizeS;
        // Optional in case fromto tag is used
        boost::optional<std::string> fromtoS;
        // Size parameter
        Eigen::VectorXd size;

        // Color of the geometry
        Eigen::Vector4d rgba = Eigen::Vector4d::Constant(1);

        // Name of the material applied on the geometry
        std::string materialName;
        // Name of the mesh (optional)
        std::string meshName;

        // Density for computing the mass
        double density = 1000;
        // If mass is only on the outer layer of the geometry
        bool shellinertia = false;

        // Geometry Placement in parent body. Center of the frame of geometry is the center of mass.
        SE3 geomPlacement = SE3::Identity();
        // Inertia of the geometry obj
        Inertia geomInertia = Inertia::Identity();
        // optional mass (if not defined, will use density)
        boost::optional<double> massGeom;

        /// @brief Find the geometry kind
        void findKind();

        /// @brief Compute Geometry size based on sizeS and fromtoS
        void computeSize();

        /// @brief Compute geometry inertia
        void computeInertia();

        /// @brief Fill Geometry element with info from ptree nodes
        void fill(const ptree & el, const MjcfBody & currentBody, const MjcfGraph & currentGraph);

        /// @bried Go through a geom ptree node, to gather informations
        void goThroughElement(const ptree & el, const MjcfGraph & currentGraph);
      };

      struct PINOCCHIO_PARSERS_DLLAPI MjcfSite
      {
        typedef boost::property_tree::ptree ptree;

        SE3 sitePlacement = SE3::Identity();

        std::string siteName;

        void fill(const ptree & el, const MjcfBody & currentBody, const MjcfGraph & currentGraph);
        void goThroughElement(const ptree & el, const MjcfGraph & currentGraph);
      };

      /*
      typedef struct mjsEquality_ {      // equality specification
        mjsElement* element;             // element type
        mjString* name;                  // name
        mjtEq type;                      // constraint type
        double data[mjNEQDATA];          // type-dependent data
        mjtByte active;                  // is equality initially active
        mjString* name1;                 // name of object 1
        mjString* name2;                 // name of object 2
        mjtNum solref[mjNREF];           // solver reference
        mjtNum solimp[mjNIMP];           // solver impedance
        mjString* info;                  // message appended to errors
      } mjsEquality;
      */
      struct PINOCCHIO_PARSERS_DLLAPI MjcfEquality
      {
        typedef boost::property_tree::ptree ptree;

        // Optional name of the equality constraint
        std::string name;

        // Type of the constraint: (connect for now)
        std::string type;

        // // Optional class for setting unspecified attributes
        // std::string class;

        // active: 'true' or 'false' (default: 'true')
        // solref and solimp

        // Name of the first body participating in the constraint
        std::string body1;
        // Name of the second body participating in the constraint (optional, default: world)
        std::string body2;

        // Name of the first site participating in the constraint
        std::string site1;
        // Name of the second site participating in the constraint (optional, default: world)
        std::string site2;

        // Coordinates of the 3D anchor point where the two bodies are connected.
        // Specified relative to the local coordinate frame of the first body.
        Eigen::Vector3d anchor = Eigen::Vector3d::Zero();

        // Relative pose of the frame anchor position where the two bodies are welded.
        // Specified relative to the local coordinate frame of the first body.
        SE3 relpose = SE3::Identity();

        // The default value of relpose is not a valid position and trigger.
        // a special event where relative pose is calculated in qref
        bool use_qref_relpose = true;
      };

      /// @brief The graph which contains all information taken from the mjcf file
      struct PINOCCHIO_PARSERS_DLLAPI MjcfGraph
      {
      public:
        typedef boost::property_tree::ptree ptree;
        typedef std::vector<std::string> VectorOfStrings;
        typedef std::unordered_map<std::string, MjcfBody> BodyMap_t;
        typedef std::unordered_map<std::string, MjcfClass> ClassMap_t;
        typedef std::unordered_map<std::string, MjcfMaterial> MaterialMap_t;
        typedef std::unordered_map<std::string, MjcfMesh> MeshMap_t;
        typedef std::unordered_map<std::string, MjcfTexture> TextureMap_t;
        typedef std::unordered_map<std::string, Eigen::VectorXd> ConfigMap_t;
        typedef std::map<std::string, MjcfEquality> EqualityMap_t;

        // Compiler Info needed to properly parse the rest of file
        MjcfCompiler compilerInfo;
        // Map of default classes
        ClassMap_t mapOfClasses;
        // Map of bodies
        BodyMap_t mapOfBodies;
        // Map of Materials
        MaterialMap_t mapOfMaterials;
        // Map of Meshes
        MeshMap_t mapOfMeshes;
        // Map of textures
        TextureMap_t mapOfTextures;
        // Map of model configurations
        ConfigMap_t mapOfConfigs;
        // Map of equality constraints
        EqualityMap_t mapOfEqualities;

        // @brief Reference configuration that allows pinocchio's FK to match mujoco's FK.
        // When doing a FK in mujoco, there are two differences with pinocchio's FK:
        // 1) In mujoco, a freeflyer's placement w.r.t its parent is never taken into consideration.
        //    Only the freeflyer's component of the configuration vector are used for the FK.
        //    For all other joints, the joints' components AND their placement w.r.t their parents
        //    are taken into consideration.
        //    In pinocchio, the placements w.r.t parents are always taken into consideration.
        // 2) In mujoco, for hinge and slide joints, a reference can be used to offset the "zero" of
        //    these joints.
        //
        // If we were to simply parse an MJCF file to construct a pinocchio model, we would find
        // that FK(mujoco, q) = FK(pinocchio, q - qref).
        // However, to make it easier to switch between mujoco and pinocchio, it's handy to use the
        // same q in both frameworks. Therefore, after parsing the model, we update the placement of
        // each pinocchio joint such that FK(mujoco, q) = FK(pinocchio, q).
        // Therefore, the `referenceConfig` vector is used to perform this update.
        Eigen::VectorXd referenceConfig;

        /// @brief Default configuration obtained when parsing an MJCF file.
        /// This configuration is not a keyframe and is only obtained by parsing the succession of
        /// <body><joint>...</body> inside <worldbody>
        /// It is handy to store this default configuration as it is typically used to define
        /// equality constraints in a MJCF file.
        Eigen::VectorXd qpos0;

        // property tree where xml file is stored
        ptree pt;

        // World body, mainly used to store geoms that are outside the body hierarchy
        MjcfBody worldBody;

        // Ordered list of bodies
        VectorOfStrings bodiesList;

        // Name of the model
        std::string modelName;
        std::string modelPath;

        // Mjcf Visitor to add joint and body
        MjcfVisitor & mjcfVisitor;
        typedef MjcfVisitor::Model Model;
        typedef MjcfVisitor::JointModel JointModel;

        /// @brief graph constructor
        /// @param mjcfVisitor
        MjcfGraph(MjcfVisitor & mjcfVisitor, const std::string & modelPath)
        : modelPath(modelPath)
        , mjcfVisitor(mjcfVisitor)
        {
        }

        /// @brief Convert pose of an mjcf element into SE3
        /// @param el ptree element with all the pose element
        /// @return pose in SE3
        SE3 convertPosition(const ptree & el) const;

        /// @brief Convert Inertia of an mjcf element into Inertia model of pinocchio
        /// @param el ptree element with all the inertial information
        /// @return Inertia element in pinocchio
        Inertia convertInertiaFromMjcf(const ptree & el) const;

        /// @brief Go through the default part of the file and get all the class name. Fill the
        /// mapOfDefault for later use.
        /// @param el ptree element. Root of the default
        void parseDefault(ptree & el, const ptree & parent, const std::string & parentTag);

        /// @brief Inspect the worlbody tag to retrieve potential geoms that are not attached
        /// to any bodies.
        /// @param el root of the tree
        void parseWorldBodyGeoms(const ptree & el);

        /// @brief Go through the main body of the mjcf file "worldbody" to get all the info ready
        /// to create the model.
        /// @param el root of the tree
        /// @param parentName name of the parentBody in the robot tree
        void parseJointAndBody(
          const ptree & el,
          const boost::optional<std::string> & childClass,
          const std::string & parentName = "");

        /// @brief Parse all the info from the compile node into compilerInfo
        /// @param el ptree compile node
        void parseCompiler(const ptree & el);

        /// @brief Parse all the info from a texture node
        /// @param el ptree texture node
        void parseTexture(const ptree & el);

        /// @brief Parse all the info from a material node
        /// @param el ptree material node
        void parseMaterial(const ptree & el);

        /// @brief Parse all the info from a mesh node
        /// @param el ptree mesh node
        void parseMesh(const ptree & el);

        /// @brief Parse all the info from the meta tag asset (mesh, material, texture)
        /// @param el ptree texture node
        void parseAsset(const ptree & el);

        /// @brief Parse all the info from the meta tag keyframe
        /// @param el ptree keyframe node
        void parseKeyFrame(const ptree & el);

        /// @brief Parse all the info from the equality tag
        /// @param el ptree equality node
        void parseEquality(const ptree & el);

        /// @brief parse the mjcf file into a graph
        void parseGraph();

        /// @brief parse the mjcf file into a graph
        /// @param xmlStr xml file name
        void parseGraphFromXML(const std::string & xmlStr);

        /// @brief Create a joint to add to the joint composite if needed
        /// @tparam TypeX joint with axis X
        /// @tparam TypeY joint with axis Y
        /// @tparam TypeZ joint with axis Z
        /// @tparam TypeUnaligned joint with axis unaligned
        /// @param axis axis of the joint
        /// @return one of the joint with the right axis
        template<typename TypeX, typename TypeY, typename TypeZ, typename TypeUnaligned>
        JointModel createJoint(const Eigen::Vector3d & axis);

        /// @brief Add a joint to the model. only needed when a body has a solo joint child
        /// @param jointInfo The joint to add to the tree
        /// @param currentBody The body associated with the joint
        /// @param bodyInJoint Position of the body wrt to its joint
        void
        addSoloJoint(const MjcfJoint & jointInfo, const MjcfBody & currentBody, SE3 & bodyInJoint);

        /// @brief Use all the infos that were parsed from the xml file to add a body and joint to
        /// the model
        /// @param nameOfBody Name of the body to add
        void fillModel(const std::string & nameOfBody);

        /// @brief Use the reference configuration that was parsed to update the joint placements so
        /// that pinocchio and mujoco forward kinematics match given the same configuration vector.
        /// See @ref referenceConfig for more information.
        void updateJointPlacementsFromReferenceConfig();

        /// @brief Fill the pinocchio model with all the infos from the graph
        /// @param rootJoint optional root joint to add to the base of the model. The root joint
        /// will be ignored if the model doesn't have a fixed base.
        /// @param rootJointName name of the optional root joint.
        /// @note If a root joint provided and the graph has a fixed base, this root joint will be
        /// added at the base of the model.
        /// If the graph doesn't have a fixed base (the first body has one or more child joints),
        /// this root joint will be ignored.
        void parseRootTree(
          const boost::optional<const JointModel &> rootJoint = boost::none,
          const boost::optional<const std::string &> rootJointName = boost::none);

        /// @brief Fill reference configuration for a body and all it's associated dof
        /// @param currentBody body to check
        void fillReferenceConfig(const MjcfBody & currentBody);

        /// @brief Add a keyframe to the model (ie reference configuration)
        /// @param keyframe Keyframe to add
        /// @param keyName Name of the keyframe
        void addKeyFrame(const Eigen::VectorXd & keyframe, const std::string & keyName);

        /// @brief Parse the equality constraints and add them to the model
        /// @param model Model to add the constraints to
        /// @param point_anchor_constraint_models Vector of contact models to add the constraints to
        /// @param frame_anchor_constraint_models Vector of contact models to add the constraints to
        void parseContactInformation(
          const Model & model,
          std::vector<PointAnchorConstraintModel> & point_anchor_constraint_models,
          std::vector<FrameAnchorConstraintModel> & frame_anchor_constraint_models);

        /// @brief Fill geometry model with all the info taken from the mjcf model file
        /// @param type Type of geometry to parse (COLLISION or VISUAL)
        /// @param geomModel geometry model to fill
        /// @param meshLoader mesh loader from coal
        void parseGeomTree(
          const GeometryType & type, GeometryModel & geomModel, ::coal::MeshLoaderPtr & meshLoader);
      };
      namespace internal
      {
        inline std::istringstream getConfiguredStringStream(const std::string & str)
        {
          std::istringstream posStream(str);
          posStream.exceptions(std::ios::badbit);
          return posStream;
        }

        template<int N>
        inline Eigen::Matrix<double, N, 1> getVectorFromStream(const std::string & str)
        {
          std::istringstream stream = getConfiguredStringStream(str);
          Eigen::Matrix<double, N, 1> vector;
          for (int i = 0; i < N; i++)
            stream >> vector(i);

          return vector;
        }

        inline Eigen::VectorXd getUnknownSizeVectorFromStream(const std::string & str)
        {
          std::istringstream stream = getConfiguredStringStream(str);
          std::vector<double> vector;
          double elem;
          while (stream >> elem)
          {
            vector.push_back(elem);
          }

          Eigen::VectorXd returnVector(vector.size());
          for (std::size_t i = 0; i < vector.size(); i++)
            returnVector(static_cast<Eigen::Index>(i)) = vector[i];

          return returnVector;
        }
      } // namespace internal
    } // namespace details
  } // namespace mjcf
} // namespace pinocchio
