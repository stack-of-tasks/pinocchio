//
// Copyright (c) 2015-2023 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_multibody_model_hpp__
#define __pinocchio_python_multibody_model_hpp__

#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/serialization/model.hpp"

#include <boost/python/overloads.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/algorithm/check.hpp"

#include "pinocchio/bindings/python/utils/cast.hpp"
#include "pinocchio/bindings/python/utils/macros.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-map.hpp"
#include "pinocchio/bindings/python/utils/pickle.hpp"
#include "pinocchio/bindings/python/utils/pickle-map.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/serialization/serializable.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Model)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Model>
    struct ModelPythonVisitor : public bp::def_visitor<ModelPythonVisitor<Model>>
    {
    public:
      typedef typename Model::Scalar Scalar;

      typedef typename Model::Index Index;
      typedef typename Model::JointIndex JointIndex;
      typedef typename Model::JointModel JointModel;
      typedef typename JointModel::JointModelVariant JointModelVariant;
      typedef typename Model::FrameIndex FrameIndex;
      typedef typename Model::IndexVector IndexVector;

      typedef typename Model::SE3 SE3;
      typedef typename Model::Motion Motion;
      typedef typename Model::Force Force;
      typedef typename Model::Frame Frame;
      typedef typename Model::Inertia Inertia;

      typedef typename Model::Data Data;

      typedef typename Model::VectorXs VectorXs;

    public:
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<>(bp::arg("self"), "Default constructor. Constructs an empty model."))
          .def(bp::init<const Model &>((bp::arg("self"), bp::arg("clone")), "Copy constructor"))

          // Class Members
          .add_property("nq", &Model::nq, "Dimension of the configuration vector representation.")
          .add_property("nv", &Model::nv, "Dimension of the velocity vector space.")
          .add_property("njoints", &Model::njoints, "Number of joints.")
          .add_property("nbodies", &Model::nbodies, "Number of bodies.")
          .add_property("nframes", &Model::nframes, "Number of frames.")
          .add_property(
            "inertias", &Model::inertias, "Vector of spatial inertias supported by each joint.")
          .def_readwrite(
            "jointPlacements", &Model::jointPlacements,
            "Vector of joint placements: placement of a joint *i* wrt its parent joint frame.")
          .add_property("joints", &Model::joints, "Vector of joint models.")
          .add_property(
            "idx_qs", &Model::idx_qs,
            "Vector of starting index of the *i*th  joint in the configuration space.")
          .add_property(
            "nqs", &Model::nqs, "Vector of dimension of the  joint configuration subspace.")
          .add_property(
            "idx_vs", &Model::idx_vs,
            "Starting index of the *i*th joint in the tangent configuration space.")
          .add_property("nvs", &Model::nvs, "Dimension of the *i*th joint tangent subspace.")
          .add_property(
            "parents", &Model::parents,
            "Vector of parent joint indexes. The parent of joint *i*, denoted *li*, "
            "corresponds to li==parents[i].")
          .add_property(
            "children", &Model::children,
            "Vector of children index. Chidren of the *i*th joint, denoted *mu(i)* "
            "corresponds to the set (i==parents[k] for k in mu(i)).")
          .add_property("names", &Model::names, "Name of the joints.")
          .def_readwrite("name", &Model::name, "Name of the model.")
          .def_readwrite(
            "referenceConfigurations", &Model::referenceConfigurations,
            "Map of reference configurations, indexed by user given names.")

          .def_readwrite("armature", &Model::armature, "Armature vector.")
          .def_readwrite(
            "rotorInertia", &Model::rotorInertia, "Vector of rotor inertia parameters.")
          .def_readwrite(
            "rotorGearRatio", &Model::rotorGearRatio, "Vector of rotor gear ratio parameters.")
          .def_readwrite("friction", &Model::friction, "Vector of joint friction parameters.")
          .def_readwrite("damping", &Model::damping, "Vector of joint damping parameters.")
          .def_readwrite("effortLimit", &Model::effortLimit, "Joint max effort.")
          .def_readwrite("velocityLimit", &Model::velocityLimit, "Joint max velocity.")
          .def_readwrite(
            "lowerPositionLimit", &Model::lowerPositionLimit, "Limit for joint lower position.")
          .def_readwrite(
            "upperPositionLimit", &Model::upperPositionLimit, "Limit for joint upper position.")

          .def_readwrite("frames", &Model::frames, "Vector of frames contained in the model.")

          .def_readwrite(
            "supports", &Model::supports,
            "Vector of supports. supports[j] corresponds to the list of joints on the "
            "path between\n"
            "the current *j* to the root of the kinematic tree.")

          .def_readwrite(
            "subtrees", &Model::subtrees,
            "Vector of subtrees. subtree[j] corresponds to the subtree supported by the joint j.")

          .def_readwrite(
            "gravity", &Model::gravity,
            "Motion vector corresponding to the gravity field expressed in the world Frame.")

          // Class Methods
          .def(
            "addJoint", &ModelPythonVisitor::addJoint0,
            bp::args("self", "parent_id", "joint_model", "joint_placement", "joint_name"),
            "Adds a joint to the kinematic tree. The joint is defined by its placement relative "
            "to its parent joint and its name.")
          .def(
            "addJoint", &ModelPythonVisitor::addJoint1,
            bp::args(
              "self", "parent_id", "joint_model", "joint_placement", "joint_name", "max_effort",
              "max_velocity", "min_config", "max_config"),
            "Adds a joint to the kinematic tree with given bounds. The joint is defined by its "
            "placement relative to its parent joint and its name."
            "This signature also takes as input effort, velocity limits as well as the bounds "
            "on the joint configuration.")
          .def(
            "addJoint", &ModelPythonVisitor::addJoint2,
            bp::args(
              "self", "parent_id", "joint_model", "joint_placement", "joint_name", "max_effort",
              "max_velocity", "min_config", "max_config", "friction", "damping"),
            "Adds a joint to the kinematic tree with given bounds. The joint is defined by its "
            "placement relative to its parent joint and its name.\n"
            "This signature also takes as input effort, velocity limits as well as the bounds "
            "on the joint configuration.\n"
            "The user should also provide the friction and damping related to the joint.")
          .def(
            "addJointFrame", &Model::addJointFrame,
            (bp::arg("self"), bp::arg("joint_id"), bp::arg("frame_id") = 0),
            "Add the joint provided by its joint_id as a frame to the frame tree.\n"
            "The frame_id may be optionally provided.")
          .def(
            "appendBodyToJoint", &Model::appendBodyToJoint,
            bp::args("self", "joint_id", "body_inertia", "body_placement"),
            "Appends a body to the joint given by its index. The body is defined by its "
            "inertia, its relative placement regarding to the joint and its name.")

          .def(
            "addBodyFrame", &Model::addBodyFrame,
            bp::args("self", "body_name", "parentJoint", "body_placement", "previous_frame"),
            "add a body to the frame tree")
          .def(
            "getBodyId", &Model::getBodyId, bp::args("self", "name"),
            "Return the index of a frame of type BODY given by its name")
          .def(
            "existBodyName", &Model::existBodyName, bp::args("self", "name"),
            "Check if a frame of type BODY exists, given its name")
          .def(
            "getJointId", &Model::getJointId, bp::args("self", "name"),
            "Return the index of a joint given by its name")
          .def(
            "existJointName", &Model::existJointName, bp::args("self", "name"),
            "Check if a joint given by its name exists")

          .def(
            "getFrameId", &Model::getFrameId,
            (bp::arg("self"), bp::arg("name"),
             bp::arg("type") = (FrameType)(JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR)),
            "Returns the index of the frame given by its name and its type."
            "If the frame is not in the frames vector, it returns the current size of the "
            "frames vector.")

          .def(
            "existFrame", &Model::existFrame,
            (bp::arg("self"), bp::arg("name"),
             bp::arg("type") = (FrameType)(JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR)),
            "Returns true if the frame given by its name exists inside the Model with the given "
            "type.")

          .def(
            "addFrame", &Model::addFrame,
            (bp::arg("self"), bp::arg("frame"), bp::arg("append_inertia") = true),
            "Add a frame to the vector of frames. If append_inertia set to True, "
            "the inertia value contained in frame will be added to the inertia supported by the "
            "parent joint.")

          .def(
            "createData", &ModelPythonVisitor::createData, bp::arg("self"),
            "Create a Data object for the given model.")

          .def(
            "check", (bool(Model::*)(const Data &) const) & Model::check, bp::args("self", "data"),
            "Check consistency of data wrt model.")

          .def(
            "hasConfigurationLimit", &Model::hasConfigurationLimit, bp::args("self"),
            "Returns list of boolean if joints have configuration limit.")
          .def(
            "hasConfigurationLimitInTangent", &Model::hasConfigurationLimitInTangent,
            bp::args("self"),
            "Returns list of boolean if joints have configuration limit in tangent space  .")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS

          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif

          .PINOCCHIO_ADD_STATIC_PROPERTY_READONLY_BYVALUE(
            Model, gravity981, "Default gravity field value on the Earth.");

        bp::register_ptr_to_python<std::shared_ptr<Model>>();
      }

      static JointIndex addJoint0(
        Model & model,
        JointIndex parent_id,
        const JointModel & jmodel,
        const SE3 & joint_placement,
        const std::string & joint_name)
      {
        return model.addJoint(parent_id, jmodel, joint_placement, joint_name);
      }

      static JointIndex addJoint1(
        Model & model,
        JointIndex parent_id,
        const JointModel & jmodel,
        const SE3 & joint_placement,
        const std::string & joint_name,
        const VectorXs & max_effort,
        const VectorXs & max_velocity,
        const VectorXs & min_config,
        const VectorXs & max_config)
      {
        return model.addJoint(
          parent_id, jmodel, joint_placement, joint_name, max_effort, max_velocity, min_config,
          max_config);
      }

      static JointIndex addJoint2(
        Model & model,
        JointIndex parent_id,
        const JointModel & jmodel,
        const SE3 & joint_placement,
        const std::string & joint_name,
        const VectorXs & max_effort,
        const VectorXs & max_velocity,
        const VectorXs & min_config,
        const VectorXs & max_config,
        const VectorXs & friction,
        const VectorXs & damping)
      {
        return model.addJoint(
          parent_id, jmodel, joint_placement, joint_name, max_effort, max_velocity, min_config,
          max_config, friction, damping);
      }

      static Data createData(const Model & model)
      {
        return Data(model);
      }

      ///
      /// \brief Provide equivalent to python list index function for
      ///        vectors.
      ///
      /// \param[in] x The input vector.
      /// \param[in] v The value of to look for in the vector.
      ///
      /// \return The index of the matching element of the vector. If
      ///         no element is found, return the size of the vector.
      ///
      template<typename T>
      static Index index(std::vector<T> const & x, typename std::vector<T>::value_type const & v)
      {
        Index i = 0;
        for (typename std::vector<T>::const_iterator it = x.begin(); it != x.end(); ++it, ++i)
        {
          if (*it == v)
          {
            return i;
          }
        }
        return x.size();
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        typedef typename Model::ConfigVectorMap ConfigVectorMap;
        typedef bp::map_indexing_suite<ConfigVectorMap, false> map_indexing_suite;
        StdVectorPythonVisitor<std::vector<Index>, true>::expose("StdVec_Index");
        serialize<std::vector<Index>>();
        StdVectorPythonVisitor<std::vector<IndexVector>>::expose("StdVec_IndexVector");
        serialize<std::vector<IndexVector>>();
        StdVectorPythonVisitor<std::vector<std::string>, true>::expose("StdVec_StdString");
        StdVectorPythonVisitor<std::vector<bool>, true>::expose("StdVec_Bool");
        StdVectorPythonVisitor<std::vector<Scalar>, true>::expose("StdVec_Scalar");

#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
        bp::scope().attr("StdVec_Double") = bp::scope().attr("StdVec_Scalar"); // alias
#endif

        serialize<std::vector<std::string>>();
        serialize<std::vector<bool>>();
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
        serialize<std::vector<Scalar>>();
#endif
        bp::class_<typename Model::ConfigVectorMap>("StdMap_String_VectorXd")
          .def(map_indexing_suite())
          .def_pickle(PickleMap<typename Model::ConfigVectorMap>())
          .def(details::overload_base_get_item_for_std_map<typename Model::ConfigVectorMap>());

        bp::class_<Model>("Model", "Articulated Rigid Body model", bp::no_init)
          .def(ModelPythonVisitor())
          .def(CastVisitor<Model>())
          .def(ExposeConstructorByCastVisitor<Model, ::pinocchio::Model>())
          .def(SerializableVisitor<Model>())
          .def(PrintableVisitor<Model>())
          .def(CopyableVisitor<Model>())
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
          .def_pickle(PickleFromStringSerialization<Model>())
#endif
          ;
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_multibody_model_hpp__
