//
// Copyright (c) 2015-2021 CNRS INRIA
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
#include "pinocchio/bindings/python/serialization/serializable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-map.hpp"
#include "pinocchio/bindings/python/utils/pickle-map.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Model)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload,Model::getFrameId,1,2)
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload,Model::existFrame,1,2)
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addJointFrame_overload,Model::addJointFrame,1,2)
  
    template<typename Model>
    struct PickleModel : bp::pickle_suite
    {
      static bp::tuple getinitargs(const Model &)
      {
        return bp::make_tuple();
      }

      static bp::tuple getstate(const Model & model)
      {
        const std::string str(model.saveToString());
        return bp::make_tuple(bp::str(str));
      }

      static void setstate(Model & model, bp::tuple tup)
      {
        if(bp::len(tup) == 0 || bp::len(tup) > 1)
        {
          throw eigenpy::Exception("Pickle was not able to reconstruct the model from the loaded data.\n"
                                   "The pickle data structure contains too many elements.");
        }
        
        bp::object py_obj = tup[0];
        boost::python::extract<std::string> obj_as_string(py_obj.ptr());
        if(obj_as_string.check())
        {
          const std::string str = obj_as_string;
          model.loadFromString(str);
        }
        else
        {
          throw eigenpy::Exception("Pickle was not able to reconstruct the model from the loaded data.\n"
                                   "The entry is not a string.");
        }
        
      }
      
      static bool getstate_manages_dict() { return true; }
      
    };
    
    template<typename Model>
    struct ModelPythonVisitor
    : public bp::def_visitor< ModelPythonVisitor<Model> >
    {
    public:
      typedef typename Model::Scalar Scalar;
      
      typedef typename Model::Index Index;
      typedef typename Model::JointIndex JointIndex;
      typedef typename Model::FrameIndex FrameIndex;
      typedef typename Model::IndexVector IndexVector;
      
      typedef typename Model::SE3 SE3;
      typedef typename Model::Motion Motion;
      typedef typename Model::Force Force;
      typedef typename Model::Frame Frame;
      typedef typename Model::Inertia Inertia;
      
      typedef typename Model::Data Data;
      
      typedef typename Model::VectorXs VectorXs;
      
      BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addFrame_overload,Model::addFrame,1,2)
      
    public:

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>(bp::arg("self"),
                        "Default constructor. Constructs an empty model."))
        
        // Class Members
        .add_property("nq", &Model::nq)
        .add_property("nv", &Model::nv)
        .add_property("njoints", &Model::njoints)
        .add_property("nbodies", &Model::nbodies)
        .add_property("nframes", &Model::nframes)
        .add_property("inertias",&Model::inertias)
        .add_property("jointPlacements",&Model::jointPlacements)
        .add_property("joints",&Model::joints)
        .add_property("idx_qs",&Model::idx_qs)
        .add_property("nqs",&Model::nqs)
        .add_property("idx_vs",&Model::idx_vs)
        .add_property("nvs",&Model::nvs)          
        .add_property("parents",&Model::parents)
        .add_property("names",&Model::names)
        .def_readwrite("name",&Model::name)
        .def_readwrite("referenceConfigurations", &Model::referenceConfigurations)
        
        .def_readwrite("rotorInertia",&Model::rotorInertia,
                       "Vector of rotor inertia parameters.")
        .def_readwrite("rotorGearRatio",&Model::rotorGearRatio,
                       "Vector of rotor gear ratio parameters.")
        .def_readwrite("friction",&Model::friction,
                       "Vector of joint friction parameters.")
        .def_readwrite("damping",&Model::damping,
                       "Vector of joint damping parameters.")
        .def_readwrite("effortLimit",&Model::effortLimit,
                       "Joint max effort.")
        .def_readwrite("velocityLimit",&Model::velocityLimit,
                       "Joint max velocity.")
        .def_readwrite("lowerPositionLimit",&Model::lowerPositionLimit,
                       "Limit for joint lower position.")
        .def_readwrite("upperPositionLimit",&Model::upperPositionLimit,
                       "Limit for joint upper position.")
        
        .def_readwrite("frames",&Model::frames,
                       "Vector of frames contained in the model.")
        
        .def_readwrite("supports",
                       &Model::supports,
                       "Vector of supports. supports[j] corresponds to the list of joints on the path between\n"
                       "the current *j* to the root of the kinematic tree.")
        
        .def_readwrite("subtrees",
                       &Model::subtrees,
                       "Vector of subtrees. subtree[j] corresponds to the subtree supported by the joint j.")
        
        .def_readwrite("gravity",&Model::gravity,
                       "Motion vector corresponding to the gravity field expressed in the world Frame.")
        
        // Class Methods
        .def("addJoint",&ModelPythonVisitor::addJoint0,
             bp::args("self","parent_id","joint_model","joint_placement","joint_name"),
             "Adds a joint to the kinematic tree. The joint is defined by its placement relative to its parent joint and its name.")
        .def("addJoint",&ModelPythonVisitor::addJoint1,
             bp::args("self","parent_id","joint_model","joint_placement","joint_name",
                      "max_effort","max_velocity","min_config","max_config"),
             "Adds a joint to the kinematic tree with given bounds. The joint is defined by its placement relative to its parent joint and its name."
             "This signature also takes as input effort, velocity limits as well as the bounds on the joint configuration.")
        .def("addJoint",&ModelPythonVisitor::addJoint2,
             bp::args("self","parent_id","joint_model","joint_placement","joint_name",
                      "max_effort","max_velocity","min_config","max_config",
                      "friction","damping"),
             "Adds a joint to the kinematic tree with given bounds. The joint is defined by its placement relative to its parent joint and its name.\n"
             "This signature also takes as input effort, velocity limits as well as the bounds on the joint configuration.\n"
             "The user should also provide the friction and damping related to the joint.")
        .def("addJointFrame", &Model::addJointFrame,
             addJointFrame_overload(bp::args("self","joint_id", "frame_id"),
                                    "Add the joint provided by its joint_id as a frame to the frame tree.\n"
                                    "The frame_id may be optionally provided."))
        .def("appendBodyToJoint",&Model::appendBodyToJoint,
             bp::args("self","joint_id","body_inertia","body_placement"),
             "Appends a body to the joint given by its index. The body is defined by its inertia, its relative placement regarding to the joint and its name.")
        
        .def("addBodyFrame", &Model::addBodyFrame, bp::args("self","body_name", "parentJoint", "body_placement", "previous_frame"), "add a body to the frame tree")
        .def("getBodyId",&Model::getBodyId, bp::args("self","name"), "Return the index of a frame of type BODY given by its name")
        .def("existBodyName", &Model::existBodyName, bp::args("self","name"), "Check if a frame of type BODY exists, given its name")
        .def("getJointId",&Model::getJointId, bp::args("self","name"), "Return the index of a joint given by its name")
        .def("existJointName", &Model::existJointName, bp::args("self","name"), "Check if a joint given by its name exists")
        
        .def("getFrameId",&Model::getFrameId,getFrameId_overload(bp::args("self","name","type"),"Returns the index of the frame given by its name and its type. If the frame is not in the frames vector, it returns the current size of the frames vector."))
        
        .def("existFrame",&Model::existFrame,existFrame_overload(bp::args("self","name","type"),"Returns true if the frame given by its name exists inside the Model with the given type."))
        
        .def("addFrame",&Model::addFrame,
             addFrame_overload((bp::arg("self"), bp::arg("frame"), bp::arg("append_inertia") = true),
                               "Add a frame to the vector of frames. If append_inertia set to True, "
                               "the inertia value contained in frame will be added to the inertia supported by the parent joint."))
        
        .def("createData",
             &ModelPythonVisitor::createData,bp::arg("self"),
             "Create a Data object for the given model.")
        
        .def("check",(bool (Model::*)(const Data &) const) &Model::check,bp::args("self","data"),
             "Check consistency of data wrt model.")
        .def("hasConfigurationLimit",&Model::hasConfigurationLimit, bp::args("self"), "Returns list of boolean if joints have configuration limit.")
        .def("hasConfigurationLimitInTangent",&Model::hasConfigurationLimitInTangent, bp::args("self"), "Returns list of boolean if joints have configuration limit in tangent space  .")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }

      static JointIndex addJoint0(Model & model,
                                  JointIndex parent_id,
                                  const JointModel & jmodel,
                                  const SE3 & joint_placement,
                                  const std::string & joint_name)
      {
        return model.addJoint(parent_id,jmodel,joint_placement,joint_name);
      }

      static JointIndex addJoint1(Model & model,
                                  JointIndex parent_id,
                                  const JointModel & jmodel,
                                  const SE3 & joint_placement,
                                  const std::string & joint_name,
                                  const VectorXs & max_effort,
                                  const VectorXs & max_velocity,
                                  const VectorXs & min_config,
                                  const VectorXs & max_config)
      {
        return model.addJoint(parent_id,jmodel,joint_placement,joint_name,
                              max_effort,max_velocity,min_config,max_config);
      }
      
      static JointIndex addJoint2(Model & model,
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
        return model.addJoint(parent_id,jmodel,joint_placement,joint_name,
                              max_effort,max_velocity,min_config,max_config,
                              friction,damping);
      }

      static Data createData(const Model & model) { return Data(model); }

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
      static Index index(std::vector<T> const& x,
                                typename std::vector<T>::value_type const& v)
      {
        Index i = 0;
        for(typename std::vector<T>::const_iterator it = x.begin(); it != x.end(); ++it, ++i)
        {
          if(*it == v)
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
        typedef bp::map_indexing_suite<ConfigVectorMap,false> map_indexing_suite;
        StdVectorPythonVisitor<Index>::expose("StdVec_Index");
        serialize< std::vector<Index> >();
        StdVectorPythonVisitor<IndexVector>::expose("StdVec_IndexVector");
        serialize< std::vector<IndexVector> >();
        StdVectorPythonVisitor<std::string>::expose("StdVec_StdString");
        serialize< std::vector<std::string> >();
        StdVectorPythonVisitor<bool>::expose("StdVec_Bool");
        serialize< std::vector<bool> >();
        StdVectorPythonVisitor<Scalar>::expose("StdVec_Double");
        serialize< std::vector<Scalar> >();
        bp::class_<typename Model::ConfigVectorMap>("StdMap_String_VectorXd")
          .def(map_indexing_suite())
          .def_pickle(PickleMap<typename Model::ConfigVectorMap>())
          .def(details::overload_base_get_item_for_std_map<typename Model::ConfigVectorMap>());

        bp::class_<Model>("Model",
                          "Articulated Rigid Body model",
                          bp::no_init)
        .def(ModelPythonVisitor())
        .def(SerializableVisitor<Model>())
        .def(PrintableVisitor<Model>())
        .def(CopyableVisitor<Model>())
        .def_pickle(PickleModel<Model>())
        ;
      }
    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_multibody_model_hpp__
