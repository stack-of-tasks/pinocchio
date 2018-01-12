//
// Copyright (c) 2015-2017 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_python_model_hpp__
#define __se3_python_model_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/overloads.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/bindings/python/utils/eigen_container.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::Model)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload,Model::getFrameId,1,2)
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload,Model::existFrame,1,2)
    
    struct ModelPythonVisitor
      : public boost::python::def_visitor< ModelPythonVisitor >
    {
    public:
      typedef Model::Index Index;
      typedef Model::JointIndex JointIndex;
      typedef Model::FrameIndex FrameIndex;

    protected:
      struct addJointVisitor : public boost::static_visitor<Model::Index>
      {
        Model & m_model;
        const JointIndex m_parent_id;
        const SE3 & m_joint_placement;
        const std::string & m_joint_name;
        
        addJointVisitor(Model & model,
                        const JointIndex parent_id,
                        const SE3 & joint_placement,
                        const std::string & joint_name)
        : m_model(model)
        , m_parent_id(parent_id)
        , m_joint_placement(joint_placement)
        , m_joint_name(joint_name)
        {}
       
        template <typename JointModelDerived>
        JointIndex operator()(JointModelDerived & jmodel) const
        {
          return m_model.addJoint(m_parent_id,jmodel,m_joint_placement,m_joint_name);
        }
      }; // struct addJointVisitor
      
    public:

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>("Default constructor. Constructs an empty model."))
          // Class Members
        .add_property("nq", &Model::nq)
        .add_property("nv", &Model::nv)
        .add_property("njoints", &Model::njoints)
        .add_property("nbodies", &Model::nbodies)
        .add_property("nframes", &Model::nframes)
        .add_property("inertias",&Model::inertias)
        .add_property("jointPlacements",&Model::jointPlacements)
        .add_property("joints",&Model::joints)
        .add_property("parents",&Model::parents)
        .add_property("names",&Model::names)
        
        .add_property("neutralConfiguration",
                      make_getter(&Model::neutralConfiguration, bp::return_value_policy<bp::return_by_value>()),
                      make_setter(&Model::neutralConfiguration, bp::return_value_policy<bp::return_by_value>()),
                      "Joint's neutral configurations.")
        .add_property("effortLimit",
                      make_getter(&Model::effortLimit, bp::return_value_policy<bp::return_by_value>()),
                      make_setter(&Model::effortLimit, bp::return_value_policy<bp::return_by_value>()),
                      "Joint max effort.")
        .add_property("velocityLimit",
                      make_getter(&Model::velocityLimit, bp::return_value_policy<bp::return_by_value>()),
                      make_setter(&Model::velocityLimit, bp::return_value_policy<bp::return_by_value>()),
                      "Joint max velocity.")
        .add_property("lowerPositionLimit",
                      make_getter(&Model::lowerPositionLimit, bp::return_value_policy<bp::return_by_value>()),
                      make_setter(&Model::lowerPositionLimit, bp::return_value_policy<bp::return_by_value>()),
                      "Limit for joint lower position.")
        .add_property("upperPositionLimit",
                      make_getter(&Model::upperPositionLimit, bp::return_value_policy<bp::return_by_value>()),
                      make_setter(&Model::upperPositionLimit, bp::return_value_policy<bp::return_by_value>()),
                      "Limit for joint upper position.")
        
        .def_readwrite("frames",&Model::frames,"Vector of frames contained in the model.")

        .def_readwrite("subtrees",
                       &Model::subtrees,
                       "Vector of subtrees. subtree[j] corresponds to the subtree supported by the joint j.")
        
        .def_readwrite("gravity",&Model::gravity,"Motion vector corresponding to the gravity field expressed in the world Frame.")

          // Class Methods
          .def("addJoint",&ModelPythonVisitor::addJoint,bp::args("parent_id","joint_model","joint_placement","joint_name"),"Adds a joint to the kinematic tree. The joint is defined by its placement relative to its parent joint and its name.")
          .def("addJointFrame", &Model::addJointFrame, bp::args("jointIndex", "frameIndex"), "add the joint at index jointIndex as a frame to the frame tree")
          .def("appendBodyToJoint",&Model::appendBodyToJoint,bp::args("joint_id","body_inertia","body_placement"),"Appends a body to the joint given by its index. The body is defined by its inertia, its relative placement regarding to the joint and its name.")

          .def("addBodyFrame", &Model::addBodyFrame, bp::args("body_name", "parentJoint", "body_plaement", "previous_frame(parent frame)"), "add a body to the frame tree")
          .def("getBodyId",&Model::getBodyId, bp::args("name"), "Return the index of a frame of type BODY given by its name")
          .def("existBodyName", &Model::existBodyName, bp::args("name"), "Check if a frame of type BODY exists, given its name")
          .def("getJointId",&Model::getJointId, bp::args("name"), "Return the index of a joint given by its name")
          .def("existJointName", &Model::existJointName, bp::args("name"), "Check if a joint given by its name exists")
        
        .def("getFrameId",&Model::getFrameId,getFrameId_overload(bp::arg("name"),"Returns the index of the frame given by its name. If the frame is not in the frames vector, it returns the current size of the frames vector."))
        .def("getFrameId",&Model::getFrameId,getFrameId_overload(bp::args("name","type"),"Returns the index of the frame given by its name and its type. If the frame is not in the frames vector, it returns the current size of the frames vector."))
       
        .def("existFrame",&Model::existFrame,existFrame_overload(bp::arg("name"),"Returns true if the frame given by its name exists inside the Model.")) 
        .def("existFrame",&Model::existFrame,existFrame_overload(bp::args("name","type"),"Returns true if the frame given by its name exists inside the Model with the given type."))

        .def("addFrame",(bool (Model::*)(const std::string &,const JointIndex, const FrameIndex, const SE3 &,const FrameType &)) &Model::addFrame,bp::args("name","parent_id","placement","type"),"Add a frame to the vector of frames. See also Frame for more details. Returns False if the frame already exists.")
        .def("addFrame",(bool (Model::*)(const Frame &)) &Model::addFrame,bp::args("frame"),"Add a frame to the vector of frames.")

          .def("createData",&ModelPythonVisitor::createData)
          .def("BuildEmptyModel",&ModelPythonVisitor::maker_empty)
          .staticmethod("BuildEmptyModel")
          .def("BuildHumanoidSimple",&ModelPythonVisitor::maker_humanoidSimple)
          .staticmethod("BuildHumanoidSimple")
        
        .def("check",(bool (Model::*)(const Data &) const) &Model::check,bp::arg("data"),"Check consistency of data wrt model.")
          ;
      }

      
      static JointIndex addJoint(Model & model,
                                 JointIndex parent_id,
                                 bp::object jmodel,
                                 const SE3 & joint_placement,
                                 const std::string & joint_name)
      {
        JointModelVariant jmodel_variant = bp::extract<JointModelVariant> (jmodel)();
        return boost::apply_visitor(addJointVisitor(model,parent_id,joint_placement,joint_name), jmodel_variant);
      }
      
      static Data createData(const Model & model) { return Data(model); }
      static Model maker_empty() { return Model(); }
      
      static Model maker_humanoidSimple()
      {
        Model model;
        buildModels::humanoidSimple(model);
        return model;
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
      static Model::Index index(std::vector<T> const& x,
                                typename std::vector<T>::value_type const& v)
      {
        Model::Index i = 0;
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
        bp::class_< std::vector<Index> >("StdVec_Index")
          .def(bp::vector_indexing_suite< std::vector<Index> >());
        bp::class_< std::vector<Model::IndexVector> >("StdVec_IndexVector")
        .def(bp::vector_indexing_suite< std::vector<Model::IndexVector> >());
        bp::class_< std::vector<std::string> >("StdVec_StdString")
          .def(bp::vector_indexing_suite< std::vector<std::string> >())
          .def("index", &ModelPythonVisitor::index<std::string>);
        bp::class_< std::vector<bool> >("StdVec_Bool")
          .def(bp::vector_indexing_suite< std::vector<bool> >());
        bp::class_< std::vector<double> >("StdVec_double")
          .def(bp::vector_indexing_suite< std::vector<double> >()); 

        bp::class_<Model>("Model",
                          "Articulated rigid body model (const)",
                          bp::no_init)
        .def(ModelPythonVisitor())
        .def(PrintableVisitor<Model>())
        .def(CopyableVisitor<Model>())
        ;
      
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_model_hpp__

