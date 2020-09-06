//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_joints_models_hpp__
#define __pinocchio_python_joints_models_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;


    // generic expose_joint_model : do nothing special
    template <class T>
    inline bp::class_<T>& expose_joint_model(bp::class_<T>& cl)
    {
      return cl;
    }

    // specialization for JointModelRevoluteUnaligned
    template<>
    inline bp::class_<JointModelRevoluteUnaligned>& expose_joint_model<JointModelRevoluteUnaligned> (bp::class_<JointModelRevoluteUnaligned> & cl)
    {
      return cl
               .def(bp::init<double, double, double> (bp::args("x", "y", "z"), "Init JointModelRevoluteUnaligned from the components x, y, z of the axis"))
               .def(bp::init<Eigen::Vector3d> (bp::args("axis"),
                                               "Init JointModelRevoluteUnaligned from an axis with x-y-z components"))
               .def_readwrite("axis",&JointModelRevoluteUnaligned::axis,
                              "Rotation axis of the JointModelRevoluteUnaligned.")
               ;
    }

    // specialization for JointModelPrismaticUnaligned
    template<>
    inline bp::class_<JointModelPrismaticUnaligned>& expose_joint_model<JointModelPrismaticUnaligned> (bp::class_<JointModelPrismaticUnaligned> & cl)
    {
      return cl
               .def(bp::init<double, double, double> (bp::args("x", "y", "z"),
                                                      "Init JointModelPrismaticUnaligned from the components x, y, z of the axis"))
               .def(bp::init<Eigen::Vector3d> (bp::args("axis"),
                                               "Init JointModelPrismaticUnaligned from an axis with x-y-z components"))
               .def_readwrite("axis",&JointModelPrismaticUnaligned::axis,
                              "Translation axis of the JointModelPrismaticUnaligned.")
               ;
    }

    // specialization for JointModelComposite

    struct JointModelCompositeAddJointVisitor : public boost::static_visitor<JointModelComposite &>
    {
      JointModelComposite & m_joint_composite;
      const SE3 & m_joint_placement;

      JointModelCompositeAddJointVisitor(JointModelComposite & joint_composite,
                                         const SE3 & joint_placement)
      : m_joint_composite(joint_composite)
      , m_joint_placement(joint_placement)
      {}

      template <typename JointModelDerived>
      JointModelComposite & operator()(JointModelDerived & jmodel) const
      {
        return m_joint_composite.addJoint(jmodel,m_joint_placement);
      }
    }; // struct JointModelCompositeAddJointVisitor

    static JointModelComposite & addJoint_proxy(JointModelComposite & joint_composite,
                                                const JointModelVariant & jmodel_variant,
                                                const SE3 & joint_placement = SE3::Identity())
    {
      return boost::apply_visitor(JointModelCompositeAddJointVisitor(joint_composite,joint_placement), jmodel_variant);
    }

    BOOST_PYTHON_FUNCTION_OVERLOADS(addJoint_proxy_overloads,addJoint_proxy,2,3)

    struct JointModelCompositeConstructorVisitor : public boost::static_visitor<JointModelComposite* >
    {
      const SE3 & m_joint_placement;

      JointModelCompositeConstructorVisitor(const SE3 & joint_placement)
      : m_joint_placement(joint_placement)
      {}

      template <typename JointModelDerived>
      JointModelComposite* operator()(JointModelDerived & jmodel) const
      {
        return new JointModelComposite(jmodel,m_joint_placement);
      }
    }; // struct JointModelCompositeConstructorVisitor

    static JointModelComposite* init_proxy1(const JointModelVariant & jmodel_variant)
    {
      return boost::apply_visitor(JointModelCompositeConstructorVisitor(SE3::Identity()), jmodel_variant);
    }

    static JointModelComposite* init_proxy2(const JointModelVariant & jmodel_variant,
                                            const SE3 & joint_placement)
    {
      return boost::apply_visitor(JointModelCompositeConstructorVisitor(joint_placement), jmodel_variant);
    }

    template<>
    inline bp::class_<JointModelComposite>& expose_joint_model<JointModelComposite> (bp::class_<JointModelComposite> & cl)
    {
      return cl
               .def(bp::init<const size_t> (bp::args("size"), "Init JointModelComposite with a defined size"))
               .def("__init__",
                    bp::make_constructor(init_proxy1,
                                         bp::default_call_policies(),
                                         bp::args("joint_model")
                                        ),
                    "Init JointModelComposite from a joint"
                   )
               .def("__init__",
                    bp::make_constructor(init_proxy2,
                                         bp::default_call_policies(),
                                         bp::args("joint_model","joint_placement")
                                        ),
                    "Init JointModelComposite from a joint and a placement"
                   )
               .add_property("joints",&JointModelComposite::joints)
               .add_property("jointPlacements",&JointModelComposite::jointPlacements)
               .add_property("njoints",&JointModelComposite::njoints)
               .def("addJoint",
                    &addJoint_proxy,
                    addJoint_proxy_overloads(bp::args("joint_model","joint_placement"),
                                             "Add a joint to the vector of joints."
                                            )[bp::return_internal_reference<>()]
                   )
               ;
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joint_models_hpp__
