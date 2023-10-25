//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_python_joint_variants_hpp__
#define __pinocchio_python_joint_variants_hpp__

#include <boost/python.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<class JointModelDerived>
    struct JointModelDerivedPythonVisitor
    : public boost::python::def_visitor< JointModelDerivedPythonVisitor<JointModelDerived> >
    {
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>(bp::arg("self")))
        // All are add_properties cause ReadOnly
        .add_property("id",&get_id)
        .add_property("idx_q",&get_idx_q)
        .add_property("idx_v",&get_idx_v)
        .add_property("nq",&get_nq)
        .add_property("nv",&get_nv)
        .add_property("hasConfigurationLimit", 
                      &JointModelDerived::hasConfigurationLimit,
                      "Return vector of boolean if joint has configuration limits.")
        .add_property("hasConfigurationLimitInTangent",
                      &JointModelDerived::hasConfigurationLimitInTangent,
                      "Return vector of boolean if joint has configuration limits in tangent space.")
        .def("setIndexes",
             &JointModelDerived::setIndexes,
             bp::args("self","id","idx_q","idx_v"))
        .def("hasSameIndexes",
             &JointModelDerived::template hasSameIndexes<JointModelDerived>,
             bp::args("self","other"),
             "Check if this has same indexes than other.")
        .def("shortname",
             &JointModelDerived::shortname,
             bp::arg("self"),
             "Returns string indicating the joint type (class name):" 
                "\n\t- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]"
                "\n\t- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z"
                "\n\t- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]"
                "\n\t- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z"
                "\n\t- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]"
                "\n\t- JointModelPlanar: Planar joint"
                "\n\t- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z"
                "\n\t- JointModelSphericalZYX: Spherical joint (3D rotation)"
                "\n\t- JointModelTranslation: Translation joint (3D translation)"
                "\n\t- JointModelFreeFlyer: Joint enabling 3D rotation and translations." 
                )
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }

      static JointIndex get_id(const JointModelDerived & self)
      { return self.id(); }
      static int get_idx_q(const JointModelDerived & self)
      { return self.idx_q(); }
      static int get_idx_v(const JointModelDerived & self)
      { return self.idx_v(); }
      static int get_nq(const JointModelDerived & self)
      { return self.nq(); }
      static int get_nv(const JointModelDerived & self)
      { return self.nv(); }

    };

    template<class JointDataDerived>
    struct JointDataDerivedPythonVisitor
    : public boost::python::def_visitor< JointDataDerivedPythonVisitor<JointDataDerived> >
    {
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
          // All are add_properties cause ReadOnly
          .add_property("S",&get_S)
          .add_property("M",&get_M)
          .add_property("v",&get_v)
          .add_property("c",&get_c)
          .add_property("U",&get_U)
          .add_property("Dinv",&get_Dinv)
          .add_property("UDinv",&get_UDinv)
          .def("shortname",&JointDataDerived::shortname)
        
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
        ;
      }

      static typename JointDataDerived::Constraint_t get_S(const JointDataDerived & self)
      { return self.S_accessor(); }
      static typename JointDataDerived::Transformation_t get_M(const JointDataDerived & self)
      { return self.M_accessor(); }
      static typename JointDataDerived::Motion_t get_v(const JointDataDerived & self)
      { return self.v_accessor(); }
      static typename JointDataDerived::Bias_t get_c(const JointDataDerived & self)
      { return self.c_accessor(); }
      static typename JointDataDerived::U_t get_U(const JointDataDerived & self)
      { return self.U_accessor(); }
      static typename JointDataDerived::D_t get_Dinv(const JointDataDerived & self)
      { return self.Dinv_accessor(); }
      static typename JointDataDerived::UD_t get_UDinv(const JointDataDerived & self)
      { return self.UDinv_accessor(); }
      
      static void expose()
      {}

    }; 
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_joint_variants_hpp__
