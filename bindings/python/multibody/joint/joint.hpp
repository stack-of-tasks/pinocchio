//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_joint_joint_hpp__
#define __pinocchio_python_multibody_joint_joint_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    struct JointModelPythonVisitor
    : public boost::python::def_visitor< JointModelPythonVisitor >
    {
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>(bp::arg("self")))
        // All are add_properties cause ReadOnly
        .add_property("id",&getId)
        .add_property("idx_q",&getIdx_q)
        .add_property("idx_v",&getIdx_v)
        .add_property("nq",&getNq)
        .add_property("nv",&getNv)
        .def("hasConfigurationLimit", &JointModel::hasConfigurationLimit,
             "Return vector of boolean if joint has configuration limits.")
        .def("hasConfigurationLimitInTangent", &JointModel::hasConfigurationLimitInTangent,
             "Return vector of boolean if joint has configuration limits in tangent space.")
        .def("setIndexes",
             &JointModel::setIndexes,
             bp::args("self","id","idx_q","idx_v"))
        .def("hasSameIndexes",
             &JointModel::hasSameIndexes<JointModel>,
             bp::args("self","other"),
             "Check if this has same indexes than other.")
        .def("shortname",
             &JointModel::shortname,
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

      static JointIndex getId( const JointModel & self ) { return self.id(); }
      static int getIdx_q(const JointModel & self) {return self.idx_q();}
      static int getIdx_v(const JointModel & self) {return self.idx_v();}
      static int getNq(const JointModel & self) {return self.nq();}
      static int getNv(const JointModel & self) {return self.nv();}

      static void expose()
      {
        bp::class_<JointModel>("JointModel",
                               "Generic Joint Model",
                               bp::no_init)
        .def(bp::init<JointModel>(bp::args("self","other")))
        .def(JointModelPythonVisitor())
        .def(PrintableVisitor<JointModel>())
        ;
      }

    }; 
    
}} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_multibody_joint_joint_hpp__
