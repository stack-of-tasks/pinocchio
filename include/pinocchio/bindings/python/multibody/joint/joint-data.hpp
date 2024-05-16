//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_multibody_joint_joint_data_hpp__
#define __pinocchio_python_multibody_joint_joint_data_hpp__

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename JointData>
    struct ExtractJointDataVariantTypeVisitor
    {
      typedef typename JointData::JointCollection JointCollection;
      typedef bp::object result_type;

      template<typename JointDataDerived>
      result_type operator()(const JointDataBase<JointDataDerived> & jdata) const
      {
        bp::object obj(boost::ref(jdata.derived()));
        return obj;
      }

      static result_type extract(const JointData & jdata)
      {
        return boost::apply_visitor(ExtractJointDataVariantTypeVisitor(), jdata);
      }
    };

    template<typename JointData>
    struct JointDataPythonVisitor
    {

      static void expose()
      {
        bp::class_<JointData>("JointData", "Generic Joint Data", bp::no_init)
          .def(
            bp::init<const typename JointData::JointDataVariant &>(bp::args("self", "joint_data")))
          .def(JointDataBasePythonVisitor<JointData>())
          .def(PrintableVisitor<JointData>())
          .def(
            "extract", ExtractJointDataVariantTypeVisitor<JointData>::extract, bp::arg("self"),
            "Returns a reference of the internal joint managed by the JointData",
            bp::with_custodian_and_ward_postcall<0, 1>());
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_multibody_joint_joint_data_hpp__
