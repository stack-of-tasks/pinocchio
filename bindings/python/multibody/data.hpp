//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_data_hpp__
#define __pinocchio_python_multibody_data_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/serialization/data.hpp"

#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/bindings/python/serialization/serializable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

#include "pinocchio/bindings/python/utils/copyable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Data)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename Data>
    struct PickleData : bp::pickle_suite
    {
      static bp::tuple getinitargs(const Data &)
      {
        return bp::make_tuple();
      }

      static bp::tuple getstate(const Data & data)
      {
        const std::string str(data.saveToString());
        return bp::make_tuple(bp::str(str));
      }

      static void setstate(Data & data, bp::tuple tup)
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
          data.loadFromString(str);
        }
        else
        {
          throw eigenpy::Exception("Pickle was not able to reconstruct the model from the loaded data.\n"
                                   "The entry is not a string.");
        }

      }
    };
  
    struct DataPythonVisitor
      : public boost::python::def_visitor< DataPythonVisitor >
    {
      typedef Data::Matrix6x Matrix6x;
      typedef Data::Matrix3x Matrix3x;
      typedef Data::Vector3 Vector3;

    public:

#define ADD_DATA_PROPERTY(NAME,DOC)         \
      def_readwrite(#NAME,                  \
      &Data::NAME,                          \
      DOC)
      
#define ADD_DATA_PROPERTY_READONLY(NAME,DOC)      \
      def_readonly(#NAME,                         \
      &Data::NAME,                                \
      DOC)
      
#define ADD_DATA_PROPERTY_READONLY_BYVALUE(NAME,DOC)                            \
      add_property(#NAME,                                                       \
      make_getter(&Data::NAME,bp::return_value_policy<bp::return_by_value>()),  \
      DOC) 


      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>(bp::arg("self"),"Default constructor."))
        .def(bp::init<Model>(bp::arg("model"),"Constructs a data structure from a given model."))
        
        .ADD_DATA_PROPERTY(a,"Joint spatial acceleration")
        .ADD_DATA_PROPERTY(oa,
                           "Joint spatial acceleration expressed at the origin of the world frame.")
        .ADD_DATA_PROPERTY(a_gf,
                           "Joint spatial acceleration containing also the contribution of the gravity acceleration")
        .ADD_DATA_PROPERTY(oa_gf,"Joint spatial acceleration containing also the contribution of the gravity acceleration, but expressed at the origin of the world frame.")
        
        .ADD_DATA_PROPERTY(v,"Joint spatial velocity expressed in the joint frame.")
        .ADD_DATA_PROPERTY(ov,"Joint spatial velocity expressed at the origin of the world frame.")
        
        .ADD_DATA_PROPERTY(f,"Joint spatial force expresssed in the joint frame.")
        .ADD_DATA_PROPERTY(of,"Joint spatial force expresssed at the origin of the world frame.")
        .ADD_DATA_PROPERTY(h,"Vector of spatial momenta expressed in the local frame of the joint.")
        .ADD_DATA_PROPERTY(oMi,"Body absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(oMf,"frames absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(liMi,"Body relative placement (wrt parent)")
        .ADD_DATA_PROPERTY(tau,"Joint torques (output of RNEA)")
        .ADD_DATA_PROPERTY(nle,"Non Linear Effects (output of nle algorithm)")
        .ADD_DATA_PROPERTY(ddq,"Joint accelerations (output of ABA)")
        .ADD_DATA_PROPERTY(Ycrb,"Inertia of the sub-tree composit rigid body")
        .ADD_DATA_PROPERTY(M,"The joint space inertia matrix")
        .ADD_DATA_PROPERTY(Minv,"The inverse of the joint space inertia matrix")
        .ADD_DATA_PROPERTY(C,"The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v")
        .ADD_DATA_PROPERTY(Fcrb,"Spatial forces set, used in CRBA")
        .ADD_DATA_PROPERTY(lastChild,"Index of the last child (for CRBA)")
        .ADD_DATA_PROPERTY(nvSubtree,"Dimension of the subtree motion space (for CRBA)")
        .ADD_DATA_PROPERTY(U,"Joint Inertia square root (upper triangle)")
        .ADD_DATA_PROPERTY(D,"Diagonal of UDUT inertia decomposition")
        .ADD_DATA_PROPERTY(parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
        .ADD_DATA_PROPERTY(nvSubtree_fromRow,"Subtree of the current row index (used in Cholesky)")
        .ADD_DATA_PROPERTY(J,"Jacobian of joint placement")
        .ADD_DATA_PROPERTY(dJ,"Time variation of the Jacobian of joint placement (data.J).")
        .ADD_DATA_PROPERTY(iMf,"Body placement wrt to algorithm end effector.")
        
        .ADD_DATA_PROPERTY(Ag,
                           "Centroidal matrix which maps from joint velocity to the centroidal momentum.")
        .ADD_DATA_PROPERTY(dAg,
                           "Time derivative of the centroidal momentum matrix Ag.")
        .ADD_DATA_PROPERTY(hg,
                           "Centroidal momentum (expressed in the frame centered at the CoM and aligned with the world frame).")
        .ADD_DATA_PROPERTY(dhg,
                           "Centroidal momentum time derivative (expressed in the frame centered at the CoM and aligned with the world frame).")
        .ADD_DATA_PROPERTY(Ig,
                           "Centroidal Composite Rigid Body Inertia.")
        
        .ADD_DATA_PROPERTY(com,"CoM position of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(vcom,"CoM velocity of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(acom,"CoM acceleration of the subtree starting at joint index i..")
        .ADD_DATA_PROPERTY(mass,"Mass of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(Jcom,"Jacobian of center of mass.")

        .ADD_DATA_PROPERTY(C,"Joint space Coriolis matrix.")
        .ADD_DATA_PROPERTY(dtau_dq,"Partial derivative of the joint torque vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY(dtau_dv,"Partial derivative of the joint torque vector with respect to the joint velocity.")
        .ADD_DATA_PROPERTY(ddq_dq,"Partial derivative of the joint acceleration vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY(ddq_dv,"Partial derivative of the joint acceleration vector with respect to the joint velocity.")
        
        .ADD_DATA_PROPERTY(kinetic_energy,"Kinetic energy in [J] computed by computeKineticEnergy")
        .ADD_DATA_PROPERTY(potential_energy,"Potential energy in [J] computed by computePotentialEnergy")
        
        .ADD_DATA_PROPERTY(lambda_c,"Lagrange Multipliers linked to contact forces")
        .ADD_DATA_PROPERTY(impulse_c,"Lagrange Multipliers linked to contact impulses")
        
        .ADD_DATA_PROPERTY(dq_after,"Generalized velocity after the impact.")
        .ADD_DATA_PROPERTY(staticRegressor,"Static regressor.")
        .ADD_DATA_PROPERTY(jointTorqueRegressor,"Joint torque regressor.")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<Data>("Data",
                         "Articulated rigid body data related to a Model.\n"
                         "It contains all the data that can be modified by the Pinocchio algorithms.",
                         bp::no_init)
        .def(DataPythonVisitor())
        .def(CopyableVisitor<Data>())
        .def(SerializableVisitor<Data>())
        .def_pickle(PickleData<Data>());
        
        typedef PINOCCHIO_ALIGNED_STD_VECTOR(Vector3) StdVec_Vector3;
        typedef PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6x) StdVec_Matrix6x;
        
        StdAlignedVectorPythonVisitor<Vector3,false>::expose("StdVec_Vector3")
        .def(details::overload_base_get_item_for_std_vector<StdVec_Vector3>());
        StdAlignedVectorPythonVisitor<Matrix6x,false>::expose("StdVec_Matrix6x")
        .def(details::overload_base_get_item_for_std_vector<StdVec_Matrix6x>());
        StdVectorPythonVisitor<int>::expose("StdVec_Int");
      }

    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_multibody_data_hpp__
