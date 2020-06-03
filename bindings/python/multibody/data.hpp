//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_data_hpp__
#define __pinocchio_python_data_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/serialization/data.hpp"

#include <eigenpy/memory.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/bindings/python/utils/macros.hpp"
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

#define ADD_DATA_PROPERTY(NAME,DOC) \
  PINOCCHIO_ADD_PROPERTY(Data,NAME,DOC)
      
#define ADD_DATA_PROPERTY_READONLY(NAME,DOC) \
  PINOCCHIO_ADD_PROPERTY_READONLY(Data,NAME,DOC)
      
#define ADD_DATA_PROPERTY_READONLY_BYVALUE(NAME,DOC) \
  PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Data,NAME,DOC)


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
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(tau,"Joint torques (output of RNEA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(nle,"Non Linear Effects (output of nle algorithm)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(ddq,"Joint accelerations (output of ABA)")
        .ADD_DATA_PROPERTY(Ycrb,"Composite Rigid Body Inertia of the sub-tree")
        .ADD_DATA_PROPERTY(oYcrb,"Composite Rigid Body Inertia of the sub-tree expressed in the WORLD coordinate system.")
        .ADD_DATA_PROPERTY(Yaba,"Articulated Body Inertia of the sub-tree")
        .ADD_DATA_PROPERTY(oYaba,"Articulated Body Inertia of the sub-tree expressed in the WORLD coordinate system.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(M,"The joint space inertia matrix")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Minv,"The inverse of the joint space inertia matrix")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(C,"The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v")
        .ADD_DATA_PROPERTY(Fcrb,"Spatial forces set, used in CRBA")
        .ADD_DATA_PROPERTY(lastChild,"Index of the last child (for CRBA)")
        .ADD_DATA_PROPERTY(nvSubtree,"Dimension of the subtree motion space (for CRBA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(U,"Joint Inertia square root (upper triangle)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(D,"Diagonal of UDUT inertia decomposition")
        .ADD_DATA_PROPERTY(parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
        .ADD_DATA_PROPERTY(nvSubtree_fromRow,"Subtree of the current row index (used in Cholesky)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(J,"Jacobian of joint placement")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(dJ,"Time variation of the Jacobian of joint placement (data.J).")
        .ADD_DATA_PROPERTY(iMf,"Body placement wrt to algorithm end effector.")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Ag,
                                            "Centroidal matrix which maps from joint velocity to the centroidal momentum.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(dAg,
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
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Jcom,"Jacobian of center of mass.")

        .ADD_DATA_PROPERTY_READONLY_BYVALUE(C,"Joint space Coriolis matrix.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(dtau_dq,"Partial derivative of the joint torque vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(dtau_dv,"Partial derivative of the joint torque vector with respect to the joint velocity.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(ddq_dq,"Partial derivative of the joint acceleration vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(ddq_dv,"Partial derivative of the joint acceleration vector with respect to the joint velocity.")

        .ADD_DATA_PROPERTY_READONLY_BYVALUE(kinetic_energy,"Kinetic energy in [J] of the system computed by computeKineticEnergy")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(potential_energy,"Potential energy in [J] of the system computed by computePotentialEnergy")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(mechanical_energy,"Mechanical energy in [J] of the system computed by computeMechanicalEnergy")

        .ADD_DATA_PROPERTY_READONLY_BYVALUE(lambda_c,"Lagrange Multipliers linked to the contact forces")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(impulse_c,"Lagrange Multipliers linked to the contact impulses")
        .ADD_DATA_PROPERTY(contact_chol,"Contact Cholesky decomposition.")

        .ADD_DATA_PROPERTY_READONLY_BYVALUE(dq_after,"Generalized velocity after the impact.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(staticRegressor,"Static regressor.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(jointTorqueRegressor,"Joint torque regressor.")
        
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
        
        StdAlignedVectorPythonVisitor<Vector3, true>::expose("StdVec_vec3d");
        StdAlignedVectorPythonVisitor<Matrix6x, true>::expose("StdVec_Matrix6x");
        StdVectorPythonVisitor<int,std::allocator<int>,true>::expose("StdVec_int");
      }

    };
    
  }} // namespace pinocchio::python

#undef ADD_DATA_PROPERTY
#undef ADD_DATA_PROPERTY_READONLY
#undef ADD_DATA_PROPERTY_READONLY_BYVALUE

#endif // ifndef __pinocchio_python_data_hpp__
