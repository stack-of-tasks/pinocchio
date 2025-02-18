//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_data_hpp__
#define __pinocchio_python_multibody_data_hpp__

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/serialization/data.hpp"

#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/macros.hpp"
#include "pinocchio/bindings/python/serialization/serializable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Data)
#endif

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
        if (bp::len(tup) == 0 || bp::len(tup) > 1)
        {
          throw eigenpy::Exception(
            "Pickle was not able to reconstruct the model from the loaded data.\n"
            "The pickle data structure contains too many elements.");
        }

        bp::object py_obj = tup[0];
        boost::python::extract<std::string> obj_as_string(py_obj.ptr());
        if (obj_as_string.check())
        {
          const std::string str = obj_as_string;
          data.loadFromString(str);
        }
        else
        {
          throw eigenpy::Exception(
            "Pickle was not able to reconstruct the model from the loaded data.\n"
            "The entry is not a string.");
        }
      }

      static bool getstate_manages_dict()
      {
        return true;
      }
    };

    template<typename Data>
    struct DataPythonVisitor : public boost::python::def_visitor<DataPythonVisitor<Data>>
    {
      typedef typename Data::Matrix6 Matrix6;
      typedef typename Data::Matrix6x Matrix6x;
      typedef typename Data::Matrix3x Matrix3x;
      typedef typename Data::Vector3 Vector3;

    public:
#define ADD_DATA_PROPERTY(NAME, DOC) PINOCCHIO_ADD_PROPERTY(Data, NAME, DOC)

#define ADD_DATA_PROPERTY_READONLY(NAME, DOC) PINOCCHIO_ADD_PROPERTY_READONLY(Data, NAME, DOC)

#define ADD_DATA_PROPERTY_READONLY_BYVALUE(NAME, DOC)                                              \
  PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Data, NAME, DOC)

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<>(bp::arg("self"), "Default constructor."))
          .def(bp::init<const context::Model &>(
            bp::args("self", "model"), "Constructs a data structure from a given model."))

          .ADD_DATA_PROPERTY(
            joints,
            "Vector of JointData associated to each JointModel stored in the related model.")
          .ADD_DATA_PROPERTY(
            a, "Vector of joint accelerations expressed in the local frame of the joint.")
          .ADD_DATA_PROPERTY(
            oa, "Joint spatial acceleration expressed at the origin of the world frame.")
          .ADD_DATA_PROPERTY(
            a_gf, "Joint spatial acceleration containing also the contribution of "
                  "the gravity acceleration")
          .ADD_DATA_PROPERTY(
            oa_gf, "Joint spatial acceleration containing also the contribution of the gravity "
                   "acceleration, but expressed at the origin of the world frame.")

          .ADD_DATA_PROPERTY(
            v, "Vector of joint velocities expressed in the local frame of the joint.")
          .ADD_DATA_PROPERTY(ov, "Vector of joint velocities expressed at the origin of the world.")

          .ADD_DATA_PROPERTY(f, "Vector of body forces expressed in the local frame of the joint.")
          .ADD_DATA_PROPERTY(of, "Vector of body forces expressed at the origin of the world.")
          .ADD_DATA_PROPERTY(
            of_augmented, "Vector of body forces expressed at the origin of the "
                          "world, in the context of lagrangian formulation")
          .ADD_DATA_PROPERTY(
            h, "Vector of spatial momenta expressed in the local frame of the joint.")
          .ADD_DATA_PROPERTY(oh, "Vector of spatial momenta expressed at the origin of the world.")
          .ADD_DATA_PROPERTY(oMi, "Body absolute placement (wrt world)")
          .ADD_DATA_PROPERTY(oMf, "frames absolute placement (wrt world)")
          .ADD_DATA_PROPERTY(liMi, "Body relative placement (wrt parent)")
          .ADD_DATA_PROPERTY(tau, "Joint torques (output of RNEA)")
          .ADD_DATA_PROPERTY(nle, "Non Linear Effects (output of nle algorithm)")
          .ADD_DATA_PROPERTY(ddq, "Joint accelerations (output of ABA)")
          .ADD_DATA_PROPERTY(Ycrb, "Inertia of the sub-tree composit rigid body")
          .ADD_DATA_PROPERTY(
            oYcrb, "Composite Rigid Body Inertia of the sub-tree expressed in the "
                   "WORLD coordinate system.")
          .ADD_DATA_PROPERTY(Yaba, "Articulated Body Inertia of the sub-tree")
          .ADD_DATA_PROPERTY(
            oYaba,
            "Articulated Body Inertia of the sub-tree expressed in the WORLD coordinate system.")
          .ADD_DATA_PROPERTY(oL, "Acceleration propagator.")
          .ADD_DATA_PROPERTY(oK, "Inverse articulated inertia.")
          .ADD_DATA_PROPERTY(M, "The joint space inertia matrix")
          .ADD_DATA_PROPERTY(Minv, "The inverse of the joint space inertia matrix")
          .ADD_DATA_PROPERTY(
            C, "The Coriolis C(q,v) matrix such that the Coriolis effects are "
               "given by c(q,v) = C(q,v)v")
          .ADD_DATA_PROPERTY(g, "Vector of generalized gravity (dim model.nv).")
          .ADD_DATA_PROPERTY(Fcrb, "Spatial forces set, used in CRBA")
          .ADD_DATA_PROPERTY(lastChild, "Index of the last child (for CRBA)")
          .ADD_DATA_PROPERTY(nvSubtree, "Dimension of the subtree motion space (for CRBA)")
          .ADD_DATA_PROPERTY(U, "Joint Inertia square root (upper triangle)")
          .ADD_DATA_PROPERTY(D, "Diagonal of UDUT inertia decomposition")
          .ADD_DATA_PROPERTY(parents_fromRow, "First previous non-zero row in M (used in Cholesky)")
          .ADD_DATA_PROPERTY(
            nvSubtree_fromRow, "Subtree of the current row index (used in Cholesky)")
          .ADD_DATA_PROPERTY(J, "Jacobian of joint placement")
          .ADD_DATA_PROPERTY(dJ, "Time variation of the Jacobian of joint placement (data.J).")
          .ADD_DATA_PROPERTY(iMf, "Body placement wrt to algorithm end effector.")

          .ADD_DATA_PROPERTY(Ivx, "Right variation of the inertia matrix.")
          .ADD_DATA_PROPERTY(vxI, "Left variation of the inertia matrix.")
          .ADD_DATA_PROPERTY(
            B, "Combined variations of the inertia matrix consistent with Christoffel symbols.")

          .ADD_DATA_PROPERTY(
            Ag, "Centroidal matrix which maps from joint velocity to the centroidal momentum.")
          .ADD_DATA_PROPERTY(dAg, "Time derivative of the centroidal momentum matrix Ag.")
          .ADD_DATA_PROPERTY(
            hg, "Centroidal momentum (expressed in the frame centered at the CoM "
                "and aligned with the world frame).")
          .ADD_DATA_PROPERTY(
            dhg, "Centroidal momentum time derivative (expressed in the frame "
                 "centered at the CoM and aligned with the world frame).")
          .ADD_DATA_PROPERTY(Ig, "Centroidal Composite Rigid Body Inertia.")

          .ADD_DATA_PROPERTY(com, "CoM position of the subtree starting at joint index i.")
          .ADD_DATA_PROPERTY(vcom, "CoM velocity of the subtree starting at joint index i.")
          .ADD_DATA_PROPERTY(acom, "CoM acceleration of the subtree starting at joint index i.")
          .ADD_DATA_PROPERTY(mass, "Mass of the subtree starting at joint index i.")
          .ADD_DATA_PROPERTY(Jcom, "Jacobian of center of mass.")

          .ADD_DATA_PROPERTY(
            dAdq,
            "Variation of the spatial acceleration set with respect to the joint configuration.")
          .ADD_DATA_PROPERTY(
            dAdv, "Variation of the spatial acceleration set with respect to the joint velocity.")
          .ADD_DATA_PROPERTY(
            dHdq, "Variation of the spatial momenta set with respect to the joint configuration.")
          .ADD_DATA_PROPERTY(
            dFdq, "Variation of the force set with respect to the joint configuration.")
          .ADD_DATA_PROPERTY(dFdv, "Variation of the force set with respect to the joint velocity.")
          .ADD_DATA_PROPERTY(
            dFda, "Variation of the force set with respect to the joint acceleration.")

          .ADD_DATA_PROPERTY(
            dtau_dq, "Partial derivative of the joint torque vector with respect "
                     "to the joint configuration.")
          .ADD_DATA_PROPERTY(
            dtau_dv,
            "Partial derivative of the joint torque vector with respect to the joint velocity.")
          .ADD_DATA_PROPERTY(
            ddq_dq, "Partial derivative of the joint acceleration vector with "
                    "respect to the joint configuration.")
          .ADD_DATA_PROPERTY(
            ddq_dv, "Partial derivative of the joint acceleration vector with "
                    "respect to the joint velocity.")
          .ADD_DATA_PROPERTY(
            ddq_dtau,
            "Partial derivative of the joint acceleration vector with respect to the joint torque.")
          .ADD_DATA_PROPERTY(
            dvc_dq, "Partial derivative of the constraint velocity vector with "
                    "respect to the joint configuration.")

          .ADD_DATA_PROPERTY(
            dac_dq, "Partial derivative of the contact acceleration vector with "
                    "respect to the joint configuration.")
          .ADD_DATA_PROPERTY(
            dac_dv, "Partial derivative of the contact acceleration vector vector "
                    "with respect to the joint velocity.")
          .ADD_DATA_PROPERTY(
            dac_da, "Partial derivative of the contact acceleration vector vector "
                    "with respect to the joint acceleration.")

          .ADD_DATA_PROPERTY(osim, "Operational space inertia matrix.")

          .ADD_DATA_PROPERTY_READONLY_BYVALUE(
            dlambda_dq, "Partial derivative of the contact force vector with "
                        "respect to the joint configuration.")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE(
            dlambda_dv,
            "Partial derivative of the contact force vector with respect to the joint velocity.")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE(
            dlambda_dtau,
            "Partial derivative of the contact force vector with respect to the torque.")
          .ADD_DATA_PROPERTY(
            kinetic_energy, "Kinetic energy in [J] computed by computeKineticEnergy")
          .ADD_DATA_PROPERTY(
            potential_energy, "Potential energy in [J] computed by computePotentialEnergy")
          .ADD_DATA_PROPERTY(
            mechanical_energy,
            "Mechanical energy in [J] of the system computed by computeMechanicalEnergy")

          .ADD_DATA_PROPERTY(lambda_c, "Lagrange Multipliers linked to contact forces")
          .ADD_DATA_PROPERTY(impulse_c, "Lagrange Multipliers linked to contact impulses")
          .ADD_DATA_PROPERTY(contact_chol, "Contact Cholesky decomposition.")
          .ADD_DATA_PROPERTY(
            primal_dual_contact_solution,
            "Right hand side vector when solving the contact dynamics KKT problem.")
          .ADD_DATA_PROPERTY(
            lambda_c_prox, "Proximal Lagrange Multipliers used in the computation "
                           "of the Forward Dynamics computations.")
          .ADD_DATA_PROPERTY(primal_rhs_contact, "Primal RHS in contact dynamic equations.")

          .ADD_DATA_PROPERTY(dq_after, "Generalized velocity after the impact.")
          .ADD_DATA_PROPERTY(staticRegressor, "Static regressor.")
          .ADD_DATA_PROPERTY(jointTorqueRegressor, "Joint torque regressor.")
          .ADD_DATA_PROPERTY(kineticEnergyRegressor, "Kinetic energy regressor.")
          .ADD_DATA_PROPERTY(potentialEnergyRegressor, "Potential energy regressor.")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif
          ;

        bp::register_ptr_to_python<std::shared_ptr<Data>>();
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<Data>(
          "Data",
          "Articulated rigid body data related to a Model.\n"
          "It contains all the data that can be modified by the Pinocchio algorithms.",
          bp::no_init)
          .def(DataPythonVisitor())
          .def(CopyableVisitor<Data>())
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
          .def(SerializableVisitor<Data>())
          .def_pickle(PickleData<Data>())
#endif
          ;

        typedef PINOCCHIO_ALIGNED_STD_VECTOR(Vector3) StdVec_Vector3;
        typedef PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6x) StdVec_Matrix6x;
        typedef PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6) StdVec_Matrix6;

        StdAlignedVectorPythonVisitor<Vector3, false>::expose(
          "StdVec_Vector3",
          eigenpy::details::overload_base_get_item_for_std_vector<StdVec_Vector3>());

        StdAlignedVectorPythonVisitor<Matrix6x, false>::expose(
          "StdVec_Matrix6x",
          eigenpy::details::overload_base_get_item_for_std_vector<StdVec_Matrix6x>());
        StdAlignedVectorPythonVisitor<Matrix6, false>::expose(
          "StdVec_Matrix6",
          eigenpy::details::overload_base_get_item_for_std_vector<StdVec_Matrix6>());
        StdVectorPythonVisitor<std::vector<int>, true>::expose("StdVec_int");
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
        serialize<typename StdAlignedVectorPythonVisitor<Vector3, false>::vector_type>();
        serialize<typename StdAlignedVectorPythonVisitor<Matrix6x, false>::vector_type>();
#endif
        serialize<std::vector<int>>();
      }
    };

  } // namespace python
} // namespace pinocchio

#undef ADD_DATA_PROPERTY
#undef ADD_DATA_PROPERTY_READONLY
#undef ADD_DATA_PROPERTY_READONLY_BYVALUE

#endif // ifndef __pinocchio_python_multibody_data_hpp__
