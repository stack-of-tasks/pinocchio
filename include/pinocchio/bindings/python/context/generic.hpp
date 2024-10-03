//
// Copyright (c) 2021-2024 INRIA
//

#ifndef __pinocchio_python_context_generic_hpp__
#define __pinocchio_python_context_generic_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/algorithm/constraints/fwd.hpp"

#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  #include "pinocchio/multibody/pool/fwd.hpp"
  #ifdef PINOCCHIO_WITH_HPP_FCL
    #include "pinocchio/collision/pool/fwd.hpp"
  #endif
#endif

#include <eigenpy/eigen-typedef.hpp>

#include <Eigen/Sparse>

namespace pinocchio
{
  namespace python
  {

    // Forward declaration
    boost::python::object getScalarType();
    void exposeSpecificTypeFeatures();

    namespace context
    {

      typedef PINOCCHIO_PYTHON_SCALAR_TYPE Scalar;
      enum
      {
        Options = 0
      };

      // Eigen
      EIGENPY_MAKE_TYPEDEFS_ALL_SIZES(Scalar, Options, s);
      typedef Eigen::SparseMatrix<Scalar> SparseMatrix;
      typedef Eigen::SparseMatrix<Scalar, Eigen::RowMajor> RowSparseMatrix;
      typedef Eigen::Matrix<Scalar, 1, 1, Options, 1, 1> Matrix1s;
      typedef Eigen::Matrix<Scalar, 7, 1, Options> Vector7s;
      typedef Eigen::Quaternion<Scalar, Options> Quaternion;
      typedef Eigen::AngleAxis<Scalar> AngleAxis;

      // Spatial
      typedef SE3Tpl<Scalar, Options> SE3;
      typedef MotionTpl<Scalar, Options> Motion;
      typedef ForceTpl<Scalar, Options> Force;
      typedef InertiaTpl<Scalar, Options> Inertia;
      typedef PseudoInertiaTpl<Scalar, Options> PseudoInertia;
      typedef LogCholeskyParametersTpl<Scalar, Options> LogCholeskyParameters;
      typedef Symmetric3Tpl<Scalar, Options> Symmetric3;

      // Multibody
      typedef FrameTpl<Scalar, Options> Frame;
      typedef ModelTpl<Scalar, Options> Model;
      typedef DataTpl<Scalar, Options> Data;

      typedef JointCollectionDefaultTpl<Scalar, Options> JointCollectionDefault;

      // Joints
      typedef JointModelTpl<Scalar, Options> JointModel;
      typedef JointDataTpl<Scalar, Options> JointData;

      typedef JointDataRevoluteTpl<Scalar, Options, 0> JointDataRX;
      typedef JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;

      typedef JointDataRevoluteTpl<Scalar, Options, 1> JointDataRY;
      typedef JointModelRevoluteTpl<Scalar, Options, 1> JointModelRY;

      typedef JointDataRevoluteTpl<Scalar, Options, 2> JointDataRZ;
      typedef JointModelRevoluteTpl<Scalar, Options, 2> JointModelRZ;

      typedef JointModelRevoluteUnalignedTpl<Scalar> JointModelRevoluteUnaligned;
      typedef JointDataRevoluteUnalignedTpl<Scalar> JointDataRevoluteUnaligned;

      typedef JointDataRevoluteUnboundedTpl<Scalar, Options, 0> JointDataRUBX;
      typedef JointModelRevoluteUnboundedTpl<Scalar, Options, 0> JointModelRUBX;

      typedef JointDataRevoluteUnboundedTpl<Scalar, Options, 1> JointDataRUBY;
      typedef JointModelRevoluteUnboundedTpl<Scalar, Options, 1> JointModelRUBY;

      typedef JointDataRevoluteUnboundedTpl<Scalar, Options, 2> JointDataRUBZ;
      typedef JointModelRevoluteUnboundedTpl<Scalar, Options, 2> JointModelRUBZ;

      typedef JointModelRevoluteUnboundedUnalignedTpl<Scalar> JointModelRevoluteUnboundedUnaligned;
      typedef JointDataRevoluteUnboundedUnalignedTpl<Scalar> JointDataRevoluteUnboundedUnaligned;

      typedef JointModelSphericalTpl<Scalar> JointModelSpherical;
      typedef JointDataSphericalTpl<Scalar> JointDataSpherical;

      typedef JointModelSphericalZYXTpl<Scalar> JointModelSphericalZYX;
      typedef JointDataSphericalZYXTpl<Scalar> JointDataSphericalZYX;

      typedef JointDataPrismaticTpl<Scalar, Options, 0> JointDataPX;
      typedef JointModelPrismaticTpl<Scalar, Options, 0> JointModelPX;

      typedef JointDataPrismaticTpl<Scalar, Options, 1> JointDataPY;
      typedef JointModelPrismaticTpl<Scalar, Options, 1> JointModelPY;

      typedef JointDataPrismaticTpl<Scalar, Options, 2> JointDataPZ;
      typedef JointModelPrismaticTpl<Scalar, Options, 2> JointModelPZ;

      typedef JointModelPrismaticUnalignedTpl<Scalar> JointModelPrismaticUnaligned;
      typedef JointDataPrismaticUnalignedTpl<Scalar> JointDataPrismaticUnaligned;

      typedef JointDataHelicalTpl<Scalar, Options, 0> JointDataHX;
      typedef JointModelHelicalTpl<Scalar, Options, 0> JointModelHX;

      typedef JointDataHelicalTpl<Scalar, Options, 1> JointDataHY;
      typedef JointModelHelicalTpl<Scalar, Options, 1> JointModelHY;

      typedef JointDataHelicalTpl<Scalar, Options, 2> JointDataHZ;
      typedef JointModelHelicalTpl<Scalar, Options, 2> JointModelHZ;

      typedef JointModelHelicalUnalignedTpl<Scalar> JointModelHelicalUnaligned;
      typedef JointDataHelicalUnalignedTpl<Scalar> JointDataHelicalUnaligned;

      typedef JointModelFreeFlyerTpl<Scalar> JointModelFreeFlyer;
      typedef JointDataFreeFlyerTpl<Scalar> JointDataFreeFlyer;

      typedef JointModelPlanarTpl<Scalar> JointModelPlanar;
      typedef JointDataPlanarTpl<Scalar> JointDataPlanar;

      typedef JointModelUniversalTpl<Scalar> JointModelUniversal;
      typedef JointDataUniversalTpl<Scalar> JointDataUniversal;

      typedef JointModelTranslationTpl<Scalar> JointModelTranslation;
      typedef JointDataTranslationTpl<Scalar> JointDataTranslation;

      typedef JointModelCompositeTpl<Scalar> JointModelComposite;
      typedef JointDataCompositeTpl<Scalar> JointDataComposite;

      // Algorithm
      typedef ProximalSettingsTpl<Scalar> ProximalSettings;
      typedef ContactCholeskyDecompositionTpl<Scalar, Options> ContactCholeskyDecomposition;

      typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
      typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

      typedef CoulombFrictionConeTpl<context::Scalar> CoulombFrictionCone;
      typedef DualCoulombFrictionConeTpl<context::Scalar> DualCoulombFrictionCone;

      typedef DelassusOperatorDenseTpl<Scalar, Options> DelassusOperatorDense;
      typedef DelassusOperatorSparseTpl<Scalar, Options> DelassusOperatorSparse;

      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(CoulombFrictionCone)
        CoulombFrictionConeVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(DualCoulombFrictionCone)
        DualCoulombFrictionConeVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)
        RigidConstraintModelVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData)
        RigidConstraintDataVector;

// Pool
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
      typedef ModelPoolTpl<Scalar> ModelPool;

  #ifdef PINOCCHIO_WITH_HPP_FCL
      typedef GeometryPoolTpl<Scalar> GeometryPool;
  #endif

#endif

    } // namespace context
  } // namespace python
} // namespace pinocchio

#endif // #ifndef __pinocchio_python_context_generic_hpp__
