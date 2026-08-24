//
// Copyright (c) 2021-2024 INRIA
//

#pragma once

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/constraints/fwd.hpp"

#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  #include "pinocchio/multibody/pool/fwd.hpp"
  #ifdef PINOCCHIO_WITH_COLLISION
    #include "pinocchio/collision/pool/fwd.hpp"
  #endif
#endif

#include <eigenpy/eigen-typedef.hpp>

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
      static constexpr int Options = 0;

      // Eigen
      EIGENPY_MAKE_TYPEDEFS_ALL_SIZES(Scalar, Options, s);
      typedef Eigen::SparseMatrix<Scalar> SparseMatrix;
      typedef Eigen::SparseMatrix<Scalar, Eigen::RowMajor> RowSparseMatrix;
      typedef Eigen::Matrix<Scalar, 1, 1, Options, 1, 1> Matrix1s;
      typedef Eigen::Matrix<Scalar, 7, 1, Options> Vector7s;
      typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6s;
      typedef Eigen::Matrix<Scalar, 3, 6, Options> Matrix36s;
      typedef Eigen::Quaternion<Scalar, Options> Quaternion;
      typedef Eigen::AngleAxis<Scalar> AngleAxis;
      typedef Eigen::Ref<VectorXs> RefVectorXs;
      typedef Eigen::Ref<MatrixXs> RefMatrixXs;
      typedef Eigen::Ref<const VectorXs> RefConstVectorXs;
      typedef Eigen::Ref<const MatrixXs> RefConstMatrixXs;

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

      typedef JointModelEllipsoidTpl<Scalar, Options> JointModelEllipsoid;
      typedef JointDataEllipsoidTpl<Scalar, Options> JointDataEllipsoid;

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

      typedef JointModelMimicTpl<Scalar> JointModelMimic;
      typedef JointDataMimicTpl<Scalar> JointDataMimic;

      // Algorithm
      typedef ProximalSettingsTpl<Scalar> ProximalSettings;
      typedef ConstraintCholeskyDecompositionTpl<Scalar, Options> ConstraintCholeskyDecomposition;

      typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
      typedef std::vector<RigidConstraintModel> RigidConstraintModelVector;

      typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;
      typedef std::vector<RigidConstraintData> RigidConstraintDataVector;

      typedef PointAnchorConstraintModelTpl<Scalar, Options> PointAnchorConstraintModel;
      typedef std::vector<PointAnchorConstraintModel> PointAnchorConstraintModelVector;
      typedef PointAnchorConstraintDataTpl<Scalar, Options> PointAnchorConstraintData;
      typedef std::vector<PointAnchorConstraintData> PointAnchorConstraintDataVector;

      typedef FrameAnchorConstraintModelTpl<Scalar, Options> FrameAnchorConstraintModel;
      typedef std::vector<FrameAnchorConstraintModel> FrameAnchorConstraintModelVector;
      typedef FrameAnchorConstraintDataTpl<Scalar, Options> FrameAnchorConstraintData;
      typedef std::vector<FrameAnchorConstraintData> FrameAnchorConstraintDataVector;

      typedef PointContactConstraintModelTpl<Scalar, Options> PointContactConstraintModel;
      typedef std::vector<PointContactConstraintModel> PointContactConstraintModelVector;
      typedef PointContactConstraintDataTpl<Scalar, Options> PointContactConstraintData;
      typedef std::vector<PointContactConstraintData> PointContactConstraintDataVector;

      typedef ConstantLengthConstraintModelTpl<Scalar, Options> ConstantLengthConstraintModel;
      typedef std::vector<ConstantLengthConstraintModel> ConstantLengthConstraintModelVector;
      typedef ConstantLengthConstraintDataTpl<Scalar, Options> ConstantLengthConstraintData;
      typedef std::vector<ConstantLengthConstraintData> ConstantLengthConstraintDataVector;

      typedef JointFrictionConstraintModelTpl<Scalar, Options> JointFrictionConstraintModel;
      typedef std::vector<JointFrictionConstraintModel> JointFrictionConstraintModelVector;
      typedef JointFrictionConstraintDataTpl<Scalar, Options> JointFrictionConstraintData;
      typedef std::vector<JointFrictionConstraintData> JointFrictionConstraintDataVector;

      typedef JointLimitConstraintModelTpl<Scalar, Options> JointLimitConstraintModel;
      typedef std::vector<JointLimitConstraintModel> JointLimitModelVector;
      typedef JointLimitConstraintDataTpl<Scalar, Options> JointLimitConstraintData;
      typedef std::vector<JointLimitConstraintData> JointLimitDataVector;

      typedef ConstraintModelTpl<Scalar, Options> ConstraintModel;
      typedef std::vector<ConstraintModel> ConstraintModelVector;
      typedef ConstraintDataTpl<Scalar, Options> ConstraintData;
      typedef std::vector<ConstraintData> ConstraintDataVector;

      typedef CoulombFrictionConeTpl<context::Scalar> CoulombFrictionCone;
      typedef std::vector<CoulombFrictionCone> CoulombFrictionConeVector;
      typedef DualCoulombFrictionConeTpl<context::Scalar> DualCoulombFrictionCone;
      typedef std::vector<DualCoulombFrictionCone> DualCoulombFrictionConeVector;
      typedef BoxSetTpl<context::Scalar> BoxSet;
      typedef ZeroConeTpl<context::Scalar> ZeroCone;
      typedef FullSpaceConeTpl<context::Scalar> FullSpaceCone;
      typedef NonNegativeOrthantConeTpl<context::Scalar> NonNegativeOrthantCone;

      typedef ConstraintCollectionDefaultTpl<Scalar, Options> ConstraintCollectionDefault;

      typedef DelassusOperatorDenseTpl<Scalar, Options> DelassusOperatorDense;
      typedef DelassusOperatorSparseTpl<Scalar, Options> DelassusOperatorSparse;

// Pool
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
      typedef ModelPoolTpl<Scalar> ModelPool;

  #ifdef PINOCCHIO_WITH_COLLISION
      typedef GeometryPoolTpl<Scalar> GeometryPool;
  #endif

#endif

    } // namespace context
  } // namespace python
} // namespace pinocchio
