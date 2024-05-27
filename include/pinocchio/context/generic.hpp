//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_context_generic_hpp__
#define __pinocchio_context_generic_hpp__

#include <Eigen/Core>
#include "pinocchio/container/aligned-vector.hpp"

namespace pinocchio
{

  template<typename _Scalar, int _Options>
  struct JointCollectionDefaultTpl;
  template<
    typename _Scalar,
    int _Options = 0,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct ModelTpl;
  template<
    typename _Scalar,
    int _Options = 0,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct DataTpl;
  template<typename _Scalar, int _Options = 0>
  class MotionTpl;
  template<typename _Scalar, int _Options = 0>
  class ForceTpl;
  template<typename _Scalar, int _Options>
  struct RigidConstraintModelTpl;
  template<typename _Scalar, int _Options>
  struct RigidConstraintDataTpl;

  template<typename _Scalar>
  struct CoulombFrictionConeTpl;
  template<typename _Scalar>
  struct DualCoulombFrictionConeTpl;

  namespace context
  {
    typedef PINOCCHIO_SCALAR_TYPE Scalar;
    enum
    {
      Options = 0
    };
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> Matrix6xs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Options>
      RowMatrixXs;
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic, Options> Matrix3x;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 10, Options> BodyRegressorType;

    typedef ModelTpl<Scalar, Options> Model;
    typedef DataTpl<Scalar, Options> Data;

    typedef CoulombFrictionConeTpl<Scalar> CoulombFrictionCone;
    typedef DualCoulombFrictionConeTpl<Scalar> DualCoulombFrictionCone;

    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(CoulombFrictionCone)
      CoulombFrictionConeVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(DualCoulombFrictionCone)
      DualCoulombFrictionConeVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)
      RigidConstraintModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData)
      RigidConstraintDataVector;

    typedef MotionTpl<Scalar, Options> Motion;
    typedef ForceTpl<Scalar, Options> Force;

  } // namespace context

  // Read and write
  template<typename Derived>
  Eigen::Ref<typename Derived::PlainObject> make_ref(const Eigen::MatrixBase<Derived> & x)
  {
    return Eigen::Ref<typename Derived::PlainObject>(x.const_cast_derived());
  }

  // Read-only
  template<typename M>
  auto make_const_ref(Eigen::MatrixBase<M> const & m) -> Eigen::Ref<typename M::PlainObject const>
  {
    return m;
  }

} // namespace pinocchio

#endif // #ifndef __pinocchio_context_generic_hpp__
