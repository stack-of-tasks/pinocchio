#ifndef __pinocchio_python_context_generic_hpp__
#define __pinocchio_python_context_generic_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/algorithm/fwd.hpp"

#include <eigenpy/eigen-typedef.hpp>

namespace pinocchio {
namespace python {

namespace context {

typedef PINOCCHIO_PYTHON_SCALAR_TYPE Scalar;
enum { Options = 0 };

// Eigen
EIGENPY_MAKE_TYPEDEFS_ALL_SIZES(Scalar,Options,s);
typedef Eigen::Matrix<Scalar,1,1,Options,1,1> Matrix1s;
typedef Eigen::Quaternion<Scalar,Options> Quaternion;
typedef Eigen::AngleAxis<Scalar> AngleAxis;

// Spatial
typedef SE3Tpl<Scalar,Options> SE3;
typedef MotionTpl<Scalar,Options> Motion;
typedef ForceTpl<Scalar,Options> Force;
typedef InertiaTpl<Scalar,Options> Inertia;

// Multibody
typedef FrameTpl<Scalar,Options> Frame;
typedef ModelTpl<Scalar,Options> Model;
typedef DataTpl<Scalar,Options> Data;

typedef JointCollectionDefaultTpl<Scalar,Options> JointCollectionDefault;

typedef JointModelTpl<Scalar,Options> JointModel;
typedef JointDataTpl<Scalar,Options> JointData;

// Algorithm
typedef ProximalSettingsTpl<Scalar> ProximalSettings;
typedef cholesky::ContactCholeskyDecompositionTpl<Scalar,Options> ContactCholeskyDecomposition;

typedef RigidConstraintModelTpl<Scalar,Options> RigidConstraintModel;
typedef RigidConstraintDataTpl<Scalar,Options> RigidConstraintData;

}}}

#endif // #ifndef __pinocchio_python_context_generic_hpp__
