// Copyright (c) 2026 INRIA

// Port of the following files:
// - include/pinocchio/bindings/python/algorithm/contact-info.hpp
// - include/pinocchio/bindings/python/algorithm/constraint-cholesky.hpp
// - bindings/python/algorithm/expose-constrained-dynamics.cpp

#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/bindings/python-nb/utils/copyable.hpp"
#include "pinocchio/bindings/python-nb/algorithm/delassus-operator.hpp"
#include "pinocchio/bindings/python-nb/utils/deprecation.hpp"

#include "pinocchio/constraints.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/delassus-operator.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/eigen/sparse.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using pinocchio::ContactType;

static void exposeContactType(nb::module_ m)
{
  nb::enum_<ContactType>(m, "ContactType")
    .value("CONTACT_3D", CONTACT_3D)
    .value("CONTACT_6D", CONTACT_6D)
    .value("CONTACT_UNDEFINED", CONTACT_UNDEFINED);
}

static void exposeProximalSettings(nb::module_ m)
{
  nb::class_<ProximalSettings>(
    m, "ProximalSettings",
    "Structure containing all the settings parameters for proximal algorithms.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<Scalar, Scalar, int>(), "accuracy"_a, "mu"_a, "max_iter"_a,
      "Structure containing all the settings parameters for the proximal algorithms.")
    .def(
      nb::init<Scalar, Scalar, Scalar, int>(), "absolute_accuracy"_a, "relative_accuracy"_a, "mu"_a,
      "max_iter"_a, "Structure containing all the settings parameters for the proximal algorithms.")
    .def_rw(
      "absolute_accuracy", &ProximalSettings::absolute_accuracy, "Absolute proximal accuracy.")
    .def_rw(
      "relative_accuracy", &ProximalSettings::relative_accuracy,
      "Relative proximal accuracy between two iterates.")
    .def_rw("mu", &ProximalSettings::mu, "Regularization parameter of the Proximal algorithms.")
    .def_rw("max_iter", &ProximalSettings::max_iter, "Maximal number of iterations.")
    .def_rw("absolute_residual", &ProximalSettings::absolute_residual, "Absolute residual.")
    .def_rw(
      "relative_residual", &ProximalSettings::relative_residual,
      "Relative residual between two iterates.")
    .def_rw(
      "iter", &ProximalSettings::iter,
      "Final number of iteration of the algorithm when it has converged or "
      "reached the maximal number of allowed iterations.")
    .def("__repr__", [](const ProximalSettings & self) {
      std::ostringstream ss;
      ss << "ProximalSettings(" << self.absolute_accuracy << ", " << self.relative_accuracy << ", "
         << self.mu << ", " << self.max_iter << ")";
      return ss.str();
    });
}

static void exposeConstraintCholesky(nb::module_ m)
{
  using Self = ConstraintCholeskyDecomposition;
  using Matrix = MatrixXs;
  using RowMatrix = RowMatrixXs;
  using DelassusCholesky = typename Self::DelassusOperatorCholeskyExpression;
  using DelassusOperatorDense = DelassusOperatorDenseTpl<Scalar, Options>;
  using DelassusOperatorSparse = DelassusOperatorSparseTpl<Scalar, Options>;

  static constexpr char kGetDelassusCholeskyMsg[] =
    "Deprecated member. Use getDelassusOperatorCholeskyExpression instead.";

  // --- ConstraintCholeskyDecomposition ---
  nb::class_<Self>(
    m, "ConstraintCholeskyDecomposition",
    "Contact information container for contact dynamic algorithms.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<const Model &, const Data &>(), "model"_a, "data"_a,
      "Constructor from a model and data.")
    .def(
      nb::init<
        const Model &, const Data &, const RigidConstraintModelVector &,
        const RigidConstraintDataVector &>(),
      "model"_a, "data"_a, "contact_models"_a, "contact_datas"_a,
      "Constructor from a model, data and a collection of RigidConstraintModels.")
    .def(
      nb::init<
        const Model &, const Data &, const ConstraintModelVector &, const ConstraintDataVector &>(),
      "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a,
      "Constructor from a model, data and a collection of ConstraintModels.")

    // Matrix views (reference into internal storage)
    .def_prop_ro(
      "U", [](const Self & self) -> Eigen::Ref<const RowMatrix> { return self.U; },
      nb::rv_policy::reference_internal, "Upper triangular factor U of the decomposition.")
    .def_prop_ro(
      "D", [](const Self & self) -> Eigen::Ref<const VectorXs> { return self.D; },
      nb::rv_policy::reference_internal, "Diagonal factor D of the decomposition.")
    .def_prop_ro(
      "Dinv", [](const Self & self) -> Eigen::Ref<const VectorXs> { return self.Dinv; },
      nb::rv_policy::reference_internal, "Inverse of the diagonal factor D.")

    // Size queries
    .def("size", &Self::size, "Size of the decomposition.")
    .def(
      "constraintDim", &Self::constraintDim,
      "Total dimension of the constraints in the Cholesky factorization.")

    // Rebuild
    .def(
      "rebuild",
      [](
        Self & self, const Model & model, const Data & data,
        const RigidConstraintModelVector & contact_models,
        const RigidConstraintDataVector & contact_datas) {
        self.rebuild(model, data, contact_models, contact_datas);
      },
      "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a,
      "Rebuild the Cholesky decomposition internal memory from a list of RigidConstraintModels.")
    .def(
      "rebuild",
      [](
        Self & self, const Model & model, const Data & data,
        const ConstraintModelVector & constraint_models,
        const ConstraintDataVector & constraint_datas) {
        self.rebuild(model, data, constraint_models, constraint_datas);
      },
      "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a,
      "Rebuild the Cholesky decomposition internal memory from a list of ConstraintModels.")

    // Compute (scalar mu)
    .def(
      "compute",
      [](
        Self & self, const Model & model, Data & data,
        const RigidConstraintModelVector & contact_models,
        RigidConstraintDataVector & contact_datas,
        Scalar mu) { self.compute(model, data, contact_models, contact_datas, mu); },
      "model"_a, "data"_a, "contact_models"_a, "contact_datas"_a, "mu"_a = Scalar(0),
      "Computes the Cholesky decomposition of the augmented KKT matrix for the given "
      " RigidConstraintModels, regularized by scalar mu.")
    .def(
      "compute",
      [](
        Self & self, const Model & model, Data & data,
        const ConstraintModelVector & constraint_models, ConstraintDataVector & constraint_datas,
        Scalar mu) { self.compute(model, data, constraint_models, constraint_datas, mu); },
      "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a, "mu"_a = Scalar(0),
      "Computes the Cholesky decomposition of the augmented KKT matrix for the given "
      "ConstraintModels, regularized by scalar mu.")

    // Compute (vector mus)
    .def(
      "compute",
      [](
        Self & self, const Model & model, Data & data,
        const RigidConstraintModelVector & contact_models,
        RigidConstraintDataVector & contact_datas,
        const VectorXs & mus) { self.compute(model, data, contact_models, contact_datas, mus); },
      "model"_a, "data"_a, "contact_models"_a, "contact_datas"_a, "mus"_a,
      "Computes the Cholesky decomposition of the KKT matrix with RigidConstraints, "
      "regularized by per-constraint vector mus.")
    .def(
      "compute",
      [](
        Self & self, const Model & model, Data & data,
        const ConstraintModelVector & constraint_models, ConstraintDataVector & constraint_datas,
        const VectorXs & mus) {
        self.compute(model, data, constraint_models, constraint_datas, mus);
      },
      "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a, "mus"_a,
      "Computes the Cholesky decomposition of the KKT matrix with ConstraintModels, "
      "regularized by per-constraint vector mus.")

    // Damping
    .def(
      "updateDamping", [](Self & self, Scalar mu) { self.updateDamping(mu); }, "mu"_a,
      "Update the damping term on the upper-left block of the KKT matrix. Must be positive.")
    .def(
      "updateDamping",
      [](Self & self, const VectorXs & mus) { self.template updateDamping<VectorXs>(mus); },
      "mus"_a,
      "Update the per-constraint damping terms on the upper-left block of the KKT matrix. "
      "All must be positive.")
    .def(
      "getDamping", [](const Self & self) -> Matrix { return self.getDamping().matrix(); },
      "Returns the current damping as a dense matrix.")

    // Matrix / solve / inverse
    .def(
      "matrix", [](const Self & self) -> Matrix { return self.matrix(); },
      "Returns the dense matrix resulting from the decomposition.")
    .def(
      "solve", [](const Self & self, const Matrix & mat) -> Matrix { return self.solve(mat); },
      "mat"_a, "Computes the solution x of A * x = mat where A is the decomposed matrix.")
    .def(
      "inverse", [](const Self & self) -> Matrix { return self.inverse(); },
      "Returns the inverse matrix resulting from the decomposition.")

    // Derived matrix accessors
    .def(
      "getInverseOperationalSpaceInertiaMatrix",
      [](const Self & self, bool enforce_symmetry) -> Matrix {
        return self.getInverseOperationalSpaceInertiaMatrix(enforce_symmetry);
      },
      "enforce_symmetry"_a = false, "Returns the inverse of the Operational Space Inertia Matrix.")
    .def(
      "getOperationalSpaceInertiaMatrix",
      [](const Self & self) -> Matrix { return self.getOperationalSpaceInertiaMatrix(); },
      "Returns the Operational Space Inertia Matrix.")
    .def(
      "getInverseMassMatrix",
      [](const Self & self) -> Matrix { return self.getInverseMassMatrix(); },
      "Returns the inverse of the Joint Space Inertia Matrix (mass matrix).")
    .def(
      "getMassMatrixChoeslkyDecomposition",
      &Self::template getMassMatrixChoeslkyDecomposition<Scalar, 0, JointCollectionDefaultTpl>,
      "Retrieves the Cholesky decomposition of the Mass Matrix.")

    // Delassus operator accessors
    .def(
      "getDelassusOperatorCholeskyExpression",
      [](Self & self) -> DelassusCholesky { return self.getDelassusOperatorCholeskyExpression(); },
      nb::rv_policy::reference_internal,
      "Returns the Cholesky expression associated to the underlying Delassus matrix.")
    .def(
      "getDelassusCholeskyExpression",
      [](Self & self) -> DelassusCholesky {
        deprecated_guard<kGetDelassusCholeskyMsg> guard;
        (void)guard;
        return self.getDelassusOperatorCholeskyExpression();
      },
      nb::rv_policy::reference_internal,
      "Deprecated. Use getDelassusOperatorCholeskyExpression instead.")
    .def(ComparableVisitor<Self>())
    .def(CopyableVisitor<Self>());

  // Alias
  m.attr("ContactCholeskyDecomposition") = m.attr("ConstraintCholeskyDecomposition");

  // --- DelassusOperatorCholeskyExpression ---
  nb::class_<DelassusCholesky>(
    m, "DelassusOperatorCholeskyExpression",
    "Delassus Cholesky expression associated to a given ConstraintCholeskyDecomposition.")
    .def(
      nb::init<Self &>(), "cholesky_decomposition"_a,
      "Build from a given ConstraintCholeskyDecomposition.")
    .def(
      "cholesky", [](DelassusCholesky & self) -> Self & { return self.cholesky(); },
      nb::rv_policy::reference_internal,
      "Returns the ConstraintCholeskyDecomposition associated to this expression.")
    .def(DelassusOperatorBaseVisitor<DelassusCholesky>());

  // Alias
  m.attr("DelassusCholeskyExpression") = m.attr("DelassusOperatorCholeskyExpression");

  // --- DelassusOperatorDense ---
  nb::class_<DelassusOperatorDense>(
    m, "DelassusOperatorDense", "Delassus dense operator built from a dense matrix.")
    .def(
      nb::init<const Eigen::Ref<const MatrixXs>>(), "matrix"_a, "Build from a given dense matrix.")
    .def(DelassusOperatorBaseVisitor<DelassusOperatorDense>());

  // --- DelassusOperatorSparse ---
  {
    using SparseMatrix = typename DelassusOperatorSparse::SparseMatrix;
    nb::class_<DelassusOperatorSparse>(
      m, "DelassusOperatorSparse", "Delassus sparse operator built from a sparse matrix.")
      .def(nb::init<const SparseMatrix &>(), "matrix"_a, "Build from a given sparse matrix.")
      .def(DelassusOperatorBaseVisitor<DelassusOperatorSparse>())
      .def(
        "matrix",
        [](const DelassusOperatorSparse & self, bool enforce_symmetry) -> SparseMatrix {
          return self.matrix(enforce_symmetry);
        },
        "enforce_symmetry"_a = false, "Returns the Delassus expression as a sparse matrix.");
  }

#ifdef PINOCCHIO_WITH_ACCELERATE_SUPPORT
  {
    using SparseMatrix = typename DelassusOperatorSparse::SparseMatrix;
    using AccelerateCholesky = Eigen::AccelerateLLT<SparseMatrix>;
    using DelassusOperatorSparseAccelerate =
      DelassusOperatorSparseTpl<Scalar, Options, AccelerateCholesky>;
    nb::class_<DelassusOperatorSparseAccelerate>(
      m, "DelassusOperatorSparseAccelerate",
      "Delassus sparse operator leveraging the Accelerate framework on Apple systems.")
      .def(nb::init<const SparseMatrix &>(), "matrix"_a, "Build from a given sparse matrix.")
      .def(DelassusOperatorBaseVisitor<DelassusOperatorSparseAccelerate>());
  }
#endif
}

static void exposeRigidConstraint(nb::module_ m)
{
  nb::class_<RigidConstraintModel>(
    m, "RigidConstraintModel", "Rigid contact model for contact dynamic algorithms.")
    .def(
      nb::init<
        ContactType, const Model &, JointIndex, const SE3 &, JointIndex, const SE3 &,
        ReferenceFrame>(),
      "contact_type"_a, "model"_a, "joint1_id"_a, "joint1_placement"_a, "joint2_id"_a,
      "joint2_placement"_a, "reference_frame"_a = LOCAL,
      "Constructor from a given ContactType, joint index and placement for the two joints "
      "implied in the constraint.")
    .def(
      nb::init<ContactType, const Model &, JointIndex, const SE3 &, ReferenceFrame>(),
      "contact_type"_a, "model"_a, "joint1_id"_a, "joint1_placement"_a, "reference_frame"_a = LOCAL,
      "Constructor from a given ContactType, joint index and placement only for the first "
      "joint implied in the constraint.")
    .def(
      nb::init<ContactType, const Model &, JointIndex, ReferenceFrame>(), "contact_type"_a,
      "model"_a, "joint1_id"_a, "reference_frame"_a = LOCAL,
      "Constructor from a given ContactType and joint index. The base joint is taken as 0 "
      "in the constraint.")
    .def_rw("name", &RigidConstraintModel::name, "Name of the contact.")
    .def_rw("type", &RigidConstraintModel::type, "Type of the contact.")
    .def_rw(
      "joint1_id", &RigidConstraintModel::joint1_id,
      "Index of first parent joint in the model tree.")
    .def_rw(
      "joint2_id", &RigidConstraintModel::joint2_id,
      "Index of second parent joint in the model tree.")
    .def_rw(
      "joint1_placement", &RigidConstraintModel::joint1_placement,
      "Relative placement with respect to the frame of joint1.")
    .def_rw(
      "joint2_placement", &RigidConstraintModel::joint2_placement,
      "Relative placement with respect to the frame of joint2.")
    .def_rw(
      "reference_frame", &RigidConstraintModel::reference_frame,
      "Reference frame where the constraint is expressed (WORLD, LOCAL_WORLD_ALIGNED or LOCAL).")
    .def_rw(
      "desired_contact_placement", &RigidConstraintModel::desired_contact_placement,
      "Desired contact placement.")
    .def_rw(
      "desired_contact_velocity", &RigidConstraintModel::desired_contact_velocity,
      "Desired contact spatial velocity.")
    .def_rw(
      "desired_contact_acceleration", &RigidConstraintModel::desired_contact_acceleration,
      "Desired contact spatial acceleration.")
    .def(
      "createData", [](const RigidConstraintModel & self) { return RigidConstraintData(self); },
      "Create a Data object for the given model.")
    .def(
      "calc",
      [](
        const RigidConstraintModel & self, const Model & model, const Data & data,
        RigidConstraintData & cdata) { self.calc(model, data, cdata); },
      "model"_a, "data"_a, "constraint_data"_a)
    .def(
      "jacobian",
      [](
        const RigidConstraintModel & self, const Model & model, const Data & data,
        RigidConstraintData & cdata) {
        MatrixXs res(self.residualSize(), model.nv);
        self.jacobian(model, data, cdata, res);
        return res;
      },
      "model"_a, "data"_a, "constraint_data"_a)
    .def(
      "residualSize",
      [](const RigidConstraintModel & self, ConstraintSelectionType sel) {
        switch (sel)
        {
        case ConstraintSelectionType::CURRENT:
          return self.residualSize(CurrentSelection());
        case ConstraintSelectionType::MAXIMAL:
          return self.residualSize(MaximalSelection());
        default:
          PINOCCHIO_UNREACHABLE();
        }
      },
      "sel"_a = ConstraintSelectionType::CURRENT, "Constraint size for the selection.")
    .def(
      "setCompliance",
      [](RigidConstraintModel & self, ConstVectorRef vec, ConstraintSelectionType sel) {
        switch (sel)
        {
        case ConstraintSelectionType::CURRENT:
          return self.setCompliance(vec, CurrentSelection());
        case ConstraintSelectionType::MAXIMAL:
          return self.setCompliance(vec, MaximalSelection());
        }
      },
      "vec"_a, "sel"_a = ConstraintSelectionType::CURRENT,
      "Set the compliance value for the selected constraint.")
    .def(
      "retrieveCompliance",
      [](const RigidConstraintModel & self, ConstraintSelectionType sel) -> VectorXs {
        switch (sel)
        {
        case ConstraintSelectionType::CURRENT: {
          VectorXs res(self.residualSize(CurrentSelection()));
          self.retrieveCompliance(res, CurrentSelection());
          return res;
        }
        case ConstraintSelectionType::MAXIMAL: {
          VectorXs res(self.residualSize(MaximalSelection()));
          self.retrieveCompliance(res, MaximalSelection());
          return res;
        }
        default:
          PINOCCHIO_UNREACHABLE();
        }
      },
      "sel"_a = ConstraintSelectionType::CURRENT,
      "Retrieve the compliance value for the selected constraint.")
    .def(
      "setBaumgarteCorrectorParameters",
      [](
        RigidConstraintModel & self, const BaumgarteCorrectorParameters & params,
        ConstraintSelectionType sel) {
        switch (sel)
        {
        case ConstraintSelectionType::CURRENT:
          return self.setBaumgarteCorrectorParameters(params, CurrentSelection());
        case ConstraintSelectionType::MAXIMAL:
          return self.setBaumgarteCorrectorParameters(params, MaximalSelection());
        }
      },
      "params"_a, "sel"_a = ConstraintSelectionType::CURRENT, "Set the Baumgarte parameters.")
    .def_prop_rw(
      "m_baumgarte_parameters",
      [](RigidConstraintModel & self) -> BaumgarteCorrectorParameters & {
        return self.baumgarte_corrector_parameters();
      },
      [](RigidConstraintModel & self, const BaumgarteCorrectorParameters & copy) {
        self.baumgarte_corrector_parameters() = copy;
      },
      "Baumgarte parameters associated with the constraint.")
    .def(ComparableVisitor<RigidConstraintModel>());

  nb::bind_vector<RigidConstraintModelVector, nb::rv_policy::reference_internal>(
    m, "StdVec_RigidConstraintModel");

  nb::class_<RigidConstraintData>(
    m, "RigidConstraintData",
    "Rigid constraint data associated to a RigidConstraintModel for contact dynamic algorithms.")
    .def(nb::init<const RigidConstraintModel &>(), "contact_model"_a, "Default constructor.")
    .def_rw("contact_force", &RigidConstraintData::contact_force, "Constraint force.")
    .def_rw(
      "oMc1", &RigidConstraintData::oMc1,
      "Placement of the constraint frame 1 with respect to the WORLD frame.")
    .def_rw(
      "oMc2", &RigidConstraintData::oMc2,
      "Placement of the constraint frame 2 with respect to the WORLD frame.")
    .def_rw("c1Mc2", &RigidConstraintData::c1Mc2, "Relative displacement between the two frames.")
    .def_rw(
      "contact_placement_error", &RigidConstraintData::contact_placement_error,
      "Current contact placement error between the two contact Frames.\n"
      "This corresponds to the relative placement between the two contact Frames seen as a "
      "Motion error.")
    .def_rw(
      "contact1_velocity", &RigidConstraintData::contact1_velocity,
      "Current contact Spatial velocity of the constraint 1.")
    .def_rw(
      "contact2_velocity", &RigidConstraintData::contact2_velocity,
      "Current contact Spatial velocity of the constraint 2.")
    .def_rw(
      "contact_velocity_error", &RigidConstraintData::contact_velocity_error,
      "Current contact Spatial velocity error between the two contact Frames.\n"
      "This corresponds to the relative velocity between the two contact Frames.")
    .def_rw(
      "contact_acceleration", &RigidConstraintData::contact_acceleration,
      "Current contact Spatial acceleration.")
    .def_rw(
      "contact_acceleration_desired", &RigidConstraintData::contact_acceleration_desired,
      "Desired contact acceleration.")
    .def_rw(
      "contact_acceleration_error", &RigidConstraintData::contact_acceleration_error,
      "Current contact spatial error (due to the integration step).")
    .def_rw(
      "contact1_acceleration_drift", &RigidConstraintData::contact1_acceleration_drift,
      "Current contact drift acceleration (acceleration only due to the Coriolis and "
      "centrifugal effects) of the contact frame 1.")
    .def_rw(
      "contact2_acceleration_drift", &RigidConstraintData::contact2_acceleration_drift,
      "Current contact drift acceleration (acceleration only due to the Coriolis and "
      "centrifugal effects) of the contact frame 2.")
    .def_rw(
      "contact_acceleration_deviation", &RigidConstraintData::contact_acceleration_deviation,
      "Contact deviation from the reference acceleration (a.k.a the error).")
    .def_rw(
      "extended_motion_propagators_joint1",
      &RigidConstraintData::extended_motion_propagators_joint1,
      "Extended force/motion propagators for joint 1.")
    .def_rw(
      "lambdas_joint1", &RigidConstraintData::lambdas_joint1,
      "Extended force/motion propagators for joint 1.")
    .def_rw(
      "extended_motion_propagators_joint2",
      &RigidConstraintData::extended_motion_propagators_joint2,
      "Extended force/motion propagators for joint 2.")
    .def(ComparableVisitor<RigidConstraintData>());

  nb::bind_vector<RigidConstraintDataVector, nb::rv_policy::reference_internal>(
    m, "StdVec_RigidConstraintData");

  using VectorOfMatrix6 = RigidConstraintData::VectorOfMatrix6;
  nb::bind_vector<VectorOfMatrix6, nb::rv_policy::reference_internal>(m, "StdVec_Matrix6_");

  m.def(
    "convertToRigidConstraintModel",
    [](
      const Model & model, const PointAnchorConstraintModel & constraint,
      ReferenceFrame reference_frame) {
      return convertToRigidConstraintModel(model, constraint, reference_frame);
    },
    "model"_a, "constraint"_a, "reference_frame"_a = ReferenceFrame::LOCAL,
    "Convert a PointAnchorConstraintModel to a RigidConstraintModel with contact type "
    "CONTACT_3D.\n\n"
    "The kinematic structure (joint IDs and placements) is preserved. The 3D desired "
    "offset is placed in the translation of desired_contact_placement, and the 3D desired "
    "velocity/acceleration are placed in the linear parts of the corresponding Motion fields.");

  m.def(
    "convertToRigidConstraintModel",
    [](
      const Model & model, const FrameAnchorConstraintModel & constraint,
      ReferenceFrame reference_frame) {
      return convertToRigidConstraintModel(model, constraint, reference_frame);
    },
    "model"_a, "constraint"_a, "reference_frame"_a = ReferenceFrame::LOCAL,
    "Convert a FrameAnchorConstraintModel to a RigidConstraintModel with contact type "
    "CONTACT_6D.\n\n"
    "The kinematic structure (joint IDs and placements) is preserved. The 6D desired "
    "offset (ordered [linear; angular]) is mapped to desired_contact_placement via exp6, "
    "and the 6D desired velocity/acceleration vectors are mapped directly to Motion fields.");
}

void exposeConstraintDynamics(nb::module_ m)
{
  exposeContactType(m);
  exposeProximalSettings(m);
  exposeRigidConstraint(m);
  exposeConstraintCholesky(m);

  m.def(
    "initConstraintDynamics",
    [](
      const Model & model, Data & data, const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas) {
      initConstraintDynamics(model, data, contact_models, contact_datas);
    },
    "model"_a, "data"_a, "contact_models"_a, "contact_datas"_a,
    "Allocate the memory before running contact dynamics algorithms.\n"
    "This avoids online memory allocation when running these algorithms.");

  m.def(
    "constraintDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      const RigidConstraintModelVector & contact_models, RigidConstraintDataVector & contact_datas,
      ProximalSettings & prox_settings) -> const Data::TangentVectorType & {
      return constraintDynamics(
        model, data, q, v, tau, contact_models, contact_datas, prox_settings);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "contact_models"_a, "contact_datas"_a,
    "prox_settings"_a,
    "Computes the forward dynamics with contact constraints according to a given list of "
    "contact information.\n"
    "When using constraintDynamics for the first time, you should call first "
    "initConstraintDynamics to initialize the internal memory used in the algorithm.\n"
    "This function returns joint acceleration of the system. The contact forces are "
    "stored in data.contact_forces.",
    nb::rv_policy::reference);

  m.def(
    "constraintDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas) -> const Data::TangentVectorType & {
      return constraintDynamics(model, data, q, v, tau, contact_models, contact_datas);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "contact_models"_a, "contact_datas"_a,
    "Computes the forward dynamics with contact constraints according to a given list of "
    "contact information.\n"
    "When using constraintDynamics for the first time, you should call first "
    "initConstraintDynamics to initialize the internal memory used in the algorithm.\n"
    "This function returns joint acceleration of the system. The contact forces are "
    "stored in data.contact_forces.",
    nb::rv_policy::reference);
}

PINOCCHIO_PYTHON_NAMESPACE_END
