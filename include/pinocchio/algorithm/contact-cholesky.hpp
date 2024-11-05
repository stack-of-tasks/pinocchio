//
// Copyright (c) 2019-2024 INRIA
//

#ifndef __pinocchio_algorithm_contact_cholesky_hpp__
#define __pinocchio_algorithm_contact_cholesky_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/math/matrix-block.hpp"
#include "pinocchio/math/triangular-matrix.hpp"

#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/delassus-operator-base.hpp"

namespace pinocchio
{

  // Forward declaration of algo
  namespace details
  {
    template<typename MatrixLike, int ColsAtCompileTime = MatrixLike::ColsAtCompileTime>
    struct UvAlgo;

    template<typename MatrixLike, int ColsAtCompileTime = MatrixLike::ColsAtCompileTime>
    struct UtvAlgo;

    template<typename MatrixLike, int ColsAtCompileTime = MatrixLike::ColsAtCompileTime>
    struct UivAlgo;

    template<typename MatrixLike, int ColsAtCompileTime = MatrixLike::ColsAtCompileTime>
    struct UtivAlgo;

    template<typename Scalar, int Options, typename VectorLike>
    VectorLike & inverseAlgo(
      const ContactCholeskyDecompositionTpl<Scalar, Options> & chol,
      const Eigen::DenseIndex col,
      const Eigen::MatrixBase<VectorLike> & vec);
  } // namespace details

  template<typename _ContactCholeskyDecomposition>
  struct DelassusCholeskyExpressionTpl;

  ///
  ///  \brief Contact Cholesky decomposition structure. This structure allows
  ///        to compute in a efficient and parsimonious way the Cholesky decomposition
  ///        of the KKT matrix related to the contact dynamics.
  ///        Such a decomposition is usefull when computing both the forward dynamics in contact
  ///        or the related analytical derivatives.
  ///
  ///
  /// \tparam _Scalar Scalar type.
  ///  \tparam _Options Alignment Options of the Eigen objects contained in the data structure.
  ///
  template<typename _Scalar, int _Options>
  struct PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
    ContactCholeskyDecompositionTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pinocchio::Index Index;
    typedef _Scalar Scalar;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3,
      Options = _Options
    };

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> Vector;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> Matrix;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Matrix) RowMatrix;
    // TODO Remove when API is stabilized
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP
    typedef Eigen::Matrix<Eigen::DenseIndex, Eigen::Dynamic, 1, Options> IndexVector;
    typedef typename PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(IndexVector) VectorOfIndexVector;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1, Options> BooleanVector;

    ///@{
    /// \brief Data information related to the Sparsity structure of the Cholesky decompostion
    struct Slice
    {
      Slice(const Eigen::DenseIndex & first_index, const Eigen::DenseIndex & size)
      : first_index(first_index)
      , size(size)
      {
      }

      Eigen::DenseIndex first_index;
      Eigen::DenseIndex size;
    };

    typedef DelassusCholeskyExpressionTpl<ContactCholeskyDecompositionTpl>
      DelassusCholeskyExpression;
    friend struct DelassusCholeskyExpressionTpl<ContactCholeskyDecompositionTpl>;

    typedef std::vector<Slice> SliceVector;
    typedef std::vector<SliceVector> VectorOfSliceVector;
    ///@}

    ///
    /// \brief Default constructor
    ///
    ContactCholeskyDecompositionTpl() = default;

    ///
    /// \brief Constructor from a model.
    ///
    /// \param[in] model Model of the kinematic tree.
    ///
    template<typename S1, int O1, template<typename, int> class JointCollectionTpl>
    explicit ContactCholeskyDecompositionTpl(const ModelTpl<S1, O1, JointCollectionTpl> & model)
    {
      // TODO Remove when API is stabilized
      PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
      PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) empty_contact_models;
      PINOCCHIO_COMPILER_DIAGNOSTIC_POP
      allocate(model, empty_contact_models);
    }

    ///
    /// \brief Constructor from a model and a collection of RigidConstraintModel objects.
    ///
    /// \param[in] model Model of the kinematic tree
    /// \param[in] contact_models Vector of RigidConstraintModel objects containing the contact
    /// information
    ///
    // TODO Remove when API is stabilized
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<typename S1, int O1, template<typename, int> class JointCollectionTpl, class Allocator>
    ContactCholeskyDecompositionTpl(
      const ModelTpl<S1, O1, JointCollectionTpl> & model,
      const std::vector<RigidConstraintModelTpl<S1, O1>, Allocator> & contact_models)
    {
      allocate(model, contact_models);
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy constructor
    ContactCholeskyDecompositionTpl(const ContactCholeskyDecompositionTpl & copy) = default;

    ///
    ///  \brief Memory allocation of the vectors D, Dinv, and the upper triangular matrix U.
    ///
    /// \param[in] model Model of the kinematic tree
    /// \param[in] contact_models Vector of RigidConstraintModel objects containing the contact
    /// information
    ///
    // TODO Remove when API is stabilized
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<typename S1, int O1, template<typename, int> class JointCollectionTpl, class Allocator>
    void allocate(
      const ModelTpl<S1, O1, JointCollectionTpl> & model,
      const std::vector<RigidConstraintModelTpl<S1, O1>, Allocator> & contact_models);
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Returns the Inverse of the Operational Space Inertia Matrix resulting from the
    /// decomposition.
    ///
    Matrix getInverseOperationalSpaceInertiaMatrix() const
    {
      Matrix res(constraintDim(), constraintDim());
      getInverseOperationalSpaceInertiaMatrix(res);
      return res;
    }

    template<typename MatrixType>
    void getInverseOperationalSpaceInertiaMatrix(const Eigen::MatrixBase<MatrixType> & res) const
    {
      typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType
        ConstBlockXpr;
      //        typedef typename RowMatrix::ConstBlockXpr ConstBlockXpr;
      const ConstBlockXpr U1 = U.topLeftCorner(constraintDim(), constraintDim());

      PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
      MatrixType & res_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType, res);
      OSIMinv_tmp.noalias() = D.head(constraintDim()).asDiagonal() * U1.adjoint();
      res_.noalias() = -U1 * OSIMinv_tmp;
      PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    }

    /// \brief Returns the Cholesky decomposition expression associated to the underlying Delassus
    /// matrix.
    DelassusCholeskyExpression getDelassusCholeskyExpression() const
    {
      return DelassusCholeskyExpression(*this);
    }

    ///
    /// \brief Returns the Operational Space Inertia Matrix resulting from the decomposition.
    ///
    Matrix getOperationalSpaceInertiaMatrix() const
    {
      Matrix res(constraintDim(), constraintDim());
      getOperationalSpaceInertiaMatrix(res);
      return res;
    }

    template<typename MatrixType>
    void getOperationalSpaceInertiaMatrix(const Eigen::MatrixBase<MatrixType> & res_) const
    {
      MatrixType & res = PINOCCHIO_EIGEN_CONST_CAST(MatrixType, res_);
      typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType
        ConstBlockXpr;
      //        typedef typename RowMatrix::ConstBlockXpr ConstBlockXpr;
      const Eigen::TriangularView<ConstBlockXpr, Eigen::UnitUpper> U1 =
        U.topLeftCorner(constraintDim(), constraintDim())
          .template triangularView<Eigen::UnitUpper>();

      PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
      U1inv.setIdentity();
      U1.solveInPlace(U1inv); // TODO: implement Sparse Inverse
      OSIMinv_tmp.noalias() = -U1inv.adjoint() * Dinv.head(constraintDim()).asDiagonal();
      res.noalias() = OSIMinv_tmp * U1inv;
      PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    }

    Matrix getInverseMassMatrix() const
    {
      Matrix res(nv, nv);
      getInverseMassMatrix(res);
      return res;
    }

    template<typename MatrixType>
    void getInverseMassMatrix(const Eigen::MatrixBase<MatrixType> & res_) const
    {
      MatrixType & res = PINOCCHIO_EIGEN_CONST_CAST(MatrixType, res_);
      typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType
        ConstBlockXpr;
      //        typedef typename RowMatrix::ConstBlockXpr ConstBlockXpr;
      const Eigen::TriangularView<ConstBlockXpr, Eigen::UnitUpper> U4 =
        U.bottomRightCorner(nv, nv).template triangularView<Eigen::UnitUpper>();

      PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
      U4inv.setIdentity();
      U4.solveInPlace(U4inv); // TODO: implement Sparse Inverse
      Minv_tmp.noalias() = U4inv.adjoint() * Dinv.tail(nv).asDiagonal();
      res.noalias() = Minv_tmp * U4inv;
      PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    }

    template<typename MatrixType>
    void getJMinv(const Eigen::MatrixBase<MatrixType> & res_) const
    {
      MatrixType & res = PINOCCHIO_EIGEN_CONST_CAST(MatrixType, res_);
      typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType
        ConstBlockXpr;
      const Eigen::TriangularView<ConstBlockXpr, Eigen::UnitUpper> U4 =
        U.bottomRightCorner(nv, nv).template triangularView<Eigen::UnitUpper>();
      ConstBlockXpr U2 = U.topRightCorner(constraintDim(), nv);
      PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
      U4inv.setIdentity();
      U4.solveInPlace(U4inv); // TODO: implement Sparse Inverse
      res.noalias() = U2 * U4inv;
      PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    }

    ///
    /// \brief Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix
    ///        related to the system mass matrix and the Jacobians of the contact patches contained
    ///        in the vector of RigidConstraintModel named contact_models.
    ///
    /// \param[in] model Model of the dynamical system
    /// \param[in] data Data related to model containing the computed mass matrix and the Jacobian
    /// of the kinematic tree \param[in] contact_models Vector containing the contact models (which
    /// frame is in contact and the type of contact: ponctual, 6D rigid, etc.) \param[out]
    /// contact_datas Vector containing the contact data related to the contact_models. \param[in]
    /// mu Positive regularization factor allowing to enforce the definite property of the KKT
    /// matrix.
    ///
    /// \remarks The mass matrix and the Jacobians of the dynamical system should have been computed
    /// first. This can be achieved by simply calling pinocchio::crba.
    ///
    // TODO Remove when API is stabilized
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<
      typename S1,
      int O1,
      template<typename, int> class JointCollectionTpl,
      class ConstraintModelAllocator,
      class ConstraintDataAllocator>
    void compute(
      const ModelTpl<S1, O1, JointCollectionTpl> & model,
      DataTpl<S1, O1, JointCollectionTpl> & data,
      const std::vector<RigidConstraintModelTpl<S1, O1>, ConstraintModelAllocator> & contact_models,
      std::vector<RigidConstraintDataTpl<S1, O1>, ConstraintDataAllocator> & contact_datas,
      const S1 mu = S1(0.))
    {
      compute(model, data, contact_models, contact_datas, Vector::Constant(U1inv.rows(), mu));
    }

    ///
    /// \brief Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix
    ///        related to the system mass matrix and the Jacobians of the contact patches contained
    ///        in the vector of RigidConstraintModel named contact_models.
    ///
    /// \param[in] model Model of the dynamical system
    /// \param[in] data Data related to model containing the computed mass matrix and the Jacobian
    /// of the kinematic tree \param[in] contact_models Vector containing the contact models (which
    /// frame is in contact and the type of contact: ponctual, 6D rigid, etc.) \param[out]
    /// contact_datas Vector containing the contact data related to the contact_models. \param[in]
    /// mus Vector of positive regularization factor allowing to enforce the definite property of
    /// the KKT matrix.
    ///
    /// \remarks The mass matrix and the Jacobians of the dynamical system should have been computed
    /// first. This can be achieved by simply calling pinocchio::crba.
    ///
    template<
      typename S1,
      int O1,
      template<typename, int> class JointCollectionTpl,
      class ConstraintModelAllocator,
      class ConstraintDataAllocator,
      typename VectorLike>
    void compute(
      const ModelTpl<S1, O1, JointCollectionTpl> & model,
      DataTpl<S1, O1, JointCollectionTpl> & data,
      const std::vector<RigidConstraintModelTpl<S1, O1>, ConstraintModelAllocator> & contact_models,
      std::vector<RigidConstraintDataTpl<S1, O1>, ConstraintDataAllocator> & contact_datas,
      const Eigen::MatrixBase<VectorLike> & mus);
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Update the damping terms on the upper left block part of the KKT matrix. The damping
    /// terms should be all positives.
    ///
    /// \param[in] mus Vector of positive regularization factor allowing to enforce the definite
    /// property of the KKT matrix.
    ///
    template<typename VectorLike>
    void updateDamping(const Eigen::MatrixBase<VectorLike> & mus);

    ///
    /// \brief Update the damping term on the upper left block part of the KKT matrix. The damping
    /// terms should be all positives.
    ///
    /// \param[in] mu Regularization factor allowing to enforce the definite property of the KKT
    /// matrix.
    ///
    void updateDamping(const Scalar & mu);

    /// \brief Size of the decomposition
    Eigen::DenseIndex size() const
    {
      return D.size();
    }

    /// \brief Returns the total dimension of the constraints contained in the Cholesky
    /// factorization
    Eigen::DenseIndex constraintDim() const
    {
      return size() - nv;
    }

    /// \brief Returns the number of contacts associated to this decomposition.
    Eigen::DenseIndex numContacts() const
    {
      return num_contacts;
    }

    ///
    ///  \brief Computes the solution of \f$ A x = b \f$ where *this is the Cholesky decomposition
    /// of A.         "in-place" version of ContactCholeskyDecompositionTpl::solve(b) where the
    /// result is written in b.
    ///        This functions takes as input the vector b, and returns the solution \f$ x = A^-1 b
    ///        \f$.
    ///
    /// \param[inout] mat The right-and-side term which also contains the solution of the linear
    /// system.
    ///
    /// \sa ContactCholeskyDecompositionTpl::solve
    template<typename MatrixLike>
    void solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const;

    ///
    ///  \brief Computes the solution of \f$ A x = b \f$ where *this is the Cholesky decomposition
    /// of A.
    ///        This functions takes as input the vector b, and returns the solution \f$ x = A^-1 b
    ///        \f$.
    ///
    /// \param[inout] mat The right-and-side term.
    ///
    /// \sa ContactCholeskyDecompositionTpl::solveInPlace
    template<typename MatrixLike>
    Matrix solve(const Eigen::MatrixBase<MatrixLike> & mat) const;

    ///
    ///  \brief Retrieves the Cholesky decomposition of the Mass Matrix contained in *this.
    ///
    /// \param[in] model Model of the dynamical system.
    ///
    template<typename S1, int O1, template<typename, int> class JointCollectionTpl>
    ContactCholeskyDecompositionTpl
    getMassMatrixChoeslkyDecomposition(const ModelTpl<S1, O1, JointCollectionTpl> & model) const;

    ///@{
    /// \brief Vectorwize operations
    template<typename MatrixLike>
    void Uv(const Eigen::MatrixBase<MatrixLike> & mat) const;

    template<typename MatrixLike>
    void Utv(const Eigen::MatrixBase<MatrixLike> & mat) const;

    template<typename MatrixLike>
    void Uiv(const Eigen::MatrixBase<MatrixLike> & mat) const;

    template<typename MatrixLike>
    void Utiv(const Eigen::MatrixBase<MatrixLike> & mat) const;
    ///@}

    /// \brief Returns the matrix resulting from the decomposition
    Matrix matrix() const;

    /// \brief Fill the input matrix with the matrix resulting from the decomposition
    template<typename MatrixType>
    void matrix(const Eigen::MatrixBase<MatrixType> & res) const;

    /// \brief Returns the inverse matrix resulting from the decomposition
    Matrix inverse() const;

    /// \brief Fill the input matrix with the inverse matrix resulting from the decomposition
    template<typename MatrixType>
    void inverse(const Eigen::MatrixBase<MatrixType> & res) const;

    // data
    Vector D, Dinv;
    RowMatrix U;

    ///@{
    /// \brief Friend algorithms
    template<typename MatrixLike, int ColsAtCompileTime>
    friend struct details::UvAlgo;

    template<typename MatrixLike, int ColsAtCompileTime>
    friend struct details::UtvAlgo;

    template<typename MatrixLike, int ColsAtCompileTime>
    friend struct details::UivAlgo;

    template<typename MatrixLike, int ColsAtCompileTime>
    friend struct details::UtivAlgo;

    // TODO Remove when API is stabilized
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<typename Scalar, int Options, typename VectorLike>
    friend VectorLike & details::inverseAlgo(
      const ContactCholeskyDecompositionTpl<Scalar, Options> & chol,
      const Eigen::DenseIndex col,
      const Eigen::MatrixBase<VectorLike> & vec);
    ///@}

    template<typename S1, int O1>
    bool operator==(const ContactCholeskyDecompositionTpl<S1, O1> & other) const;

    template<typename S1, int O1>
    bool operator!=(const ContactCholeskyDecompositionTpl<S1, O1> & other) const;
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  protected:
    IndexVector parents_fromRow;
    IndexVector nv_subtree_fromRow;

    ///  \brief Last child of the given joint index
    IndexVector last_child;

    Vector DUt; // temporary containing the results of D * U^t

    /// \brief Dimension of the tangent of the configuration space of the model
    Eigen::DenseIndex nv;

    ///  \brief Number of contacts.
    Eigen::DenseIndex num_contacts;

    VectorOfSliceVector rowise_sparsity_pattern;

    /// \brief Inverse of the top left block of U
    mutable Matrix U1inv;
    /// \brief Inverse of the bottom right block of U
    mutable Matrix U4inv;

    mutable RowMatrix OSIMinv_tmp, Minv_tmp;
  };

  template<typename ContactCholeskyDecomposition>
  struct traits<DelassusCholeskyExpressionTpl<ContactCholeskyDecomposition>>
  {
    enum
    {
      RowsAtCompileTime = Eigen::Dynamic
    };
    typedef typename ContactCholeskyDecomposition::Scalar Scalar;
    typedef typename ContactCholeskyDecomposition::Matrix Matrix;
    typedef typename ContactCholeskyDecomposition::Vector Vector;
  };

  template<typename _ContactCholeskyDecomposition>
  struct DelassusCholeskyExpressionTpl
  : DelassusOperatorBase<DelassusCholeskyExpressionTpl<_ContactCholeskyDecomposition>>
  {
    typedef _ContactCholeskyDecomposition ContactCholeskyDecomposition;
    typedef typename ContactCholeskyDecomposition::Scalar Scalar;
    typedef typename ContactCholeskyDecomposition::Vector Vector;
    typedef typename ContactCholeskyDecomposition::Matrix Matrix;
    typedef typename ContactCholeskyDecomposition::RowMatrix RowMatrix;
    typedef DelassusCholeskyExpressionTpl<_ContactCholeskyDecomposition> Self;
    typedef DelassusOperatorBase<Self> Base;

    typedef
      typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::Type RowMatrixBlockXpr;
    typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType
      RowMatrixConstBlockXpr;

    enum
    {
      RowsAtCompileTime = traits<DelassusCholeskyExpressionTpl>::RowsAtCompileTime
    };

    explicit DelassusCholeskyExpressionTpl(const ContactCholeskyDecomposition & self)
    : Base(self.constraintDim())
    , self(self)
    {
    }

    template<typename MatrixIn, typename MatrixOut>
    void applyOnTheRight(
      const Eigen::MatrixBase<MatrixIn> & x, const Eigen::MatrixBase<MatrixOut> & res) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(x.rows(), self.constraintDim());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(res.rows(), self.constraintDim());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(res.cols(), x.cols());

      PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();

      const RowMatrixConstBlockXpr U1 =
        self.U.topLeftCorner(self.constraintDim(), self.constraintDim());

      if (x.cols() <= self.constraintDim())
      {
        RowMatrixBlockXpr tmp_mat =
          const_cast<ContactCholeskyDecomposition &>(self).OSIMinv_tmp.topLeftCorner(
            self.constraintDim(), x.cols());
        //            tmp_mat.noalias() = U1.adjoint() * x;
        triangularMatrixMatrixProduct<Eigen::UnitLower>(U1.adjoint(), x.derived(), tmp_mat);
        tmp_mat.array().colwise() *= -self.D.head(self.constraintDim()).array();
        //            res.const_cast_derived().noalias() = U1 * tmp_mat;
        triangularMatrixMatrixProduct<Eigen::UnitUpper>(U1, tmp_mat, res.const_cast_derived());
      }
      else // do memory allocation
      {
        RowMatrix tmp_mat(x.rows(), x.cols());
        //            tmp_mat.noalias() = U1.adjoint() * x;
        triangularMatrixMatrixProduct<Eigen::UnitLower>(U1.adjoint(), x.derived(), tmp_mat);
        tmp_mat.array().colwise() *= -self.D.head(self.constraintDim()).array();
        //            res.const_cast_derived().noalias() = U1 * tmp_mat;
        triangularMatrixMatrixProduct<Eigen::UnitUpper>(U1, tmp_mat, res.const_cast_derived());
      }

      PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    }

    template<typename MatrixDerived>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived)
    operator*(const Eigen::MatrixBase<MatrixDerived> & x) const
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived) ReturnType;
      ReturnType res(self.constraintDim(), x.cols());
      applyOnTheRight(x.derived(), res);
      return res;
    }

    template<typename MatrixDerived>
    void solveInPlace(const Eigen::MatrixBase<MatrixDerived> & x) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(x.rows(), self.constraintDim());

      const Eigen::TriangularView<RowMatrixConstBlockXpr, Eigen::UnitUpper> U1 =
        self.U.topLeftCorner(self.constraintDim(), self.constraintDim())
          .template triangularView<Eigen::UnitUpper>();

      PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
      U1.solveInPlace(x.const_cast_derived());
      x.const_cast_derived().array().colwise() *= -self.Dinv.head(self.constraintDim()).array();
      U1.adjoint().solveInPlace(x);
      PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    }

    template<typename MatrixDerivedIn, typename MatrixDerivedOut>
    void solve(
      const Eigen::MatrixBase<MatrixDerivedIn> & x,
      const Eigen::MatrixBase<MatrixDerivedOut> & res) const
    {
      res.const_cast_derived() = x;
      solveInPlace(res.const_cast_derived());
    }

    template<typename MatrixDerived>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived)
      solve(const Eigen::MatrixBase<MatrixDerived> & x) const
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived) ReturnType;
      ReturnType res(self.constraintDim(), x.cols());
      solve(x.derived(), res);
      return res;
    }

    /// \brief Returns the Constraint Cholesky decomposition associated to this
    /// DelassusCholeskyExpression.
    const ContactCholeskyDecomposition & cholesky() const
    {
      return self;
    }

    Matrix matrix() const
    {
      return self.getInverseOperationalSpaceInertiaMatrix();
    }

    Matrix inverse() const
    {
      return self.getOperationalSpaceInertiaMatrix();
    }

    ///
    /// \brief Add a damping term to the diagonal of the Delassus matrix. The damping terms should
    /// be all positives.
    ///
    /// \param[in] mus Vector of positive regularization factor allowing to enforce the definite
    /// positiveness of the matrix.
    ///
    template<typename VectorLike>
    void updateDamping(const Eigen::MatrixBase<VectorLike> & mus)
    {
      const_cast<ContactCholeskyDecomposition &>(self).updateDamping(mus);
    }

    ///
    /// \brief Add a damping term to the diagonal of the Delassus matrix. The damping term should be
    /// positive.
    ///
    /// \param[in] mu Regularization factor allowing to enforce the definite positiveness of the
    /// matrix.
    ///
    void updateDamping(const Scalar & mu)
    {
      const_cast<ContactCholeskyDecomposition &>(self).updateDamping(mu);
    }

    Eigen::DenseIndex size() const
    {
      return self.constraintDim();
    }
    Eigen::DenseIndex rows() const
    {
      return size();
    }
    Eigen::DenseIndex cols() const
    {
      return size();
    }

  protected:
    const ContactCholeskyDecomposition & self;
  }; // DelassusCholeskyExpression

} // namespace pinocchio

// Because of a GCC bug we should NEVER define a function that use ContactCholeskyDecompositionTpl
// before doing the explicit template instantiation.
// If we don't take care, GCC will not accept any visibility attribute when declaring the
// explicit template instantiation of the ContactCholeskyDecompositionTpl class.
// The warning message will look like this: type attributes ignored after type is already defined
// [-Wattributes] A minimal code example is added on the PR
// (https://github.com/stack-of-tasks/pinocchio/pull/2469)
#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/contact-cholesky.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#include "pinocchio/algorithm/contact-cholesky.hxx"

#endif // ifndef __pinocchio_algorithm_contact_cholesky_hpp__
