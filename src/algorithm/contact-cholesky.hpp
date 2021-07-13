//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_algorithm_contact_cholesky_hpp__
#define __pinocchio_algorithm_contact_cholesky_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/math/matrix-block.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{
  
  namespace cholesky
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
      VectorLike & inverseAlgo(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                               const Eigen::DenseIndex col,
                               const Eigen::MatrixBase<VectorLike> & vec);
    }
    
    ///
    /// \brief Contact Cholesky decomposition structure. This structure allows
    ///        to compute in a efficient and parsimonious way the Cholesky decomposition
    ///        of the KKT matrix related to the contact dynamics.
    ///        Such a decomposition is usefull when computing both the forward dynamics in contact
    ///        or the related analytical derivatives.
    ///
    ///
    /// \tparam _Scalar Scalar type.
    /// \tparam _Options Alignment Options of the Eigen objects contained in the data structure.
    ///
    template<typename _Scalar, int _Options>
    struct ContactCholeskyDecompositionTpl
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef pinocchio::Index Index;
      typedef _Scalar Scalar;
      enum {
        LINEAR = 0,
        ANGULAR = 3,
        Options = _Options
      };
      
      typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> Vector;
      typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> Matrix;
      typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Matrix) RowMatrix;
      typedef RigidConstraintModelTpl<Scalar,Options> RigidConstraintModel;
      typedef RigidConstraintDataTpl<Scalar,Options> RigidConstraintData;
      typedef Eigen::Matrix<Eigen::DenseIndex,Eigen::Dynamic,1,Options> IndexVector;
      typedef typename PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(IndexVector) VectorOfIndexVector;
      typedef Eigen::Matrix<bool,Eigen::Dynamic,1,Options> BooleanVector;
      
      ///@{
      /// \brief Data information related to the Sparsity structure of the Cholesky decompostion
      struct Slice
      {
        Slice(const Eigen::DenseIndex & first_index,
              const Eigen::DenseIndex & size)
        : first_index(first_index), size(size)
        {}
        
        Eigen::DenseIndex first_index;
        Eigen::DenseIndex size;
      };
      
      typedef std::vector<Slice> SliceVector;
      typedef std::vector<SliceVector> VectorOfSliceVector;
      ///@}
      
      ///
      /// \brief Default constructor
      ///
      ContactCholeskyDecompositionTpl() {}
      
      ///
      /// \brief Constructor from a model.
      ///
      /// \param[in] model Model of the kinematic tree.
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
      ContactCholeskyDecompositionTpl(const ModelTpl<S1,O1,JointCollectionTpl> & model)
      {
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) empty_contact_models;
        allocate(model,empty_contact_models);
      }
      
      ///
      /// \brief Constructor from a model and a collection of RigidConstraintModel objects.
      ///
      /// \param[in] model Model of the kinematic tree
      /// \param[in] contact_models Vector of RigidConstraintModel objects containing the contact information
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
      ContactCholeskyDecompositionTpl(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                                      const std::vector<RigidConstraintModelTpl<S1,O1>,Allocator> & contact_models)
      {
        allocate(model,contact_models);
      }
      
      ///
      /// \brief Memory allocation of the vectors D, Dinv, and the upper triangular matrix U.
      ///
      /// \param[in] model Model of the kinematic tree
      /// \param[in] contact_models Vector of RigidConstraintModel objects containing the contact information
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
      void allocate(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                    const std::vector<RigidConstraintModelTpl<S1,O1>,Allocator> & contact_models);
      
      ///
      /// \brief Returns the Inverse of the Operational Space Inertia Matrix resulting from the decomposition.
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
        typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType ConstBlockXpr;
//        typedef typename RowMatrix::ConstBlockXpr ConstBlockXpr;
        const ConstBlockXpr U1
        = U.topLeftCorner(constraintDim(),constraintDim());
        
        PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
        MatrixType & res_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType,res);
        OSIMinv_tmp.noalias() = D.head(constraintDim()).asDiagonal() * U1.adjoint();
        res_.noalias() = -U1 * OSIMinv_tmp;
        PINOCCHIO_EIGEN_MALLOC_ALLOWED();
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
        MatrixType& res = PINOCCHIO_EIGEN_CONST_CAST(MatrixType,res_);
        typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType ConstBlockXpr;
//        typedef typename RowMatrix::ConstBlockXpr ConstBlockXpr;
        const Eigen::TriangularView<ConstBlockXpr,Eigen::UnitUpper> U1
        = U.topLeftCorner(constraintDim(),constraintDim()).template triangularView<Eigen::UnitUpper>();

        PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
        U1inv.setIdentity(); U1.solveInPlace(U1inv); // TODO: implement Sparse Inverse
        OSIMinv_tmp.noalias() = -U1inv.adjoint() * Dinv.head(constraintDim()).asDiagonal();
        res.noalias() = OSIMinv_tmp * U1inv;
        PINOCCHIO_EIGEN_MALLOC_ALLOWED();
      }

      template<typename MatrixType>
      void getInverseMassMatrix(const Eigen::MatrixBase<MatrixType> & res_) const
      {
        MatrixType& res = PINOCCHIO_EIGEN_CONST_CAST(MatrixType,res_);
        typedef typename SizeDepType<Eigen::Dynamic>::template BlockReturn<RowMatrix>::ConstType ConstBlockXpr;
//        typedef typename RowMatrix::ConstBlockXpr ConstBlockXpr;
        const Eigen::TriangularView<ConstBlockXpr,Eigen::UnitUpper> U4
          = U.bottomRightCorner(nv,nv).template triangularView<Eigen::UnitUpper>();

        PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();
        U4inv.setIdentity(); U4.solveInPlace(U4inv); // TODO: implement Sparse Inverse
        Minv_tmp.noalias() = U4inv.adjoint() * Dinv.tail(nv).asDiagonal();
        res.noalias() = Minv_tmp * U4inv;
        PINOCCHIO_EIGEN_MALLOC_ALLOWED();
      }
      
      ///
      /// \brief Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix
      ///        related to the system mass matrix and the Jacobians of the contact patches contained in
      ///        the vector of RigidConstraintModel named contact_models.
      ///
      /// \param[in] model Model of the dynamical system
      /// \param[in] data Data related to model containing the computed mass matrix and the Jacobian of the kinematic tree
      /// \param[in] contact_models Vector containing the contact models (which frame is in contact and the type of contact: ponctual, 6D rigid, etc.)
      /// \param[out] contact_datas Vector containing the contact data related to the contact_models.
      /// \param[in] mu Regularization factor allowing to enforce the definite propertie of the KKT matrix.
      ///
      /// \remarks The mass matrix and the Jacobians of the dynamical system should have been computed first. This can be achieved by simply calling pinocchio::crba.
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator>
      void compute(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                   DataTpl<S1,O1,JointCollectionTpl> & data,
                   const std::vector<RigidConstraintModelTpl<S1,O1>,ContactModelAllocator> & contact_models,
                   std::vector<RigidConstraintDataTpl<S1,O1>,ContactDataAllocator> & contact_datas,
                   const S1 mu = S1(0.));
      
      /// \brief Size of the decomposition
      Eigen::DenseIndex size() const { return D.size(); }
      
      /// \brief Returns the total dimension of the constraints contained in the Cholesky factorization
      Eigen::DenseIndex constraintDim() const
      {
        return size() - nv;
      }
      
      /// \brief Returns the number of contacts associated to this decomposition.
      Eigen::DenseIndex numContacts() const { return num_contacts; }
      
      ///
      /// \brief Computes the solution of \f$ A x = b \f$ where *this is the Cholesky decomposition of A.
      ///        "in-place" version of ContactCholeskyDecompositionTpl::solve(b) where the result is written in b.
      ///        This functions takes as input the vector b, and returns the solution \f$ x = A^-1 b \f$.
      ///
      /// \param[inout] mat The right-and-side term which also contains the solution of the linear system.
      ///
      /// \sa ContactCholeskyDecompositionTpl::solve
      template<typename MatrixLike>
      void solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const;
      
      ///
      /// \brief Computes the solution of \f$ A x = b \f$ where *this is the Cholesky decomposition of A.
      ///        This functions takes as input the vector b, and returns the solution \f$ x = A^-1 b \f$.
      ///
      /// \param[inout] mat The right-and-side term.
      ///
      /// \sa ContactCholeskyDecompositionTpl::solveInPlace
      template<typename MatrixLike>
      Matrix solve(const Eigen::MatrixBase<MatrixLike> & mat) const;
      
      ///
      /// \brief Retrieves the Cholesky decomposition of the Mass Matrix contained in *this.
      ///
      /// \param[in] model Model of the dynamical system.
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
      ContactCholeskyDecompositionTpl
      getMassMatrixChoeslkyDecomposition(const ModelTpl<S1,O1,JointCollectionTpl> & model) const;
      
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
      
      template<typename Scalar, int Options, typename VectorLike>
      friend VectorLike & details::inverseAlgo(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                                               const Eigen::DenseIndex col,
                                               const Eigen::MatrixBase<VectorLike> & vec);
      ///@}
      
      template<typename S1, int O1>
      bool operator==(const ContactCholeskyDecompositionTpl<S1,O1> & other) const
      {
        bool is_same = true;
        
        if(nv != other.nv || num_contacts != other.num_contacts)
          return false;
        
        if(   D.size() != other.D.size()
           || Dinv.size() != other.Dinv.size()
           || U.rows() != other.U.rows()
           || U.cols() != other.U.cols())
          return false;
        
        is_same &= (D == other.D);
        is_same &= (Dinv == other.Dinv);
        is_same &= (U == other.U);
        
        is_same &= (parents_fromRow == other.parents_fromRow);
        is_same &= (nv_subtree_fromRow == other.nv_subtree_fromRow);
        is_same &= (last_child == other.last_child);
        is_same &= (joint1_indexes == other.joint1_indexes);
        is_same &= (joint2_indexes == other.joint2_indexes);
        is_same &= (colwise_sparsity_patterns == other.colwise_sparsity_patterns);
	is_same &= (colwise_loop_sparsity_patterns == other.colwise_loop_sparsity_patterns);
//        is_same &= (rowise_sparsity_pattern == other.rowise_sparsity_pattern);
        
        return is_same;
      }
      
      template<typename S1, int O1>
      bool operator!=(const ContactCholeskyDecompositionTpl<S1,O1> & other) const
      {
        return !(*this == other);
      }
      
      const IndexVector & getConstraintSparsityPattern(const size_t constraint_id) const
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(constraint_id < colwise_sparsity_patterns.size(),
                                       "The index of the constraint is invalid.");
        return colwise_sparsity_patterns[constraint_id];
      }

      const IndexVector & getLoopSparsityPattern(const size_t constraint_id) const
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(constraint_id < colwise_loop_sparsity_patterns.size(),
                                       "The index of the constraint is invalid.");
        return colwise_loop_sparsity_patterns[constraint_id];
      }
      
      const BooleanVector & getJoint1SparsityPattern(const size_t constraint_id) const
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(constraint_id < joint1_indexes.size(),
                                       "The index of the constraint is invalid.");
        return joint1_indexes[constraint_id];
      }

      const BooleanVector & getJoint2SparsityPattern(const size_t constraint_id) const
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(constraint_id < joint2_indexes.size(),
                                       "The index of the constraint is invalid.");
        return joint2_indexes[constraint_id];
      }

      
      
    protected:
      
      IndexVector parents_fromRow;
      IndexVector nv_subtree_fromRow;
      
      /// \brief Last child of the given joint index
      IndexVector last_child;
      
      std::vector<BooleanVector> joint1_indexes, joint2_indexes;
      VectorOfIndexVector colwise_sparsity_patterns;
      VectorOfIndexVector colwise_loop_sparsity_patterns;
      Vector DUt; // temporary containing the results of D * U^t
      
      /// \brief Dimension of the tangent of the configuration space of the model
      Eigen::DenseIndex nv;
      
      /// \brief Number of contacts.
      Eigen::DenseIndex num_contacts;
      
      VectorOfSliceVector rowise_sparsity_pattern;
      
      /// \brief Inverse of the top left block of U
      mutable Matrix U1inv;
      /// \brief Inverse of the bottom right block of U
      mutable Matrix U4inv;
      
    private:
      
      mutable RowMatrix OSIMinv_tmp, Minv_tmp;
      
    };
    
  } // namespace cholesky
    
}

#endif // ifndef __pinocchio_algorithm_contact_cholesky_hpp__
