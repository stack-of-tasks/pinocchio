//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_algorithm_contact_cholesky_hpp__
#define __pinocchio_algorithm_contact_cholesky_hpp__

#include "pinocchio/multibody/model.hpp"
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
      typedef ContactInfoTpl<Scalar,Options> ContactInfo;
      typedef Eigen::Matrix<Eigen::DenseIndex,Eigen::Dynamic,1,Options> IndexVector;
      typedef Eigen::Matrix<bool,Eigen::Dynamic,1,Options> BooleanVector;
      
      ///
      /// \brief Memory allocation of the vectors D, Dinv, and the upper triangular matrix U.
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
      void allocate(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                    const std::vector<ContactInfoTpl<S1,O1>,Allocator> & contact_infos);
      
      ///
      /// \brief Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix
      ///        related to the system mass matrix and the Jacobians of the contact patches contained in
      ///        the vector of ContactInfo named contact_infos.
      ///
      /// \param[in] model Model of the dynamical system
      /// \param[in] data Data related to model containing the computed mass matrix and the Jacobian of the kinematic tree
      /// \param[in] contact_infos Vector containing the contact information (which frame is in contact and the type of contact: ponctual, 6D rigid, etc.)
      /// \param[in] mu Regularization factor allowing to enforce the definite propertie of the KKT matrix.
      ///
      /// \remarks The system mass matrix and the Jacobians of the kinematic tree should have been computed first. This can be achieved by calling pinocchio::crba.
      ///
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
      void compute(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                   const DataTpl<S1,O1,JointCollectionTpl> & data,
                   const std::vector<ContactInfoTpl<S1,O1>,Allocator> & contact_infos,
                   const S1 mu = 0.);
      
      /// \brief Size of the decomposition
      Eigen::DenseIndex dim() const { return D.size(); }
      
      template<typename MatrixLike>
      void solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const;
      
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
      
      // data
      Vector D, Dinv;
      Matrix U;
      
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
      ///@}
    protected:
      
      IndexVector parents_fromRow;
      IndexVector nv_subtree_fromRow;
      /// \brief Last child of the given joint index
      IndexVector last_child;
      
      std::vector<BooleanVector> extented_parents_fromRow;
      Vector DUt; // temporary containing the results of D * U^t
      
      /// \brief Dimension of the tangent of the configuration space of the model
      Eigen::DenseIndex nv;
      
    };
    
    typedef ContactCholeskyDecompositionTpl<double,0> ContactCholeskyDecomposition;
    
  } // namespace cholesky
    
}

#include "pinocchio/algorithm/contact-cholesky.hxx"

#endif // ifndef __pinocchio_algorithm_contact_cholesky_hpp__
