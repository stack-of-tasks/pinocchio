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
    ///
    /// \brief Contact cholesky decomposition
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
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
      void allocate(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                    const container::aligned_vector< ContactInfoTpl<S1,O1> > & contact_infos);
      
      template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
      void compute(const ModelTpl<S1,O1,JointCollectionTpl> & model,
                   const DataTpl<S1,O1,JointCollectionTpl> & data,
                   const container::aligned_vector< ContactInfoTpl<S1,O1> > & contact_infos,
                   const S1 mu = 0.);
      
      /// \brief Size of the decomposition
      Eigen::DenseIndex dim() const { return D.size(); }
      
      // data
      Vector D, Dinv;
      Matrix U;
      
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
