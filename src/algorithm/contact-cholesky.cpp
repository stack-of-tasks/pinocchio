//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONTACT_CHOLESKY

#include "pinocchio/algorithm/contact-cholesky.hpp"

namespace pinocchio {
    namespace details {
      template context::VectorXs & inverseAlgo
        <context::Scalar, context::Options, context::VectorXs>
      (const ContactCholeskyDecompositionTpl<context::Scalar,context::Options> &, const Eigen::DenseIndex, const Eigen::MatrixBase<context::VectorXs> &);
    }
    template struct ContactCholeskyDecompositionTpl<context::Scalar, context::Options>;

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::allocate 
      <context::Scalar, context::Options, JointCollectionDefaultTpl, typename context::RigidConstraintModelVector::allocator_type>
    (const context::Model &, const context::RigidConstraintModelVector &);

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::getInverseOperationalSpaceInertiaMatrix 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::getOperationalSpaceInertiaMatrix 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::getInverseMassMatrix 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;  
    
    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::compute 
      <context::Scalar, context::Options, JointCollectionDefaultTpl, typename context::RigidConstraintModelVector::allocator_type, typename context::RigidConstraintDataVector::allocator_type>
    (const context::Model &, context::Data &, const context::RigidConstraintModelVector &, context::RigidConstraintDataVector &, const context::Scalar);

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::solveInPlace 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;  

    template ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Matrix ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::solve 
      <context::MatrixXs>
    (const Eigen::MatrixBase<ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Matrix> &) const;  

    template ContactCholeskyDecompositionTpl<context::Scalar, context::Options> ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::getMassMatrixChoeslkyDecomposition 
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &) const;  

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Uv 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;  

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Utv 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;  

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Uiv 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;                       

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Utiv 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;   

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::matrix 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;    

    template void ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::inverse 
      <context::MatrixXs>
    (const Eigen::MatrixBase<context::MatrixXs> &) const;    

    template bool ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::operator== 
      <context::Scalar, context::Options>
    (const ContactCholeskyDecompositionTpl<context::Scalar, context::Options> &) const;  

    template bool ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::operator!= 
      <context::Scalar, context::Options>
    (const ContactCholeskyDecompositionTpl<context::Scalar, context::Options> &) const;  
        

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CONTACT_CHOLESKY
