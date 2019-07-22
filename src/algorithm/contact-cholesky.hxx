//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_algorithm_contact_cholesky_hxx__
#define __pinocchio_algorithm_contact_cholesky_hxx__

#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  
  namespace cholesky
  {
    
    template<typename Scalar, int Options>
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
    void
    ContactCholeskyDecompositionTpl<Scalar,Options>::
    allocate(const ModelTpl<S1,O1,JointCollectionTpl> & model,
             const container::aligned_vector< ContactInfoTpl<S1,O1> > & contact_infos)
    {
      typedef container::aligned_vector< ContactInfoTpl<S1,O1> > ContactInfoVector;
      
      Eigen::DenseIndex num_total_constraints = 0;
      for(typename ContactInfoVector::const_iterator it = contact_infos.begin();
          it != contact_infos.end();
          ++it)
      {
        assert(it->dim() > 0 && "The dimension of the contact info must be positive");
        num_total_constraints += it->dim();
      }
      
      const Eigen::DenseIndex total_dim = model.nv + num_total_constraints;
      
      // Compute first parents_fromRow for all the joints.
      // This code is very similar to the code of Data::computeParents_fromRow,
      // but shifted with a value corresponding to the number of constraints.
      parents_fromRow.resize(total_dim);
      parents_fromRow.head(num_total_constraints).fill(-1);
      
      nvSubtree_fromRow.resize(total_dim);
      parents_fromRow.fill(-1);
      
      for(JointIndex joint_id=1;joint_id<(JointIndex)(model.njoints);joint_id++)
      {
        const JointIndex & parent = model.parents[joint_id];
        
        const typename Model::JointModel & joint = model.joints[joint_id];
        const typename Model::JointModel & parent_joint = model.joints[parent];
        const int nvj    = joint.nv();
        const int idx_vj = joint.idx_v();
        
        if(parent>0)
          parents_fromRow[idx_vj + num_total_constraints]
          = parent_joint.idx_v() + parent_joint.nv() - 1 + num_total_constraints;
        else
          parents_fromRow[idx_vj+num_total_constraints] = -1;
        
        for(int row=1;row<nvj;++row)
          parents_fromRow[idx_vj+num_total_constraints+row] = idx_vj + row - 1 + num_total_constraints;
      }
      
      // Allocate and fill sparsity indexes
      extented_parents_fromRow.resize(contact_infos.size(),BooleanVector::Constant(total_dim,false));
      for(size_t ee_id = 0; ee_id < extented_parents_fromRow.size(); ++ee_id)
      {
        BooleanVector & indexes = extented_parents_fromRow[ee_id];
        const JointIndex joint_id = model.frames[contact_infos[ee_id].parent].parent;
        const typename Model::JointModel & joint = model.joints[joint_id];
        
        Eigen::DenseIndex current_id = joint.idx_v() + joint.nv() - 1 + num_total_constraints;
        while(parents_fromRow[current_id] != -1)
        {
          indexes[current_id] = true;
          current_id = parents_fromRow[current_id];
        }
        indexes[current_id] = true;
      }
      
      // Allocate memory
      D.resize(total_dim); Dinv.resize(total_dim);
      U.resize(total_dim,total_dim);
      U.setIdentity();
      DUt.resize(total_dim);
    }
    
    template<typename Scalar, int Options>
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    compute(const ModelTpl<S1,O1,JointCollectionTpl> & model,
            const DataTpl<S1,O1,JointCollectionTpl> & data,
            const container::aligned_vector< ContactInfoTpl<S1,O1> > & contact_infos,
            const S1 mu)
    {
      typedef ContactInfoTpl<S1,O1> ContactInfo;
      assert(model.check(data) && "data is not consistent with model.");
      
      const Eigen::DenseIndex total_dim = dim();
      const Eigen::DenseIndex total_constraints_dim = total_dim - model.nv;
      
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
      const typename Data::MatrixXs & M = data.M;
      
      const size_t num_ee = contact_infos.size();
      for(Eigen::DenseIndex j=model.nv-1;j>=0;--j)
      {
        // Classic Cholesky decomposition related to the mass matrix
        const Eigen::DenseIndex jj = total_constraints_dim + j; // shifted index
        const Eigen::DenseIndex NVT = data.nvSubtree_fromRow[(size_t)j]-1;
        typename Vector::SegmentReturnType DUt_partial = DUt.head(NVT);
        
        if(NVT)
          DUt_partial.noalias() = U.row(jj).segment(jj+1,NVT).transpose()
          .cwiseProduct(D.segment(jj+1,NVT));
        
        D[jj] = M(j,j) - U.row(jj).segment(jj+1,NVT).dot(DUt_partial);
        assert(D[jj] != 0. && "The diagonal element is equal to zero.");
        Dinv[jj] = Scalar(1)/D[jj];
        
        for(Eigen::DenseIndex _i = data.parents_fromRow[(size_t)j]; _i >= 0; _i = data.parents_fromRow[(size_t)_i])
        {
          const Eigen::DenseIndex _ii = _i + total_constraints_dim;
          U(_ii,jj) = (M(_i,j) - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
        }
        
        // Constraint handling
        if(num_ee == 0)
          continue;
        
        int current_row = (int)total_constraints_dim - 1;
        for(size_t k = 0; k < num_ee; ++k)
        {
          size_t ee_id = num_ee - k - 1; // start from the last end effector
          
          const BooleanVector & indexes = extented_parents_fromRow[ee_id];
          const ContactInfo & cinfo = contact_infos[ee_id];
          const Eigen::DenseIndex constraint_dim = cinfo.dim();
          
          if(indexes[jj])
          {
            switch(cinfo.type)
            {
              case CONTACT_3D:
                for(int _i = 0; _i < 3; _i++)
                {
                  const Eigen::DenseIndex _ii = current_row - _i;
                  U(_ii,jj) = (data.J(3-_i-1 + LINEAR,j) - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                }
                break;
                
              case CONTACT_6D:
                for(int _i = 0; _i < 6; _i++)
                {
                  const Eigen::DenseIndex _ii = current_row - _i;
                  U(_ii,jj) = (data.J(6-_i-1,j) - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                }
                break;
                
              default:
                assert(false && "Must never happened");
            }
            
          }
          current_row -= constraint_dim;
        }
        
      }
      
      for(Eigen::DenseIndex j = total_constraints_dim-1; j>=0; --j)
      {
        const Eigen::DenseIndex slice_dim = total_dim - j - 1;
        typename Vector::SegmentReturnType DUt_partial = DUt.head(slice_dim);
        DUt_partial.noalias() = U.row(j).segment(j+1,slice_dim).transpose().cwiseProduct(D.segment(j+1,slice_dim));
        
        D[j] = -mu - U.row(j).segment(j+1,slice_dim).dot(DUt_partial);
        assert(D[j] != 0. && "The diagonal element is equal to zero.");
        Dinv[j] = Scalar(1)/D[j];
        
        for(Eigen::DenseIndex _i = j-1; _i >= 0; _i--)
        {
          U(_i,j) = -U.row(_i).segment(j+1,slice_dim).dot(DUt_partial) * Dinv[j];
        }
      }
    }
    
  } // namespace cholesky
}

#endif // ifndef __pinocchio_algorithm_contact_cholesky_hxx__
