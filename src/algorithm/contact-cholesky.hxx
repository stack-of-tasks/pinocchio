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
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
    void
    ContactCholeskyDecompositionTpl<Scalar,Options>::
    allocate(const ModelTpl<S1,O1,JointCollectionTpl> & model,
             const std::vector<ContactInfoTpl<S1,O1>,Allocator> & contact_infos)
    {
      typedef ContactInfoTpl<S1,O1> ContactInfo;
      typedef std::vector<ContactInfo,Allocator> ContactInfoVector;
      
      nv = model.nv;
      
      Eigen::DenseIndex num_total_constraints = 0;
      for(typename ContactInfoVector::const_iterator it = contact_infos.begin();
          it != contact_infos.end();
          ++it)
      {
        assert(it->dim() > 0 && "The dimension of the contact info must be positive");
        num_total_constraints += it->dim();
      }
      
      const Eigen::DenseIndex total_dim = nv + num_total_constraints;
      
      // Compute first parents_fromRow for all the joints.
      // This code is very similar to the code of Data::computeParents_fromRow,
      // but shifted with a value corresponding to the number of constraints.
      parents_fromRow.resize(total_dim);
      parents_fromRow.fill(-1);
      
      nv_subtree_fromRow.resize(total_dim);
      
      last_child.resize(model.njoints);
      last_child.fill(-1);
      for(long joint_id = model.njoints-1; joint_id >= 0; --joint_id)
      {
        const JointIndex & parent = model.parents[(size_t)joint_id];
        if(last_child[joint_id] == -1)
          last_child[joint_id] = joint_id;
        last_child[(Eigen::DenseIndex)parent] = std::max(last_child[joint_id],
                                                         last_child[(Eigen::DenseIndex)parent]);
      }
      
      for(JointIndex joint_id = 1;joint_id < (JointIndex)(model.njoints);joint_id++)
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
        
        nv_subtree_fromRow[idx_vj+num_total_constraints]
        = model.joints[(size_t)last_child[(Eigen::DenseIndex)joint_id]].idx_v()
        + model.joints[(size_t)last_child[(Eigen::DenseIndex)joint_id]].nv()
        - idx_vj;

        for(int row=1;row<nvj;++row)
        {
          parents_fromRow[idx_vj+num_total_constraints+row] = idx_vj + row - 1 + num_total_constraints;
          nv_subtree_fromRow[idx_vj+num_total_constraints+row] = nv_subtree_fromRow[idx_vj+num_total_constraints]-row;
        }
      }
      
      // Fill nv_subtree_fromRow for constraints
      Eigen::DenseIndex row_id = 0;
      for(typename ContactInfoVector::const_iterator it = contact_infos.begin();
          it != contact_infos.end();
          ++it)
      {
        const ContactInfo & cinfo = *it;
        const FrameIndex frame_id = cinfo.frame_id;
        const JointIndex joint_id = model.frames[frame_id].parent;
        const typename Model::JointModel & joint = model.joints[joint_id];
        
        const Eigen::DenseIndex nv = joint.idx_v() + joint.nv();
        for(Eigen::DenseIndex k = 0; k < cinfo.dim(); ++k)
        {
          nv_subtree_fromRow[row_id] = nv + (num_total_constraints - row_id);
          row_id++;
        }
      }
      assert(row_id == num_total_constraints);
     
      // Allocate and fill sparsity indexes
      static const bool default_sparsity_value = false;
      extented_parents_fromRow.resize(contact_infos.size(),BooleanVector::Constant(total_dim,default_sparsity_value));
      for(size_t ee_id = 0; ee_id < extented_parents_fromRow.size(); ++ee_id)
      {
        BooleanVector & indexes = extented_parents_fromRow[ee_id];
        indexes.resize(total_dim); indexes.fill(default_sparsity_value);
        
        const FrameIndex frame_id = contact_infos[ee_id].frame_id;
        const JointIndex joint_id = model.frames[frame_id].parent;
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
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    compute(const ModelTpl<S1,O1,JointCollectionTpl> & model,
            const DataTpl<S1,O1,JointCollectionTpl> & data,
            const std::vector<ContactInfoTpl<S1,O1>,Allocator> & contact_infos,
            const S1 mu)
    {
      typedef ContactInfoTpl<S1,O1> ContactInfo;
      assert(model.check(data) && "data is not consistent with model.");
      
      const Eigen::DenseIndex total_dim = dim();
      const Eigen::DenseIndex total_constraints_dim = total_dim - nv;
      
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
      const typename Data::MatrixXs & M = data.M;
      
      const size_t num_ee = contact_infos.size();
      
      // Update frame placements if needed
      for(size_t f = 0; f < num_ee; ++f)
      {
        if(contact_infos[f].reference_frame == WORLD) continue; // skip useless computations
        
        const typename Model::FrameIndex & parent_frame_id = contact_infos[f].frame_id;
        const typename Model::Frame & frame = model.frames[parent_frame_id];
        typename Data::SE3 & oMf = data.oMf[parent_frame_id];
        
        const typename Model::JointIndex & parent_joint_id = model.frames[parent_frame_id].parent;
        
        oMf = data.oMi[parent_joint_id] * frame.placement;
      }
      
      // Core
      Motion Jcol_motion;
      for(Eigen::DenseIndex j=nv-1;j>=0;--j)
      {
        // Classic Cholesky decomposition related to the mass matrix
        const Eigen::DenseIndex jj = total_constraints_dim + j; // shifted index
        const Eigen::DenseIndex NVT = nv_subtree_fromRow[jj]-1;
        typename Vector::SegmentReturnType DUt_partial = DUt.head(NVT);
        
        if(NVT)
          DUt_partial.noalias() = U.row(jj).segment(jj+1,NVT).transpose()
          .cwiseProduct(D.segment(jj+1,NVT));

        D[jj] = M(j,j) - U.row(jj).segment(jj+1,NVT).dot(DUt_partial);
        assert(D[jj] != 0. && "The diagonal element is equal to zero.");
        Dinv[jj] = Scalar(1)/D[jj];
        
        for(Eigen::DenseIndex _ii = parents_fromRow[jj]; _ii >= total_constraints_dim; _ii = parents_fromRow[_ii])
        {
          const Eigen::DenseIndex _i = _ii - total_constraints_dim;
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
              case WORLD:
              {
                typedef typename Data::Matrix6x::ColXpr ColXpr;
                ColXpr Jcol = data.J.col(j);
                MotionRef<ColXpr> Jcol_motion(Jcol);
                
                switch(cinfo.type)
                {
                  case CONTACT_3D:
                    for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                    {
                      const Eigen::DenseIndex _ii = current_row - _i;
                      U(_ii,jj) = (Jcol_motion.linear()[contact_dim<CONTACT_3D>::value-_i-1] - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                    }
                    break;
                    
                  case CONTACT_6D:
                    for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_6D>::value; _i++)
                    {
                      const Eigen::DenseIndex _ii = current_row - _i;
                      U(_ii,jj) = (Jcol_motion.toVector()[contact_dim<CONTACT_6D>::value-_i-1] - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                    }
                    break;
                    
                  default:
                    assert(false && "Must never happened");
                }
                break;
              } // end case WORLD
              case LOCAL:
              {
                typedef typename Data::Matrix6x::ColXpr ColXpr;
                ColXpr Jcol = data.J.col(j);
                MotionRef<ColXpr> Jcol_motion(Jcol);
                
                const typename Data::SE3 & oMf = data.oMf[cinfo.frame_id];
                Motion Jcol_local(oMf.actInv(Jcol_motion));
                
                switch(cinfo.type)
                {
                  const Eigen::DenseIndex _ii = current_row - _i;
                  U(_ii,jj) = (data.J(contact_dim<CONTACT_3D>::value-_i-1 + LINEAR,j) - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                }
                break;
              } // end case LOCAL
              case LOCAL_WORLD_ALIGNED:
              {
                typedef typename Data::Matrix6x::ColXpr ColXpr;
                ColXpr Jcol = data.J.col(j);
                MotionRef<ColXpr> Jcol_motion(Jcol);
                
                const typename Data::SE3 & oMf = data.oMf[cinfo.frame_id];
                Motion Jcol_local_world_aligned(Jcol_motion);
                Jcol_local_world_aligned.linear() -= oMf.translation().cross(Jcol_local_world_aligned.angular());
                
                switch(cinfo.type)
                {
                  const Eigen::DenseIndex _ii = current_row - _i;
                  U(_ii,jj) = (data.J(contact_dim<CONTACT_6D>::value-_i-1,j) - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
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
    
    template<typename Scalar, int Options>
    template<typename MatrixLike>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      MatrixLike & mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat);
      
      Uiv(mat_);
      mat_.array().colwise() *= Dinv.array();
      Utiv(mat_);
    }
    
    template<typename Scalar, int Options>
    template<typename MatrixLike>
    typename ContactCholeskyDecompositionTpl<Scalar,Options>::Matrix
    ContactCholeskyDecompositionTpl<Scalar,Options>::
    solve(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      Matrix res(mat);
      solveInPlace(res);
      return res;
    }
    
    namespace details
    {
      template<typename MatrixLike, int ColsAtCompileTime>
      struct UvAlgo
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<MatrixLike> & mat)
        {
          MatrixLike & mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat);
          
          assert(mat.rows() == chol.dim() && "The input matrix is of wrong size");
          
          for(Eigen::DenseIndex col_id = 0; col_id < mat_.cols(); ++col_id)
            UvAlgo<typename MatrixLike::ColXpr>::run(chol,mat_.col(col_id));
        }
      };
      
      template<typename VectorLike>
      struct UvAlgo<VectorLike,1>
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<VectorLike> & vec)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike)
          VectorLike & vec_ = PINOCCHIO_EIGEN_CONST_CAST(VectorLike,vec);
          
          assert(vec.size() == chol.dim() && "The input vector is of wrong size");
          const Eigen::DenseIndex num_total_constraints = chol.dim() - chol.nv;
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = 0; k < num_total_constraints; ++k)
          {
            const Eigen::DenseIndex slice_dim = chol.dim() - k - 1;
            vec_[k] += chol.U.row(k).tail(slice_dim).dot(vec_.tail(slice_dim));
          }
          
          for(Eigen::DenseIndex k = num_total_constraints; k <= chol.dim()-2; ++k)
            vec_[k] += chol.U.row(k).segment(k+1,chol.nv_subtree_fromRow[k]-1).dot(vec_.segment(k+1,chol.nv_subtree_fromRow[k]-1));
        }
      };
    } // namespace details
    
    template<typename Scalar, int Options>
    template<typename MatrixLike>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    Uv(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      details::UvAlgo<MatrixLike>::run(*this,
                                       PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat));
    }
    
    namespace details
    {
      template<typename MatrixLike, int ColsAtCompileTime>
      struct UtvAlgo
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<MatrixLike> & mat)
        {
          MatrixLike & mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat);
          
          assert(mat.rows() == chol.dim() && "The input matrix is of wrong size");
          
          for(Eigen::DenseIndex col_id = 0; col_id < mat_.cols(); ++col_id)
            UtvAlgo<typename MatrixLike::ColXpr>::run(chol,mat_.col(col_id));
        }
      };
      
      template<typename VectorLike>
      struct UtvAlgo<VectorLike,1>
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<VectorLike> & vec)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike)
          VectorLike & vec_ = PINOCCHIO_EIGEN_CONST_CAST(VectorLike,vec);
          
          assert(vec.size() == chol.dim() && "The input vector is of wrong size");
          const Eigen::DenseIndex num_total_constraints = chol.constraintDim();
          
          for(Eigen::DenseIndex k = chol.dim()-2; k >= num_total_constraints; --k)
            vec_.segment(k+1,chol.nv_subtree_fromRow[k]-1)
            += chol.U.row(k).segment(k+1,chol.nv_subtree_fromRow[k]-1).transpose()*vec_[k];
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = num_total_constraints-1; k >=0; --k)
          {
            const Eigen::DenseIndex slice_dim = chol.dim() - k - 1;
            vec_.tail(slice_dim) += chol.U.row(k).tail(slice_dim).transpose()*vec_[k];
          }
        }
      };
    } // namespace details
    
    template<typename Scalar, int Options>
    template<typename MatrixLike>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    Utv(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      details::UtvAlgo<MatrixLike>::run(*this,
                                        PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat));
    }
    
    namespace details
    {
      template<typename MatrixLike, int ColsAtCompileTime>
      struct UivAlgo
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<MatrixLike> & mat)
        {
          MatrixLike & mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat);
          
          assert(mat.rows() == chol.dim() && "The input matrix is of wrong size");
          
          for(Eigen::DenseIndex col_id = 0; col_id < mat_.cols(); ++col_id)
            UivAlgo<typename MatrixLike::ColXpr>::run(chol,mat_.col(col_id));
        }
      };
      
      template<typename VectorLike>
      struct UivAlgo<VectorLike,1>
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<VectorLike> & vec)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike)
          VectorLike & vec_ = PINOCCHIO_EIGEN_CONST_CAST(VectorLike,vec);
          
          assert(vec.size() == chol.dim() && "The input vector is of wrong size");
          
          const Eigen::DenseIndex num_total_constraints = chol.dim() - chol.nv;
          for(Eigen::DenseIndex k = chol.dim()-2; k >= num_total_constraints; --k)
            vec_[k] -= chol.U.row(k).segment(k+1,chol.nv_subtree_fromRow[k]-1).dot(vec_.segment(k+1,chol.nv_subtree_fromRow[k]-1));
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = num_total_constraints-1; k >=0; --k)
          {
            const Eigen::DenseIndex slice_dim = chol.dim() - k - 1;
            vec_[k] -= chol.U.row(k).tail(slice_dim).dot(vec_.tail(slice_dim));
          }
        }
      };
    } // namespace details
    
    template<typename Scalar, int Options>
    template<typename MatrixLike>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    Uiv(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      details::UivAlgo<MatrixLike>::run(*this,
                                        PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat));
    }
    
    namespace details
    {
      template<typename MatrixLike, int ColsAtCompileTime>
      struct UtivAlgo
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<MatrixLike> & mat)
        {
          MatrixLike & mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat);
          
          assert(mat.rows() == chol.dim() && "The input matrix is of wrong size");
          
          for(Eigen::DenseIndex col_id = 0; col_id < mat_.cols(); ++col_id)
            UtivAlgo<typename MatrixLike::ColXpr>::run(chol,mat_.col(col_id));
        }
      };
      
      template<typename VectorLike>
      struct UtivAlgo<VectorLike,1>
      {
        template<typename Scalar, int Options>
        static void run(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                        const Eigen::MatrixBase<VectorLike> & vec)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike)
          VectorLike & vec_ = PINOCCHIO_EIGEN_CONST_CAST(VectorLike,vec);
          
          assert(vec.size() == chol.dim() && "The input vector is of wrong size");
          const Eigen::DenseIndex num_total_constraints = chol.constraintDim();
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = 0; k < num_total_constraints; ++k)
          {
            const Eigen::DenseIndex slice_dim = chol.dim() - k - 1;
            vec_.tail(slice_dim) -= chol.U.row(k).tail(slice_dim).transpose() * vec_[k];
          }
          
          for(Eigen::DenseIndex k = num_total_constraints; k <= chol.dim()-2; ++k)
            vec_.segment(k+1,chol.nv_subtree_fromRow[k]-1)
            -= chol.U.row(k).segment(k+1,chol.nv_subtree_fromRow[k]-1).transpose() * vec_[k];
        }
      };
    } // namespace details
    
    template<typename Scalar, int Options>
    template<typename MatrixLike>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    Utiv(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      details::UtivAlgo<MatrixLike>::run(*this,
                                         PINOCCHIO_EIGEN_CONST_CAST(MatrixLike,mat));
    }
    
    template<typename Scalar, int Options>
    typename ContactCholeskyDecompositionTpl<Scalar,Options>::Matrix
    ContactCholeskyDecompositionTpl<Scalar,Options>::matrix() const
    {
      Matrix res(dim(),dim());
      matrix(res);
      return res;
    }
    
    template<typename Scalar, int Options>
    template<typename MatrixType>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    matrix(const Eigen::MatrixBase<MatrixType> & res) const
    {
      MatrixType & res_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType,res);
      res_.noalias() = U * D.asDiagonal() * U.transpose();
    }
    
  } // namespace cholesky
}

#endif // ifndef __pinocchio_algorithm_contact_cholesky_hxx__
