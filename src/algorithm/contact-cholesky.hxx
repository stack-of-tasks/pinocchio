//
// Copyright (c) 2019-2021 INRIA
//

#ifndef __pinocchio_algorithm_contact_cholesky_hxx__
#define __pinocchio_algorithm_contact_cholesky_hxx__

#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/check.hpp"

#include <algorithm>

namespace pinocchio
{
  
  namespace cholesky
  {
    
    template<typename Scalar, int Options>
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class Allocator>
    void
    ContactCholeskyDecompositionTpl<Scalar,Options>::
    allocate(const ModelTpl<S1,O1,JointCollectionTpl> & model,
             const std::vector<RigidConstraintModelTpl<S1,O1>,Allocator> & contact_models)
    {
      typedef ModelTpl<S1,O1,JointCollectionTpl> Model;
      typedef RigidConstraintModelTpl<S1,O1> RigidConstraintModel;
      typedef std::vector<RigidConstraintModel,Allocator> RigidConstraintModelVector;
      
      nv = model.nv;
      num_contacts = (Eigen::DenseIndex)contact_models.size();
      
      Eigen::DenseIndex num_total_constraints = 0;
      for(typename RigidConstraintModelVector::const_iterator it = contact_models.begin();
          it != contact_models.end();
          ++it)
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(it->size() > 0,
                                       "The dimension of the constraint must be positive");
        num_total_constraints += it->size();
      }
      
      U1inv.resize(num_total_constraints,num_total_constraints);
      OSIMinv_tmp.resize(num_total_constraints,num_total_constraints);
      U4inv.resize(nv,nv);
      Minv_tmp.resize(nv,nv);

      const Eigen::DenseIndex total_dim = nv + num_total_constraints;
      
      // Compute first parents_fromRow for all the joints.
      // This code is very similar to the code of Data::computeParents_fromRow,
      // but shifted with a value corresponding to the number of constraints.
      parents_fromRow.resize(total_dim);
      parents_fromRow.fill(-1);
      
      nv_subtree_fromRow.resize(total_dim);
//      nv_subtree_fromRow.fill(0);
      
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
        const JointIndex & parent_id = model.parents[joint_id];
        
        const typename Model::JointModel & joint = model.joints[joint_id];
        const typename Model::JointModel & parent_joint = model.joints[parent_id];
        const int nvj    = joint.nv();
        const int idx_vj = joint.idx_v();
        
        if(parent_id>0)
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
      for(typename RigidConstraintModelVector::const_iterator it = contact_models.begin();
          it != contact_models.end();
          ++it)
      {
        const RigidConstraintModel & cmodel = *it;
        const JointIndex joint1_id = cmodel.joint1_id;
        const typename Model::JointModel & joint1 = model.joints[joint1_id];
        const JointIndex joint2_id = cmodel.joint2_id;
        const typename Model::JointModel & joint2 = model.joints[joint2_id];
        
        const Eigen::DenseIndex nv1 = joint1.idx_v() + joint1.nv();
        const Eigen::DenseIndex nv2 = joint2.idx_v() + joint2.nv();
        const Eigen::DenseIndex nv = std::max(nv1,nv2);
        for(Eigen::DenseIndex k = 0; k < cmodel.size(); ++k)
        {
          nv_subtree_fromRow[row_id] = nv + (num_total_constraints - row_id);
          row_id++;
        }
      }
      assert(row_id == num_total_constraints);
     
      // Allocate and fill sparsity joint1_indexes and joint2_indexes
      static const bool default_sparsity_value = false;
      joint1_indexes.resize(static_cast<size_t>(num_contacts),
                            BooleanVector::Constant(total_dim,default_sparsity_value));
      joint2_indexes.resize(static_cast<size_t>(num_contacts),
                            BooleanVector::Constant(total_dim,default_sparsity_value));
      colwise_sparsity_patterns.resize(static_cast<size_t>(num_contacts),
                                       IndexVector());
      colwise_loop_sparsity_patterns.resize(static_cast<size_t>(num_contacts),
                                       IndexVector());      
      for(size_t ee_id = 0; ee_id < joint1_indexes.size(); ++ee_id)
      {
        BooleanVector & joint1_indexes_ee = joint1_indexes[ee_id];
        joint1_indexes_ee.resize(total_dim); joint1_indexes_ee.fill(default_sparsity_value);
        BooleanVector & joint2_indexes_ee = joint2_indexes[ee_id];
        joint2_indexes_ee.resize(total_dim); joint2_indexes_ee.fill(default_sparsity_value);
        IndexVector & colwise_sparsity_patterns_ee = colwise_sparsity_patterns[ee_id];
	IndexVector & colwise_loop_sparsity_patterns_ee = colwise_loop_sparsity_patterns[ee_id];
        
        const RigidConstraintModel & cmodel = contact_models[ee_id];
        
        const JointIndex joint1_id = cmodel.joint1_id;
        JointIndex current1_id = 0;
        if(joint1_id > 0)
          current1_id = joint1_id;

        const JointIndex joint2_id = cmodel.joint2_id;
        JointIndex current2_id = 0;
        if(joint2_id > 0)
          current2_id = joint2_id;

        while(current1_id != current2_id)
        {
          if(current1_id > current2_id)
          {
            const typename Model::JointModel & joint1 = model.joints[current1_id];
            Eigen::DenseIndex current1_row_id = joint1.idx_v() + num_total_constraints;
            for(int k = 0; k < joint1.nv(); ++k,++current1_row_id)
            {
              joint1_indexes_ee[current1_row_id] = true;
            }
            current1_id = model.parents[current1_id];
          }
          else
          {
            const typename Model::JointModel & joint2 = model.joints[current2_id];
            Eigen::DenseIndex current2_row_id = joint2.idx_v() + num_total_constraints;
            for(int k = 0; k < joint2.nv(); ++k,++current2_row_id)
            {
              joint2_indexes_ee[current2_row_id] = true;
            }
            current2_id = model.parents[current2_id];
          }
        }
        assert(current1_id == current2_id && "current1_id should be equal to current2_id");
        // current1_id and current2_id now contains the common ancestor to the two joints.
        if(cmodel.type == CONTACT_3D && cmodel.reference_frame != WORLD)
        {
          JointIndex current_id = current1_id;
          while(current_id > 0)
          {
            const typename Model::JointModel & joint = model.joints[current_id];
            Eigen::DenseIndex current_row_id = joint.idx_v() + num_total_constraints;
            for(int k = 0; k < joint.nv(); ++k,++current_row_id)
            {
              joint1_indexes_ee[current_row_id] = true;
              joint2_indexes_ee[current_row_id] = true;
            }
            current_id = model.parents[current_id];
          }
        }
        
        Eigen::DenseIndex size = 0;
        colwise_sparsity_patterns_ee.resize(total_dim);
        for(Eigen::DenseIndex col_id = 0; col_id < total_dim; ++col_id)
        {
          if(joint1_indexes_ee[col_id] || joint2_indexes_ee[col_id])
            colwise_sparsity_patterns_ee[size++] = col_id;
        }
        colwise_sparsity_patterns_ee.conservativeResize(size);
	
	size = 0;
	colwise_loop_sparsity_patterns_ee.resize(total_dim);
        for(Eigen::DenseIndex col_id = 0; col_id < total_dim; ++col_id)
        {
          if(joint1_indexes_ee[col_id] != joint2_indexes_ee[col_id])
            colwise_loop_sparsity_patterns_ee[size++] = col_id;
        }
	colwise_loop_sparsity_patterns_ee.conservativeResize(size);
	
      }
      
      // Fill the sparsity pattern for each Row of the Cholesky decomposition (matrix U)
      static const Slice default_slice_value(1,1);
      static const SliceVector default_slice_vector(1,default_slice_value);

      rowise_sparsity_pattern.clear();
      rowise_sparsity_pattern.resize((size_t)num_total_constraints,default_slice_vector);
      row_id = 0; size_t ee_id = 0;
      for(typename RigidConstraintModelVector::const_iterator it = contact_models.begin();
          it != contact_models.end();
          ++it, ++ee_id)
      {
        const RigidConstraintModel & cmodel = *it;
        const BooleanVector & joint1_indexes_ee = joint1_indexes[ee_id];
        const Eigen::DenseIndex contact_dim = cmodel.size();

        for(Eigen::DenseIndex k = 0; k < contact_dim; ++k)
        {
          SliceVector & slice_vector = rowise_sparsity_pattern[(size_t)row_id];
          slice_vector.clear();
          slice_vector.push_back(Slice(row_id,num_total_constraints-row_id));
          
          bool previous_index_was_true = true;
          for(Eigen::DenseIndex joint1_indexes_ee_id = num_total_constraints;
              joint1_indexes_ee_id < total_dim;
              ++joint1_indexes_ee_id)
          {
            if(joint1_indexes_ee[joint1_indexes_ee_id])
            {
              if(previous_index_was_true) // no discontinuity
                slice_vector.back().size++;
              else // discontinuity; need to create a new slice
              {
                const Slice new_slice(joint1_indexes_ee_id,1);
                slice_vector.push_back(new_slice);
              }
            }

            previous_index_was_true = joint1_indexes_ee[joint1_indexes_ee_id];
          }

          row_id++;
        }
      }
      
      // Allocate memory
      D.resize(total_dim); Dinv.resize(total_dim);
      U.resize(total_dim,total_dim);
      U.setIdentity();
      DUt.resize(total_dim);
    }
    
    template<typename Scalar, int Options>
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    compute(const ModelTpl<S1,O1,JointCollectionTpl> & model,
            DataTpl<S1,O1,JointCollectionTpl> & data,
            const std::vector<RigidConstraintModelTpl<S1,O1>,ContactModelAllocator> & contact_models,
            std::vector<RigidConstraintDataTpl<S1,O1>,ContactDataAllocator> & contact_datas,
            const S1 mu)
    {
      typedef RigidConstraintModelTpl<S1,O1> RigidConstraintModel;
      typedef RigidConstraintDataTpl<S1,O1> RigidConstraintData;
      typedef MotionTpl<Scalar,Options> Motion;
      typedef SE3Tpl<Scalar,Options> SE3;
      assert(model.check(data) && "data is not consistent with model.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT((Eigen::DenseIndex)contact_models.size() == num_contacts,
                                     "The number of contacts inside contact_models and the one during allocation do not match.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT((Eigen::DenseIndex)contact_datas.size() == num_contacts,
                                     "The number of contacts inside contact_datas and the one during allocation do not match.");
      PINOCCHIO_UNUSED_VARIABLE(model);
      
      const Eigen::DenseIndex total_dim = size();
      const Eigen::DenseIndex total_constraints_dim = total_dim - nv;
      
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
      const typename Data::MatrixXs & M = data.M;
      
      const size_t num_ee = contact_models.size();
      
      // Update frame placements if needed
      for(size_t ee_id = 0; ee_id < num_ee; ++ee_id)
      {
        const RigidConstraintModel & cmodel = contact_models[ee_id];
        RigidConstraintData & cdata = contact_datas[ee_id];

        const typename Model::JointIndex joint1_id = cmodel.joint1_id;
        if(joint1_id > 0)
          cdata.oMc1 = data.oMi[joint1_id] * cmodel.joint1_placement;
        else
          cdata.oMc1 = cmodel.joint1_placement;

        const typename Model::JointIndex joint2_id = cmodel.joint2_id;
        if(joint2_id > 0)
          cdata.oMc2 = data.oMi[joint2_id] * cmodel.joint2_placement;
        else
          cdata.oMc2 = cmodel.joint2_placement;
        
        // Compute relative placement
        cdata.c1Mc2 = cdata.oMc1.actInv(cdata.oMc2);
      }

      // Core
//      Motion Jcol_motion;
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
        assert(check_expression_if_real<Scalar>(D[jj] != Scalar(0)) && "The diagonal element is equal to zero.");
        Dinv[jj] = Scalar(1)/D[jj];
        
        for(Eigen::DenseIndex _ii = parents_fromRow[jj]; _ii >= total_constraints_dim; _ii = parents_fromRow[_ii])
        {
          const Eigen::DenseIndex _i = _ii - total_constraints_dim;
          U(_ii,jj) = (M(_i,j) - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
        }
        
        // Constraint handling
        if(num_ee == 0)
          continue;

        Eigen::DenseIndex current_row = total_constraints_dim - 1;
        
        for(size_t k = 0; k < num_ee; ++k)
        {
          const size_t ee_id = num_ee - k - 1; // start from the last end effector

          const BooleanVector & joint1_indexes_ee = joint1_indexes[ee_id];
          const BooleanVector & joint2_indexes_ee = joint2_indexes[ee_id];
          const RigidConstraintModel & cmodel = contact_models[ee_id];
          const RigidConstraintData & cdata = contact_datas[ee_id];
          
          const Eigen::DenseIndex constraint_dim = cmodel.size();
	        const SE3 & oMc1 = cdata.oMc1;
          const SE3 & oMc2 = cdata.oMc2;
          const SE3 & c1Mc2 = cdata.c1Mc2;
   
          if(joint1_indexes_ee[jj] || joint2_indexes_ee[jj])
          {
            const int sign =
            joint1_indexes_ee[jj] != joint2_indexes_ee[jj]
            ? joint1_indexes_ee[jj] ? +1:-1
            : 0; // specific case for CONTACT_3D

            typedef typename Data::Matrix6x::ColXpr ColXpr;
            const ColXpr Jcol = data.J.col(j);
            const MotionRef<const ColXpr> Jcol_motion(Jcol);
            
            switch(cmodel.type)
            {
              case CONTACT_3D:
              {
                switch(cmodel.reference_frame)
                {
                  case WORLD:
                  {
                    for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                    {
                      const Eigen::DenseIndex _ii = current_row - _i;
                      U(_ii,jj) = (Jcol_motion.linear()[contact_dim<CONTACT_3D>::value-_i-1] * sign
                                - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                    }
                    break;
                  }
                  case LOCAL:
                  {
                    if(sign == 0)
                    {
                      const Motion Jcol_local1(oMc1.actInv(Jcol_motion)); // TODO: simplify computations
                      const Motion Jcol_local2(oMc2.actInv(Jcol_motion)); // TODO: simplify computations
                      const typename Motion::Vector3 Jdiff_linear = Jcol_local1.linear() - c1Mc2.rotation()*Jcol_local2.linear();
                      for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                      {
                        const Eigen::DenseIndex _ii = current_row - _i;
                        U(_ii,jj) = (Jdiff_linear[contact_dim<CONTACT_3D>::value-_i-1]
                                  - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                      }
                      break;
                    }
                    else if(sign == 1)
                    {
                      const Motion Jcol_local(oMc1.actInv(Jcol_motion));
                      for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                      {
                        const Eigen::DenseIndex _ii = current_row - _i;
                        U(_ii,jj) = (Jcol_local.linear()[contact_dim<CONTACT_3D>::value-_i-1] * sign
                                  - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                      }
                      break;
                    }
                    else // sign == -1
                    {
                      Motion Jcol_local(oMc2.actInv(Jcol_motion));
                      Jcol_local.linear() = c1Mc2.rotation()*Jcol_local.linear();
                      for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                      {
                        const Eigen::DenseIndex _ii = current_row - _i;
                        U(_ii,jj) = (Jcol_local.linear()[contact_dim<CONTACT_3D>::value-_i-1] * sign
                                  - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                      }
                      break;
                    }
                  }
                  case LOCAL_WORLD_ALIGNED:
                  {
                    if(sign == 0)
                    {
                      const typename Motion::Vector3 Jdiff_linear = (oMc2.translation() - oMc1.translation()).cross(Jcol_motion.angular());
                      for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                      {
                        const Eigen::DenseIndex _ii = current_row - _i;
                        U(_ii,jj) = (Jdiff_linear[contact_dim<CONTACT_3D>::value-_i-1]
                                  - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                      }
                      break;
                    }
                    else
                    {
                      typename Motion::Vector3 Jcol_local_world_aligned_linear(Jcol_motion.linear());
                      if(sign == 1)
                        Jcol_local_world_aligned_linear
                        -= oMc1.translation().cross(Jcol_motion.angular());
                      else
                        Jcol_local_world_aligned_linear
                        -= oMc2.translation().cross(Jcol_motion.angular());
                      for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_3D>::value; _i++)
                      {
                        const Eigen::DenseIndex _ii = current_row - _i;
                        U(_ii,jj) = (Jcol_local_world_aligned_linear[contact_dim<CONTACT_3D>::value-_i-1] * sign
                                     - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                      }
                      break;
                    }
                  }
                }
                break;
              }
                
              case CONTACT_6D:
              {
                assert(check_expression_if_real<Scalar>(sign != 0) && "sign should be equal to +1 or -1.");
                switch(cmodel.reference_frame)
                {
                  case WORLD:
                  {
                    for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_6D>::value; _i++)
                    {
                      const Eigen::DenseIndex _ii = current_row - _i;
                      U(_ii,jj) = (Jcol_motion.toVector()[contact_dim<CONTACT_6D>::value-_i-1] * sign
                                - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                    }
                    break;
                  }
                  case LOCAL:
                  {
                    const Motion Jcol_local(oMc1.actInv(Jcol_motion));
                    for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_6D>::value; _i++)
                    {
                      const Eigen::DenseIndex _ii = current_row - _i;
                      U(_ii,jj) = (Jcol_local.toVector()[contact_dim<CONTACT_6D>::value-_i-1] * sign
                                   - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                    }
                    break;
                  }
                  case LOCAL_WORLD_ALIGNED:
                  {
                    Motion Jcol_local_world_aligned(Jcol_motion);
                    Jcol_local_world_aligned.linear()
                    -= oMc1.translation().cross(Jcol_local_world_aligned.angular());
                    for(Eigen::DenseIndex _i = 0; _i < contact_dim<CONTACT_6D>::value; _i++)
                    {
                      const Eigen::DenseIndex _ii = current_row - _i;
                      U(_ii,jj) = (Jcol_local_world_aligned.toVector()[contact_dim<CONTACT_6D>::value-_i-1] * sign
                                - U.row(_ii).segment(jj+1,NVT).dot(DUt_partial)) * Dinv[jj];
                    }
                    break;
                  }
                }
                break;
              }
                
              default:
                assert(false && "must never happened");
                break;
            }

          }
          else if((joint1_indexes_ee[jj] == false) && (joint2_indexes_ee[jj] == false))
          {
            for(Eigen::DenseIndex _i = 0; _i < constraint_dim; _i++)
            {
              const Eigen::DenseIndex _ii = current_row - _i;
              U(_ii,jj) = -U.row(_ii).segment(jj+1,NVT).dot(DUt_partial) * Dinv[jj];
            }
          }
          current_row -= constraint_dim;
        }

      }

      // Upper left triangular part of U
      for(Eigen::DenseIndex j = total_constraints_dim-1; j>=0; --j)
      {
        const Eigen::DenseIndex slice_dim = total_dim - j - 1;
        typename Vector::SegmentReturnType DUt_partial = DUt.head(slice_dim);
        DUt_partial.noalias() = U.row(j).segment(j+1,slice_dim).transpose().cwiseProduct(D.segment(j+1,slice_dim));

        D[j] = -mu - U.row(j).segment(j+1,slice_dim).dot(DUt_partial);
        assert(check_expression_if_real<Scalar>(D[j] != Scalar(0)) && "The diagonal element is equal to zero.");
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
  
    template<typename Scalar, int Options>
    template<typename S1, int O1, template<typename,int> class JointCollectionTpl>
    ContactCholeskyDecompositionTpl<Scalar,Options>
    ContactCholeskyDecompositionTpl<Scalar,Options>::
    getMassMatrixChoeslkyDecomposition(const ModelTpl<S1,O1,JointCollectionTpl> & model) const
    {
      typedef ContactCholeskyDecompositionTpl<Scalar,Options> ReturnType;
      ReturnType res(model);
      
      res.D = D.tail(nv);
      res.Dinv = Dinv.tail(nv);
      res.U = U.bottomRightCorner(nv,nv);
      
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(mat.rows() == chol.size(),
                                         "The input matrix is of wrong size");
          
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(vec.size() == chol.size(),
                                         "The input vector is of wrong size");
          const Eigen::DenseIndex num_total_constraints = chol.size() - chol.nv;
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = 0; k < num_total_constraints; ++k)
          {
            const Eigen::DenseIndex slice_dim = chol.size() - k - 1;
            vec_[k] += chol.U.row(k).tail(slice_dim).dot(vec_.tail(slice_dim));
          }
          
          for(Eigen::DenseIndex k = num_total_constraints; k <= chol.size()-2; ++k)
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(mat.rows() == chol.size(),
                                         "The input matrix is of wrong size");
          
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(vec.size() == chol.size(),
                                         "The input vector is of wrong size");
          const Eigen::DenseIndex num_total_constraints = chol.constraintDim();
          
          for(Eigen::DenseIndex k = chol.size()-2; k >= num_total_constraints; --k)
            vec_.segment(k+1,chol.nv_subtree_fromRow[k]-1)
            += chol.U.row(k).segment(k+1,chol.nv_subtree_fromRow[k]-1).transpose()*vec_[k];
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = num_total_constraints-1; k >=0; --k)
          {
            const Eigen::DenseIndex slice_dim = chol.size() - k - 1;
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(mat.rows() == chol.size(),
                                         "The input matrix is of wrong size");
          
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(vec.size() == chol.size(),
                                         "The input vector is of wrong size");
          
          const Eigen::DenseIndex num_total_constraints = chol.size() - chol.nv;
          for(Eigen::DenseIndex k = chol.size()-2; k >= num_total_constraints; --k)
            vec_[k] -= chol.U.row(k).segment(k+1,chol.nv_subtree_fromRow[k]-1).dot(vec_.segment(k+1,chol.nv_subtree_fromRow[k]-1));
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = num_total_constraints-1; k >=0; --k)
          {
            const Eigen::DenseIndex slice_dim = chol.size() - k - 1;
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(mat.rows() == chol.size(),
                                         "The input matrix is of wrong size");
          
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
          
          PINOCCHIO_CHECK_INPUT_ARGUMENT(vec.size() == chol.size(),
                                         "The input vector is of wrong size");
          const Eigen::DenseIndex num_total_constraints = chol.constraintDim();
          
          // TODO: exploit the Sparsity pattern of the first rows of U
          for(Eigen::DenseIndex k = 0; k < num_total_constraints; ++k)
          {
            const Eigen::DenseIndex slice_dim = chol.size() - k - 1;
            vec_.tail(slice_dim) -= chol.U.row(k).tail(slice_dim).transpose() * vec_[k];
          }
          
          for(Eigen::DenseIndex k = num_total_constraints; k <= chol.size()-2; ++k)
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
      Matrix res(size(),size());
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
    
    template<typename Scalar, int Options>
    typename ContactCholeskyDecompositionTpl<Scalar,Options>::Matrix
    ContactCholeskyDecompositionTpl<Scalar,Options>::inverse() const
    {
      Matrix res(size(),size());
      inverse(res);
      return res;
    }
    
    namespace details
    {
  
      template<typename Scalar, int Options, typename VectorLike>
      EIGEN_DONT_INLINE
      VectorLike & inverseAlgo(const ContactCholeskyDecompositionTpl<Scalar,Options> & chol,
                               const Eigen::DenseIndex col,
                               const Eigen::MatrixBase<VectorLike> & vec)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike);
        
        typedef ContactCholeskyDecompositionTpl<Scalar,Options> ContactCholeskyDecomposition;
        typedef typename ContactCholeskyDecomposition::RowMatrix RowMatrix;
        
        const Eigen::DenseIndex & chol_dim = chol.size();
        PINOCCHIO_CHECK_INPUT_ARGUMENT(col < chol_dim && col >= 0);
        PINOCCHIO_CHECK_INPUT_ARGUMENT(vec.size() == chol_dim);
        
        const typename ContactCholeskyDecomposition::IndexVector & nvt = chol.nv_subtree_fromRow;
        VectorLike & vec_ = PINOCCHIO_EIGEN_CONST_CAST(VectorLike,vec);
        
        const Eigen::DenseIndex last_col = std::min(col-1,chol_dim-2); // You can start from nv-2 (no child in nv-1)
        vec_[col] = Scalar(1);
        vec_.tail(chol_dim - col - 1).setZero();

        // TODO: exploit the sparsity pattern of the first rows of U
        for(Eigen::DenseIndex k = last_col; k >= 0; --k)
        {
          const Eigen::DenseIndex nvt_max = std::min(col-k,nvt[k]-1);
          const typename RowMatrix::ConstRowXpr U_row = chol.U.row(k);
          vec_[k] = -U_row.segment(k+1,nvt_max).dot(vec_.segment(k+1,nvt_max));
//          if(k >= chol_constraint_dim)
//          {
//            vec_[k] = -U_row.segment(k+1,nvt_max).dot(vec_.segment(k+1,nvt_max));
//          }
//          else
//          {
//            typedef typename ContactCholeskyDecomposition::SliceVector SliceVector;
//            typedef typename ContactCholeskyDecomposition::Slice Slice;
//            const SliceVector & slice_vector = chol.rowise_sparsity_pattern[(size_t)k];
//
//            const Slice & slice_0 = slice_vector[0];
//            assert(slice_0.first_index == k);
//            Eigen::DenseIndex last_index1 = slice_0.first_index + slice_0.size;
//            const Eigen::DenseIndex last_index2 = k + nvt_max;
//            Eigen::DenseIndex slice_dim = std::min(last_index1,last_index2) - k;
//            vec_[k] = -U_row.segment(slice_0.first_index+1,slice_dim-1).dot(vec_.segment(slice_0.first_index+1,slice_dim-1));
//
//            typename SliceVector::const_iterator slice_it = slice_vector.begin()++;
//            for(;slice_it != slice_vector.end(); ++slice_it)
//            {
//              const Slice & slice = *slice_it;
//              last_index1 = slice.first_index + slice.size;
//              slice_dim = std::min(last_index1,last_index2+1) - slice.first_index;
//              if(slice_dim <= 0) break;
//
//              vec_[k] -= U_row.segment(slice.first_index,slice_dim).dot(vec_.segment(slice.first_index,slice_dim));
//            }
//          }
        }

        vec_.head(col+1).array() *= chol.Dinv.head(col+1).array();

        for(Eigen::DenseIndex k = 0; k < col+1; ++k) // You can stop one step before nv.
        {
          const Eigen::DenseIndex nvt_max = nvt[k]-1;
          vec_.segment(k+1,nvt_max) -= chol.U.row(k).segment(k+1,nvt_max).transpose() * vec_[k];
        }
        
        return vec_;
      }
    } // namespace details
    
    template<typename Scalar, int Options>
    template<typename MatrixType>
    void ContactCholeskyDecompositionTpl<Scalar,Options>::
    inverse(const Eigen::MatrixBase<MatrixType> & res) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(res.rows() == size());
      PINOCCHIO_CHECK_INPUT_ARGUMENT(res.cols() == size());
      
      MatrixType & res_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType,res);
      
      for(Eigen::DenseIndex col_id = 0; col_id < size(); ++col_id)
        details::inverseAlgo(*this,col_id,res_.col(col_id));
      
      res_.template triangularView<Eigen::StrictlyLower>()
      = res_.transpose().template triangularView<Eigen::StrictlyLower>();
    }
    
  } // namespace cholesky
}

#endif // ifndef __pinocchio_algorithm_contact_cholesky_hxx__
