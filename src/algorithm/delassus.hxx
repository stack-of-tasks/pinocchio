//
// Copyright (c) 2020-2021 INRIA CNRS
//

#ifndef __pinocchio_algorithm_contact_delassus_hxx__
#define __pinocchio_algorithm_contact_delassus_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/model.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  struct ComputeOSIMForwardStep
  : public fusion::JointUnaryVisitorBase< ComputeOSIMForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex i = jmodel.id();
      jmodel.calc(jdata.derived(),q.derived());
      
      const JointIndex parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      if(parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      data.oYaba[i] = data.oYcrb[i].matrix();
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ComputeOSIMBackwardStep
  : public fusion::JointUnaryVisitorBase< ComputeOSIMBackwardStep<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Matrix6 Matrix6;
      typedef typename Data::Matrix6x Matrix6x;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent  = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.oYaba[i];
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);
      
      jdata.U().noalias() = Ia * Jcols;
      jdata.StU().noalias() = Jcols.transpose() * jdata.U();
      
      // Account for the rotor inertia contribution
      jdata.StU().diagonal() += jmodel.jointVelocitySelector(model.armature);
      
      internal::PerformStYSInversion<Scalar>::run(jdata.StU(),jdata.Dinv());

      jdata.UDinv().noalias() = Jcols * jdata.Dinv().transpose();
      data.oK[i].noalias() = jdata.UDinv() * Jcols.transpose();
      data.oL[i].noalias() = -jdata.UDinv() * jdata.U().transpose();
      data.oL[i] += Matrix6::Identity();
      
      if(parent > 0)
      {
        jdata.UDinv().noalias() = jdata.U() * jdata.Dinv();
        data.oYaba[parent] += Ia;
        data.oYaba[parent].noalias() -= jdata.UDinv() * jdata.U().transpose();
      }
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, class ModelAllocator, class DataAllocator, typename MatrixType>
  void computeDelassusMatrix(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             DataTpl<Scalar,Options,JointCollectionTpl> & data,
                             const Eigen::MatrixBase<ConfigVectorType> & q,
                             const std::vector<RigidConstraintModelTpl<Scalar,Options>,ModelAllocator> & contact_models,
                             std::vector<RigidConstraintDataTpl<Scalar,Options>,DataAllocator> & contact_data,
                             const Eigen::MatrixBase<MatrixType> & delassus_,
                             const Scalar mu)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq,
                                  "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(check_expression_if_real<Scalar>(mu >= Scalar(0)),
                                   "mu has to be positive");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(contact_models.size(), contact_data.size(),
                                  "contact models and data size are not the same");
    
    MatrixType & delassus = delassus_.const_cast_derived();
    const size_t constraint_total_size = getTotalConstraintSize(contact_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(delassus_.rows(),(Eigen::DenseIndex)constraint_total_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(delassus_.cols(),(Eigen::DenseIndex)constraint_total_size);
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::SE3 SE3;
    typedef typename Model::IndexVector IndexVector;
    typedef RigidConstraintModelTpl<Scalar,Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar,Options> RigidConstraintData;
    
    typedef ComputeOSIMForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],typename Pass1::ArgsType(model,data,q.derived()));
    }
    
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];
      
      const JointIndex joint1_id = cmodel.joint1_id;

      // Compute relative placement between the joint and the contact frame
      SE3 & oMc = cdata.oMc1;
      oMc = data.oMi[joint1_id] * cmodel.joint1_placement; // contact placement
      
      typedef typename Data::Inertia Inertia;
      typedef typename Inertia::Symmetric3 Symmetric3;
      
      // Add contact inertia to the joint articulated inertia
      Symmetric3 S(Symmetric3::Zero());
      if(cmodel.type == CONTACT_6D)
        S.setDiagonal(Symmetric3::Vector3::Constant(mu));
      
      const Inertia contact_inertia(mu,oMc.translation(),S);
      data.oYaba[joint1_id] += contact_inertia.matrix();
    }
    
    typedef ComputeOSIMBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],typename Pass2::ArgsType(model,data));
    }
    
    Eigen::DenseIndex current_row_id = 0;
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      typedef typename RigidConstraintData::VectorOfMatrix6 VectorOfMatrix6;
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];
      
      const JointIndex joint1_id = cmodel.joint1_id;
      const SE3 & oMc1 = cdata.oMc1;
      const IndexVector & support1 = model.supports[joint1_id];
      
      {
        VectorOfMatrix6 & propagators = cdata.extended_motion_propagators_joint1;
        VectorOfMatrix6 & lambdas = cdata.lambdas_joint1;
        switch(cmodel.type)
        {
          case CONTACT_3D:
          {
            propagators.back().template leftCols<3>().template topRows<3>() = oMc1.rotation().transpose();
            propagators.back().template leftCols<3>().template bottomRows<3>().setZero();
            for(size_t j = support1.size()-1; j > 1; --j)
            {
              lambdas[j].template leftCols<3>().noalias() = data.oK[(size_t)support1[j]] * propagators[j].template topRows<3>().transpose();
              propagators[j-1].template topRows<3>().noalias() = propagators[j].template topRows<3>() * data.oL[(size_t)support1[j]];
            }
            lambdas[1].template leftCols<3>().noalias() = data.oK[(size_t)support1[1]] * propagators[1].template topRows<3>().transpose();

            for(size_t j = 2; j < support1.size(); ++j)
            {
              lambdas[j].template leftCols<3>().noalias() += data.oL[(size_t)support1[j]] * lambdas[j-1].template leftCols<3>();
            }
            break;
          }
          case CONTACT_6D:
          {
            oMc1.toActionMatrixInverse(propagators.back());
            for(size_t j = support1.size()-1; j > 1; --j)
            {
              lambdas[j].noalias() = data.oK[(size_t)support1[j]] * propagators[j].transpose();
              propagators[j-1].noalias() = propagators[j] * data.oL[(size_t)support1[j]];
            }
            lambdas[1].noalias() = data.oK[(size_t)support1[1]] * propagators[1].transpose();

            for(size_t j = 2; j < support1.size(); ++j)
            {
              lambdas[j].noalias() += data.oL[(size_t)support1[j]] * lambdas[j-1];
            }
            break;
          }
          default:
          {
            assert(false && "must never happened");
            break;
          }
        }
      }
      
      // Fill the delassus matrix block-wise
      {
        const int size = cmodel.size();
        
        const VectorOfMatrix6 & propagators = cdata.extended_motion_propagators_joint1;
        const VectorOfMatrix6 & lambdas = cdata.lambdas_joint1;
        
        Eigen::DenseIndex current_row_id_other = 0;
        for(size_t i = 0; i < k; ++i)
        {
          const RigidConstraintModel & cmodel_other = contact_models[i];
          const RigidConstraintData & cdata_other  = contact_data[i];
          const int size_other = cmodel_other.size();
          const IndexVector & support1_other = model.supports[cmodel_other.joint1_id];
          
          const VectorOfMatrix6 & propagators_other = cdata_other.extended_motion_propagators_joint1;
          
          size_t id_in_support1, id_in_support1_other;
          findCommonAncestor(model,joint1_id,cmodel_other.joint1_id,
                             id_in_support1,id_in_support1_other);
          
//          std::cout << "k: " << k << std::endl;
//          std::cout << "i: " << i << std::endl;
//          std::cout << "joint1_id: " << joint1_id << std::endl;
//          std::cout << "joint1_id_other: " << cmodel_other.joint1_id << std::endl;
//          std::cout << "support1[id_in_support1]: " << support1[id_in_support1] << std::endl;
//          std::cout << "support1_other[id_in_support1_other]: " << support1_other[id_in_support1_other] << std::endl;
//          std::cout << "propagators_other[ support1_other[id_in_support1_other] ]" << std::endl;
//          std::cout << propagators_other[ support1_other[id_in_support1_other] ] << std::endl;
          
//          std::cout << "lambdas[ support1[id_in_support1] ]" << std::endl;
//          std::cout << lambdas[ support1[id_in_support1] ] << std::endl;
          delassus.block(current_row_id_other,current_row_id,size_other,size).noalias()
          = propagators_other[id_in_support1_other] * lambdas[id_in_support1];
          
          current_row_id_other += size_other;
        }
        
        assert(current_row_id_other == current_row_id && "current row indexes do not match.");
        delassus.block(current_row_id,current_row_id,size,size).noalias() = propagators.back() * lambdas.back();
        current_row_id += size;
      }
      
      assert(current_row_id == delassus.rows() && "current row indexes do not the number of rows in the Delassus matrix.");
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, class ModelAllocator, class DataAllocator, typename MatrixType>
  void computeDampedDelassusMatrixInverse(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                          DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                          const Eigen::MatrixBase<ConfigVectorType> & q,
                                          const std::vector<RigidConstraintModelTpl<Scalar,Options>,ModelAllocator> & contact_models,
                                          std::vector<RigidConstraintDataTpl<Scalar,Options>,DataAllocator> & contact_data,
                                          const Eigen::MatrixBase<MatrixType> & damped_delassus_inverse_,
                                          const Scalar mu,
                                          const bool scaled)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(check_expression_if_real<Scalar>(mu >= Eigen::NumTraits<Scalar>::dummy_precision()),"mu is too small.");
    
    const Scalar mu_inv = Scalar(1)/mu;
    MatrixType & damped_delassus_inverse = damped_delassus_inverse_.const_cast_derived();
    computeDelassusMatrix(model,data,q,contact_models,contact_data,damped_delassus_inverse,mu_inv);
    
    const Scalar mu_inv_square = mu_inv * mu_inv;
    assert(mu_inv_square != std::numeric_limits<Scalar>::infinity() && "mu_inv**2 is equal to infinity.");
    damped_delassus_inverse *= -mu_inv;
    damped_delassus_inverse.diagonal().array() += Scalar(1);
    if(not scaled)
      damped_delassus_inverse *= mu_inv;
  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_delassus_hxx__
