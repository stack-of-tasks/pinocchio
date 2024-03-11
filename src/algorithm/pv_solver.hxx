//
// Copyright (c) 2016-2021 CNRS INRIA
//

#ifndef __pinocchio_algorithm_pv_hxx__
#define __pinocchio_algorithm_pv_hxx__

#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"

#include "Eigen/Cholesky"
#include "Eigen/Dense"
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>

/// @cond DEV

namespace pinocchio
{

  // Ask Justin where to initialize pv_settings, or whether it should be combined with proximal settings

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void initPvSolver(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                  const std::vector<RigidConstraintModelTpl<Scalar,Options>,Allocator> & contact_models)
  {

    // Allocate memory for the backward propagation of LA, KA and lA
    typedef typename Model::JointIndex JointIndex;
    // typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;

    std::fill(data.constraints_supported.begin(), data.constraints_supported.end(), 0);
    // Getting the constrained links
    for(std::size_t i=0;i<contact_models.size();++i)
    {
      const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      switch (contact_model.reference_frame)
      {
        case LOCAL:
          if (contact_model.type == CONTACT_6D)
            data.constraints_supported[joint_id] += 6;
          else
            if (contact_model.type == CONTACT_3D)
              data.constraints_supported[joint_id] += 3;
          break;
        case WORLD:
          assert(false && "WORLD not implemented");
          break;
        case LOCAL_WORLD_ALIGNED:
          assert(false && "LOCAL_WORLD_ALIGNED not implemented");
          break;
        default:
          assert(false && "Must never happen");
          break;
      }
    }
    // Running backprop to get the count of constraints
    for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
    {
      const JointIndex & parent = model.parents[i];
      data.par_cons_ind[i] = data.constraints_supported[parent];
      data.constraints_supported[parent] += data.constraints_supported[i];
    }

    // Allocating memory for LA, KA and lA
    // size_t constraint_counter = 0;
    for (JointIndex i=0;i<(JointIndex)model.njoints;++i)
    {
      data.lA[i] = Data::VectorXs::Zero(data.constraints_supported[i]);
      data.lambdaA[i] = Data::VectorXs::Zero(data.constraints_supported[i]);

      // const JointIndex & parent = model.parents[i];
      // if (parent > 0)
      // {
      //   if (data.constraints_supported[parent] == data.constraints_supported[i])
      //     data.constraint_ind[i] = data.constraint_ind[parent];
      //   else
      //   {
      //     if (constraint_counter != data.constraint_ind[parent])
      //       data.constraint_ind[i] = constraint_counter;
      //     constraint_counter += data.constraints_supported[parent] - data.constraints_supported[i];
      //   }
      //   data.par_cons_ind[i] = data.constraint_ind[i] - data.constraint_ind[parent];
        
      // }
      
      data.LA[i] = Data::MatrixXs::Zero(data.constraints_supported[i], data.constraints_supported[i]);
      data.KA_temp[i] = Data::MatrixXs::Zero(6, data.constraints_supported[i]);
      data.KAS[i] = Data::MatrixXs::Zero( data.joints[i].S().matrix().cols(), data.constraints_supported[i]);
    }

    // For Local, append the constraint matrices in K
    std::vector<size_t> condim_counter(model.njoints, 0);
    for(std::size_t i=0;i<contact_models.size();++i)
    {
      const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      const auto & oMc = contact_model.joint1_placement;
      if (contact_model.type == CONTACT_6D)
      {
        // data.KA_temp[joint_id].setIdentity();
        data.KA_temp[joint_id].middleCols(condim_counter[joint_id],6) = oMc.toActionMatrixInverse().transpose();
        condim_counter[joint_id] += 6;
      }
      else if (contact_model.type == CONTACT_3D)
      {
        data.KA_temp[joint_id].middleCols(condim_counter[joint_id],3) = oMc.toActionMatrixInverse().transpose().leftCols(3);
        condim_counter[joint_id] += 3;
      }
      // std::cout << "KA_temp = " << data.KA_temp[joint_id] << "\n";
    } 

    data.lambda_c_prox.resize(data.constraints_supported[0]);

    data.lambda_c.setZero();
    data.lambda_c_prox.setZero();
    data.osim_llt = Eigen::LLT<Data::MatrixXs>(data.constraints_supported[0]);
    for (size_t j : model.children[1])
    {
      data.fb_osim_llt.push_back(Eigen::LLT<Data::MatrixXs>(data.constraints_supported[j]));
    }
    
    // for early elimination
    for (JointIndex i=0;i<(JointIndex)model.njoints;++i)
    {
      data.w[i] = Data::VectorXs::Zero(data.constraints_supported[i]);
      data.KAopt[i] = Data::VectorXs::Zero(data.constraints_supported[i]);
    }


  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  struct PvForwardStep1
  : public fusion::JointUnaryVisitorBase< PvForwardStep1<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex i = jmodel.id();
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
      
      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      
      data.v[i] = jdata.v();
      if (parent>0)
      {
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      // data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      data.a_gf[i].linear() = data.liMi[i].rotation().transpose() * data.a_gf[parent].linear();

      data.a_bias[i] = jdata.c() + (data.v[i] ^ jdata.v());
      
      data.Yaba[i] = model.inertias[i].matrix();
      data.h[i] = model.inertias[i] * data.v[i];
      data.f[i] = data.v[i].cross(data.h[i]); // -f_ext
      
    }
    
  };

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct cAbaBackwardStep
  : public fusion::JointUnaryVisitorBase< cAbaBackwardStep<Scalar,Options,JointCollectionTpl> >
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
      typedef typename Data::Force Force;
      
      Force bias_and_force = Force::Zero();
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.Yaba[i];

      bias_and_force.toVector() -= data.Yaba[i]*data.a_bias[i].toVector();

      jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose()*data.f[i];
      jmodel.calc_aba(jdata.derived(),
                      jmodel.jointVelocitySelector(model.armature),
                      Ia, parent > 0);
      
      Force & pa = data.f[i];

      if (parent > 0)
      {
        pa.toVector() += Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        // pa.toVector() += Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.Yaba[parent] += impl::internal::SE3actOn<Scalar>::run(data.liMi[i], Ia);
        data.f[parent] += data.liMi[i].act(pa);
      }
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct PvRegBackwardStep
  : public fusion::JointUnaryVisitorBase< PvRegBackwardStep<Scalar,Options,JointCollectionTpl> >
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
      typedef typename Data::Force Force;
      
      Force bias_and_force = Force::Zero();
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.Yaba[i];

      bias_and_force.toVector() -= data.Yaba[i]*data.a_bias[i].toVector();

      jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose()*data.f[i];
      jmodel.calc_aba(jdata.derived(),
                      jmodel.jointVelocitySelector(model.armature),
                      Ia, parent > 0);
      
      Force & pa = data.f[i];

      if (parent > 0)
      {
        pa.toVector() += Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.Yaba[parent] += impl::internal::SE3actOn<Scalar>::run(data.liMi[i], Ia);
        data.f[parent] += data.liMi[i].act(pa);
      }

      if (data.constraints_supported[i] > 0)
      {
        // std::cout << data.KA_temp[i].eval() << std::endl; //TODO; debug
        data.KAS[i].noalias() = jdata.S().transpose()*data.KA_temp[i]; //(data.KA_temp[i]*jdata.S().matrix()); //.eval();

        // int n_rows = data.KA_temp[i].rows();
        // data.KA_temp[parent].middleRows(data.par_cons_ind[i], n_rows) = 
        //   (data.KA_temp[i] - data.KAS[i]*(jdata.UDinv().transpose()))*data.liMi[i].toActionMatrixInverse_impl();

        // data.LA[parent].block(data.par_cons_ind[i], data.par_cons_ind[i], n_rows, n_rows) = data.LA[i] +
        //     data.KAS[i]*jdata.Dinv()*data.KAS[i].transpose();

        // auto a_bf_js = (jdata.Dinv()* (jdata.S().transpose()*bias_and_force ) + jdata.Dinv() * (jmodel.jointVelocitySelector(data.u))).eval();
        // data.lA[parent].segment(data.par_cons_ind[i], n_rows) += data.lA[i] + 
        //   data.KA_temp[i]*(jdata.S().matrix()*a_bf_js + data.a_bias[i].toVector());
        // Propagate KA backwards
      }
        for (int ind = 0; ind < data.constraints_supported[i]; ind++)
        {
          Force za = Force(data.KA_temp[i].col(ind));
          // za.toVector() += data.KA_temp[i].col(ind);
          za.toVector().noalias() -= (jdata.UDinv() * (data.KAS[i].col(ind))); //data.KA[i][ind].toVector() - 
          //   jdata.UDinv() * jdata.S().transpose()*data.KA[i][ind];
          data.KA_temp[parent].col(data.par_cons_ind[i] + ind).noalias() = data.liMi[i].act(za).toVector();

        }

        //Propagate LA backwards, we only care about tril because symmetric
        for (int ind = 0; ind < data.constraints_supported[i]; ind++)
        {
          // auto zb = data.KAS[i].col(ind); //(jdata.S().transpose()*data.KA_temp[i].col(ind)).eval();
          auto zc = (jdata.Dinv()*data.KAS[i].col(ind)).eval();
          for (int ind2 = ind; ind2 < data.constraints_supported[i]; ind2++)
          {
            // auto zd = ((jdata.S().transpose()*data.KA_temp[i].col(ind2)).transpose()).eval();
            // auto ze = (data.KAS[i].col(ind2).dot(zc));
            data.LA[parent](data.par_cons_ind[i] + ind2, data.par_cons_ind[i] + ind) = data.LA[i](ind2, ind) + 
              (data.KAS[i].col(ind2).dot(zc));
            // data.LA[0].coeffRef(data.constraint_ind[i] + ind, data.constraint_ind[i] + ind2)  += ze.coeff(0,0);
          }
        }

        // Propagate lA backwards
        if (data.constraints_supported[i] > 0)
        {          
          auto a_bf_js = (jdata.Dinv()* (jdata.S().transpose()*bias_and_force + jmodel.jointVelocitySelector(data.u))).eval();
          const Motion a_bf =  jdata.S()*a_bf_js;
          const Motion  a_bf_motion = a_bf + data.a_bias[i];
          for (int ind = 0; ind < data.constraints_supported[i]; ind++)
          {
            data.lA[parent](data.par_cons_ind[i] + ind) =  data.lA[i](ind) + 
              (data.KA_temp[i].col(ind).dot(a_bf_motion.toVector()));
          }
        }
      

    }
    
  };
  
    
  // template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  // struct PvBackwardStep
  // : public fusion::JointUnaryVisitorBase< PvBackwardStep<Scalar,Options,JointCollectionTpl> >
  // {
  //   typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
  //   typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
  //   typedef boost::fusion::vector<const Model &,
  //   Data &> ArgsType;
    
  //   template<typename JointModel>
  //   static void algo(const JointModelBase<JointModel> & jmodel,
  //                    JointDataBase<typename JointModel::JointDataDerived> & jdata,
  //                    const Model & model,
  //                    Data & data)
  //   {

  //     bool early_full = data.pv_settings.use_early;
  //     const double mu = data.pv_settings.mu;
  //     bool prox = mu > 0; //TODO: unhardcode these 
  //     typedef typename Model::JointIndex JointIndex;
  //     typedef typename Data::Inertia Inertia;
  //     typedef typename Data::Force Force;
      
  //     Force bias_and_force = Force::Zero();
      
  //     const JointIndex i = jmodel.id();
  //     const JointIndex parent = model.parents[i];
  //     typename Inertia::Matrix6 & Ia = data.Yaba[i];

      

  //     if (prox && early_full && data.constraints_supported[i] > 0)
  //     {
  //       // data.LA[i].setIdentity();
  //       // data.LA[i].noalias() = data.LA[i]*(1/mu);

  //       data.Yaba[i].matrix().noalias() += data.KA_temp[i]*(1/mu)*data.KA_temp[i].transpose();
  //       // Force constraint_force = Force::Zero();
  //       // constraint_force.toVector().noalias() += data.KA_temp[i]*data.LA[i]*data.lA[i];

  //       // jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose()*constraint_force;
  //       data.f[i].toVector().noalias() += data.KA_temp[i]*(1/mu)*data.lA[i];

  //       data.constraints_supported[parent] = 0;
        
  //     }

  //     bias_and_force.toVector() -= data.Yaba[i]*data.a_bias[i].toVector();

  //     jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose()*data.f[i];
  //     jmodel.calc_aba(jdata.derived(),
  //                     jmodel.jointVelocitySelector(model.armature),
  //                     Ia, parent > 0);
      
  //     Force & pa = data.f[i];

  //     if (parent > 0)
  //     {
  //       pa.toVector() += Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
  //     }

  //     if (data.constraints_supported[i] > 0 && !prox)
  //     {
  //       // std::cout << data.KA_temp[i].eval() << std::endl; //TODO; debug
  //       data.KAS[i] = jdata.S().transpose()*data.KA_temp[i].leftCols(data.constraints_supported[i]); //(data.KA_temp[i]*jdata.S().matrix()); //.eval();

  //       // int n_rows = data.KA_temp[i].rows();
  //       // data.KA_temp[parent].middleRows(data.par_cons_ind[i], n_rows) = 
  //       //   (data.KA_temp[i] - data.KAS[i]*(jdata.UDinv().transpose()))*data.liMi[i].toActionMatrixInverse_impl();

  //       // data.LA[parent].block(data.par_cons_ind[i], data.par_cons_ind[i], n_rows, n_rows) = data.LA[i] +
  //       //     data.KAS[i]*jdata.Dinv()*data.KAS[i].transpose();

  //       // auto a_bf_js = (jdata.Dinv()* (jdata.S().transpose()*bias_and_force ) + jdata.Dinv() * (jmodel.jointVelocitySelector(data.u))).eval();
  //       // data.lA[parent].segment(data.par_cons_ind[i], n_rows) += data.lA[i] + 
  //       //   data.KA_temp[i]*(jdata.S().matrix()*a_bf_js + data.a_bias[i].toVector());
  //     }
      
  //     if (early_full && !prox && data.constraints_supported[i] > 0 && data.KAS[i].rows() == 1 )
  //     {
  //       Scalar KAS_sum_sqr = data.KAS[i].row(0).dot(data.KAS[i].row(0));
  //       data.w[i] = data.KAS[i].transpose()/std::sqrt(KAS_sum_sqr);
  //       data.sigma[i] = jdata.Dinv().matrix()(0,0)*KAS_sum_sqr;

  //       // Find index of the maximum element in KAS
  //       // size_t data.svd_max_ind[i] = 0;
  //       // If pivoting  is allowed
  //       Scalar absMaxCoeff = data.w[i].array().abs().matrix().maxCoeff(&data.svd_max_ind[i]);

  //       // TODO remove, only for debug
  //       // max_ind = 0;
  //       // absMaxCoeff = std::abs(data.w[i](0));
        

  //       // for (int k = 1; k < data.KAS[i].cols(); k++)
  //       // {
  //       //   if (data.KAS[i].coeff(0,k) > max_element)
  //       //   {
  //       //     max_element = data.KAS[i].coeff(0,k);
  //       //     data.svd_max_ind[i] = k;
  //       //   }
  //       // }
  //       data.w[i].coeffRef(data.svd_max_ind[i]) += data.w[i].coeff(data.svd_max_ind[i])/absMaxCoeff;
  //       Scalar w_sum_sqr = data.w[i].dot(data.w[i]);

  //       auto w_normalized = (data.w[i]/w_sum_sqr).eval(); // Data::VectorXs not working
        

  //       // Propagate lA backwards       
  //       auto a_bf_js = (jdata.Dinv()* (jdata.S().transpose()*bias_and_force ) + jdata.Dinv() * (jmodel.jointVelocitySelector(data.u))).eval();
  //       Motion a_bf =  jdata.S()*a_bf_js;
  //       Eigen::Vector<double, 6> a_bf_motion = (a_bf + data.a_bias[i]).toVector();
  //       //Debug remove blocks below
  //         // std::cout << "Dinv = " << jdata.Dinv().matrix().eval() << std::endl; 
  //         // std::cout << "data.u = " << data.u.matrix() << std::endl;
  //       for (int ind = 0; ind < data.constraints_supported[i]; ind++)
  //       {
  //         data.lA[parent](ind) =  data.lA[i](ind) + 
  //           (data.KA_temp[i].col(ind).dot(a_bf_motion)); // todo: add par_cons_ind
  //       }


  //       data.lA[parent].head(data.constraints_supported[i]) -= 
  //         2*data.w[i].dot(data.lA[parent].head(data.constraints_supported[i]))*w_normalized;

  //       // data.KA_temp[parent] = data.KA_temp[i];
  //       for (int ind = 0; ind < data.constraints_supported[i]; ind++)
  //       {
  //         data.KA_temp[parent].col(ind) = data.KA_temp[i].col(ind); //TODO: comment and chekc if the line above the loop is faster
  //         data.KA_temp[parent].col(ind).noalias() -= (jdata.UDinv() * (data.KAS[i].col(ind)));
  //       }

  //       auto KAw = (data.KA_temp[parent].leftCols(data.constraints_supported[i])*data.w[i]).eval();

  //       int par_counter = 0;
  //       for (int ind = 0; ind < data.constraints_supported[i]; ind++)
  //       {
  //         if (ind == data.svd_max_ind[i])
  //         {
  //           data.KAopt[i] = data.KA_temp[parent].col(ind) - 2*w_normalized[ind]*KAw;
  //           data.lAopt[i] = data.lA[parent](ind);
  //         }
  //         else
  //         {
  //           data.KA_temp[parent].col(par_counter) = data.liMi[i].act(
  //             Force(data.KA_temp[parent].col(ind) - 2*w_normalized[ind]*KAw)).toVector();
  //           data.lA[parent](par_counter) = data.lA[parent](ind);
  //           par_counter++;
  //         }
  //       }

  //       // Updating inertia and pA
  //       Ia.noalias() += data.KAopt[i]*(data.KAopt[i].transpose()/data.sigma[i]);
  //       pa.toVector().noalias() += (data.lAopt[i]/data.sigma[i])*data.KAopt[i];
  //       // }
  //       data.constraints_supported[parent] = data.constraints_supported[i] - 1; // TODO: fix for constraint accumulation

  //     }
  //     else if (data.constraints_supported[i] > 0)
  //     {
  //       // Propagate KA backwards
  //       for (int ind = 0; ind < data.constraints_supported[i]; ind++)
  //       {
  //         Force za = Force(data.KA_temp[i].col(ind));
  //         // za.toVector() += data.KA_temp[i].col(ind);
  //         za.toVector().noalias() -= (jdata.UDinv() * (data.KAS[i].col(ind))); //data.KA[i][ind].toVector() - 
  //         //   jdata.UDinv() * jdata.S().transpose()*data.KA[i][ind];
  //         data.KA_temp[parent].col(data.par_cons_ind[i] + ind).noalias() = data.liMi[i].act(za).toVector();

  //       }

  //       //Propagate LA backwards, we only care about tril because symmetric
  //       for (int ind = 0; ind < data.constraints_supported[i]; ind++)
  //       {
  //         // auto zb = data.KAS[i].col(ind); //(jdata.S().transpose()*data.KA_temp[i].col(ind)).eval();
  //         auto zc = (jdata.Dinv()*data.KAS[i].col(ind)).eval();
  //         for (int ind2 = ind; ind2 < data.constraints_supported[i]; ind2++)
  //         {
  //           // auto zd = ((jdata.S().transpose()*data.KA_temp[i].col(ind2)).transpose()).eval();
  //           // auto ze = (data.KAS[i].col(ind2).dot(zc));
  //           data.LA[parent](data.par_cons_ind[i] + ind2, data.par_cons_ind[i] + ind) = data.LA[i](ind2, ind) + 
  //             (data.KAS[i].col(ind2).dot(zc));
  //           // data.LA[0].coeffRef(data.constraint_ind[i] + ind, data.constraint_ind[i] + ind2)  += ze.coeff(0,0);
  //         }
  //       }

  //       // Propagate lA backwards
  //       if (data.constraints_supported[i] > 0)
  //       {          
  //         auto a_bf_js = (jdata.Dinv()* (jdata.S().transpose()*bias_and_force + jmodel.jointVelocitySelector(data.u))).eval();
  //         const Motion a_bf =  jdata.S()*a_bf_js;
  //         const Motion  a_bf_motion = a_bf + data.a_bias[i];
  //         for (int ind = 0; ind < data.constraints_supported[i]; ind++)
  //         {
  //           data.lA[parent](data.par_cons_ind[i] + ind) =  data.lA[i](ind) + 
  //             (data.KA_temp[i].col(ind).dot(a_bf_motion.toVector()));
  //         }
  //       }
  //     }
  //     else if (early_full && parent > 0)
  //     {
  //       data.constraints_supported[parent] = 0;
  //     }

  //     if (parent > 0)
  //     {
  //       // pa.toVector() += Ia * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
  //       data.Yaba[parent] += impl::internal::SE3actOn<Scalar>::run(data.liMi[i], Ia);
  //       data.f[parent] += data.liMi[i].act(pa);
  //     }
  //   }
    
  // };

  // A reduced backward sweep that only propagates the affine terms
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct PvBackwardStepReduced
  : public fusion::JointUnaryVisitorBase< PvBackwardStepReduced<Scalar,Options,JointCollectionTpl> >
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
      // typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      auto f_proj = (jdata.S().transpose()*data.f[i]).eval();
      jmodel.jointVelocitySelector(data.u) -= f_proj;   
      data.f[i].toVector().noalias() -= jdata.UDinv()*f_proj; 
      // data.f[i].toVector().noalias() += data.Yaba[i] * data.a_bias[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
      if (parent > 0)
      {
        data.f[parent] += data.liMi[i].act(data.f[i]);
      }
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct cAbaForwardStep2
  : public fusion::JointUnaryVisitorBase< cAbaForwardStep2<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
    Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      
      data.a[i] = data.liMi[i].actInv(data.a[parent]) + data.a_bias[i];
      jmodel.jointVelocitySelector(data.ddq).noalias() =
      jdata.Dinv() * (jmodel.jointVelocitySelector(data.u)) - jdata.UDinv().transpose() * data.a[i].toVector();

      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct PvRegForwardStep2
  : public fusion::JointUnaryVisitorBase< PvRegForwardStep2<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
    Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      
      data.a[i] = data.liMi[i].actInv(data.a[parent]) + data.a_bias[i];
      jmodel.jointVelocitySelector(data.ddq).noalias() =
      jdata.Dinv() * (jmodel.jointVelocitySelector(data.u)) - jdata.UDinv().transpose() * data.a[i].toVector();

      // if (data.constraints_supported[i] > 0)
      // {
        data.lambdaA[i].noalias() = data.lambdaA[parent].segment(data.par_cons_ind[i], data.lambdaA[i].size());
      // }
      for (int j = 0; j < data.constraints_supported[i]; j++)
      {
        jmodel.jointVelocitySelector(data.ddq).noalias() -= 
          data.lambdaA[i][j]*jdata.Dinv() * (data.KAS[i].col(j));
      }
      // if (data.lA[i].size() > 0)
      //   jmodel.jointVelocitySelector(data.ddq).noalias() -= jdata.Dinv() * jdata.S().matrix().transpose() * data.KA_temp[i].transpose()*data.lambdaA[i];

      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
    }
    
  };
    
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct PvForwardStep2
  : public fusion::JointUnaryVisitorBase< PvForwardStep2<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
    Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      bool early_full = data.pv_settings.use_early;
      bool prox = data.pv_settings.mu > 0.0;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      
      data.a[i] = data.liMi[i].actInv(data.a[parent]) + data.a_bias[i];
      jmodel.jointVelocitySelector(data.ddq).noalias() =
      jdata.Dinv() * (jmodel.jointVelocitySelector(data.u)) - jdata.UDinv().transpose() * data.a[i].toVector();

      if (early_full && data.constraints_supported[i] > 0 && !prox)
      {
        Scalar lambda_opt = (data.KAopt[i].dot((data.a[i] - data.a_bias[i]).toVector())
          + data.lAopt[i])/data.sigma[i];

        // size_t max_ind = 0;
        //data.KAS[i].row(0).array().abs().matrix().maxCoeff(&data.svd_max_ind[i]); // Maybe save this from the previous computation?
        data.lambdaA[i].head(data.svd_max_ind[i]) = data.lambdaA[parent].head(data.svd_max_ind[i]);
        data.lambdaA[i](data.svd_max_ind[i]) = lambda_opt;
        for (int k = data.svd_max_ind[i] + 1; k < data.constraints_supported[i]; k++)
          data.lambdaA[i](k) = data.lambdaA[parent](k - 1);
        auto w_normalized = (data.w[i]/(data.w[i].dot(data.w[i]))).eval(); // check whether to save
        data.lambdaA[i].head(data.constraints_supported[i]) -= 2*data.w[i].dot(data.lambdaA[i].head(data.constraints_supported[i]))*w_normalized;
        // 
      }
      else
      {
        data.lambdaA[i].noalias() = data.lambdaA[parent].segment(data.par_cons_ind[i], data.lambdaA[i].size());
      }
      for (int j = 0; j < data.constraints_supported[i] && !prox; j++)
      {
        jmodel.jointVelocitySelector(data.ddq).noalias() -= 
          data.lambdaA[i][j]*jdata.Dinv() * (data.KAS[i].col(j));
      }
      // if (data.lA[i].size() > 0)
      //   jmodel.jointVelocitySelector(data.ddq).noalias() -= jdata.Dinv() * jdata.S().matrix().transpose() * data.KA_temp[i].transpose()*data.lambdaA[i];

      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class ContactModelAllocator, class ContactDataAllocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  pv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
      DataTpl<Scalar,Options,JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau,
      const std::vector<RigidConstraintModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
      std::vector<RigidConstraintDataTpl<Scalar,Options>,ContactDataAllocator> & contact_datas,
      ProximalSettingsTpl<Scalar> & settings,
      Data::PvSettings & pv_settings)
  {

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(tau.size(), model.nv, "The joint torque vector is not of right size");

    data.pv_settings = pv_settings;
    bool early_base = pv_settings.use_early_base;
    bool early_full = pv_settings.use_early;
    // bool early_prox = pv_settings.mu > 0.0;
    
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;

    // for (JointIndex i=0;i<(JointIndex)model.njoints;++i)
    // {
    //   data.lA[i].setZero();
    //   data.lambdaA[i].setZero();
    //   data.LA[i].setZero();
    //   data.KA_temp[i].setZero();
    //   data.KAS[i].setZero();
    // }
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    data.a[0] = data.a_gf[0];
    data.f[0].setZero();
    data.u = tau;

    // Set the lA and LA at the contact points to zero.
    for(std::size_t i=0;i<contact_models.size();++i)
    {
      const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      data.lA[joint_id].setZero();
      data.LA[joint_id].setZero();
    } 
    
    typedef PvForwardStep1<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }

    std::vector<int> condim_counter(model.njoints, 0);
    // Update lAs
    for(std::size_t i=0;i<contact_models.size();++i)
    {
      const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[i];
      typename RigidConstraintData::Motion & vc1 = contact_datas[i].contact1_velocity;
      const JointIndex & joint_id = contact_model.joint1_id;
      int con_dim = contact_model.size();
      // data.lA[joint_id].noalias() -= data.KA_temp[joint_id].template topRows<3>().transpose()*(data.a_gf[joint_id].linear_impl());
      for (int j = condim_counter[joint_id]; j < condim_counter[joint_id] + con_dim; j++)
      {
        // auto lA_update = data.KA[i][0].toVector().transpose()*a_bf_motion.toVector();
        data.lA[joint_id][j] -=
          (data.KA_temp[joint_id].col(j).template head<3>().transpose()*data.a_gf[joint_id].linear_impl());
      }
      // if (contact_model.type == CONTACT_6D)
      // {
      //   data.lA[joint_id].noalias() -= contact_model.joint1_placement.actInv(data.a_bias[joint_id]).toVector();
      // }
      if(contact_model.type == CONTACT_3D)
      { 
        vc1 = contact_model.joint1_placement.actInv(data.v[joint_id]);
        data.lA[joint_id].segment(condim_counter[joint_id], 3).noalias() += vc1.angular().cross(vc1.linear());
      }
      condim_counter[joint_id] += con_dim;
      
    }

    // if (model.nv > 15) // TODO: remove this whole block
    //   std::cout << "data.lA[13] = " << data.lA[13].transpose() << "and data.lA[12] = " << data.lA[12].transpose() << "\n";
    
    if (!early_full)
    {
    typedef PvRegBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1;i>1; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    // Implement the base OSIM speed-up
    // if (early_base)
    // {
    //   for (size_t i = 0; i < model.children[1].size(); i++)
    //   {
    //     JointIndex child = model.children[1][i];
    //     if (data.constraints_supported[child] > 0)
    //     {
    //       data.fb_osim_llt[i].compute(data.LA[1].block(data.par_cons_ind[child],data.par_cons_ind[child],
    //         data.constraints_supported[child], data.constraints_supported[child]));

    //       // copy contents of the KA force vector list to KA_temp
    //       for (size_t j = 0; j < data.KA[child].size(); j++)
    //       {
    //         data.KA_temp[child].col(j) = data.KA[1][j + data.par_cons_ind[child]].toVector();
    //       }
    //       data.Yaba[1].noalias() += data.KA_temp[child]*
    //         data.fb_osim_llt[i].solve(data.KA_temp[child].transpose());

    //       data.lambdaA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]) = data.lA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]);
    //       data.fb_osim_llt[i].solveInPlace(data.lambdaA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]));
    //       data.f[1].toVector() += data.KA_temp[child]*data.lambdaA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]);
    //     }
    //   }
    //   data.constraints_supported[1] = 0;
    // }


    // Do the backward pass from the base to the reference frame
    Pass2::run(model.joints[1],data.joints[1],
                 typename Pass2::ArgsType(model,data));
    }
    else
    { 
      // TODO: add PV-early for when mu = 0
      for(std::size_t i=0;i<contact_models.size();++i)
      {
        const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[i];
        const JointIndex & joint_id = contact_model.joint1_id;

        data.Yaba[joint_id].matrix().noalias() += data.KA_temp[joint_id]*(1/pv_settings.mu)*data.KA_temp[joint_id].transpose();
        data.f[joint_id].toVector().noalias() += data.KA_temp[joint_id]*(1/pv_settings.mu)*data.lA[joint_id];
        
      }

      typedef cAbaBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
      for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
      {
        Pass2::run(model.joints[i],data.joints[i],
                   typename Pass2::ArgsType(model,data));
      }
    }
    

    // // // Compute the Cholesky decomposition
    if (!early_base && !early_full)
    {

      if (data.lA[0].size() > 0)
      {
        data.lA[0].noalias() += data.KA_temp[0].transpose()*data.a_gf[0].toVector();
        // for (int j = 0; j < data.constraints_supported[0]; j++)
        // {
        // data.lA[0][j] += data.KA_temp[0].col(j).transpose()*data.a_gf[0].toVector();
        // }

        // data.lambdaA[0] = data.LA[0].ldlt().solve(data.lA[0]);

        // data.lambdaA[0] = data.lA[0];
        // data.LA[0].ldlt().solveInPlace(data.lambdaA[0]);

        data.lambdaA[0].noalias() = data.lA[0];
        data.LA[0].noalias() += pv_settings.mu * Data::MatrixXs::Identity(data.constraints_supported[0], data.constraints_supported[0]);
        data.osim_llt.compute(data.LA[0]);
        data.lambda_c_prox.setZero();
        int i = 0;
        for (i = 0; i < pv_settings.max_iter; i++)
        {
          // std::cout << "LA[0] = " << data.LA[0] << ", lA[0] = " << data.lA[0] << ", lambdaA[0] = " << data.lambdaA[0] << "\n";
          data.lambdaA[0].noalias() = pv_settings.mu*data.lambda_c_prox + data.lA[0];
          data.osim_llt.solveInPlace(data.lambdaA[0]);
          pv_settings.absolute_residual = (data.lambda_c_prox - data.lambdaA[0]).template lpNorm<Eigen::Infinity>();
          if(check_expression_if_real<Scalar,false>(pv_settings.absolute_residual <= pv_settings.absolute_accuracy)) // In the case where Scalar is not double, this will iterate for max_it.
            break;
          data.lambda_c_prox.noalias() = data.lambdaA[0];
        }

        data.LA[0].template triangularView<Eigen::Upper>() = data.LA[0].template triangularView<Eigen::Lower>().transpose();
        // std::cout << "Num iters = " << i << " and Error L2 residual = " << ((data.LA[0]-pv_settings.mu * Data::MatrixXs::Identity(data.constraints_supported[0], data.constraints_supported[0]))*data.lambdaA[0] - data.lA[0]).template lpNorm<2>() << "\n";
      }

      typedef PvRegForwardStep2<Scalar,Options,JointCollectionTpl> Pass3;
      Pass3::run(model.joints[1],data.joints[1],
                 typename Pass3::ArgsType(model,data));

      // if (early_base)
      // { 
      //   for (size_t i = 0; i < model.children[1].size(); i++)
      //   {
      //     JointIndex child = model.children[1][i];
      //     if (data.constraints_supported[child] > 0)
      //     {
      //       data.lA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]).noalias() += 
      //         data.KA_temp[child].transpose()*data.a[1].toVector();
      //       data.lambdaA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]) = data.lA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]);
      //       data.fb_osim_llt[i].solveInPlace(data.lambdaA[1].segment(data.par_cons_ind[child],data.constraints_supported[child]));
      //     }
      //   }
      // }
    
      for(JointIndex i=2; i<(JointIndex)model.njoints; ++i)
      {
        Pass3::run(model.joints[i],data.joints[i],
                   typename Pass3::ArgsType(model,data));
      }

    }
    
    

    if (early_full)
    {
      std::fill(condim_counter.begin(), condim_counter.end(), 0);
      assert(pv_settings.mu > 0 && "constrainedABA requires mu > 0.");
      typedef PvBackwardStepReduced<Scalar,Options,JointCollectionTpl> Pass4;
      int lambda_ind = 0;
      for(std::size_t j=0;j<contact_models.size();++j)
      {
        const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[j];
        const JointIndex & joint_id = contact_model.joint1_id;
        data.lambda_c_prox.segment(lambda_ind, contact_model.size()).noalias() = 
          (data.lA[joint_id].segment(condim_counter[joint_id], contact_model.size()) + data.KA_temp[joint_id].middleCols(condim_counter[joint_id], contact_model.size()).transpose()*data.a[joint_id].toVector());
        lambda_ind += contact_model.size();
        condim_counter[joint_id] += contact_model.size();
      }

      typedef cAbaForwardStep2<Scalar,Options,JointCollectionTpl> Pass3;
      for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
      {
        if (data.constraints_supported[i] > 0)
          Pass3::run(model.joints[i],data.joints[i],
                   typename Pass3::ArgsType(model,data));
      }

      for (int i = 1; i < pv_settings.max_iter; i++)
      { 
        
        data.lambdaA[0].noalias() += (1/pv_settings.mu)*data.lambda_c_prox;
        // set all f[i] to zero or TODO: to external forces
        for(JointIndex j=1; j<(JointIndex)model.njoints; ++j)
        {
          if (data.constraints_supported[j] > 0)
            data.f[j].toVector().setZero(); //data.v[j].cross(data.h[j]);
        }
        // Compute lambda_prox and update the data.f
        lambda_ind = 0;
        std::fill(condim_counter.begin(), condim_counter.end(), 0);
        for(std::size_t j=0;j<contact_models.size();++j)
        {
          const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[j];
          const JointIndex & joint_id = contact_model.joint1_id;
          // if (condim_counter[joint_id] > 0)
          //   continue;
          data.f[joint_id].toVector().noalias() += data.KA_temp[joint_id].middleCols(condim_counter[joint_id],contact_model.size())*(1/pv_settings.mu)*
            (data.lambda_c_prox.segment(lambda_ind, contact_model.size()));
          lambda_ind += contact_model.size();
          condim_counter[joint_id] += contact_model.size();
        }
        // reduced backward sweep
        for(JointIndex j=(JointIndex)model.njoints-1;j>0; --j)
        {
          if (data.constraints_supported[j] > 0)
            Pass4::run(model.joints[j],data.joints[j],
                 typename Pass4::ArgsType(model,data));
        }
        // outward sweep
        for(JointIndex j=1; j<(JointIndex)model.njoints; ++j)
        {
          if (data.constraints_supported[j] > 0)
            Pass3::run(model.joints[j],data.joints[j],
                 typename Pass3::ArgsType(model,data));
        }
        lambda_ind = 0; 
        std::fill(condim_counter.begin(), condim_counter.end(), 0);
        for(std::size_t j=0;j<contact_models.size();++j)
        {
          const RigidConstraintModelTpl<Scalar,Options> & contact_model = contact_models[j];
          const JointIndex & joint_id = contact_model.joint1_id;
          // if (condim_counter[joint_id] > 0)
            // continue;
          data.lambda_c_prox.segment(lambda_ind,contact_model.size()).noalias() = 
            (data.lA[joint_id].segment(condim_counter[joint_id],contact_model.size()) + data.KA_temp[joint_id].middleCols(condim_counter[joint_id], contact_model.size()).transpose()*data.a[joint_id].toVector());
          lambda_ind += contact_model.size();
          condim_counter[joint_id] += contact_model.size();
        }
        pv_settings.absolute_residual = (data.lambda_c_prox).template lpNorm<Eigen::Infinity>();
        // std::cout << "iter = " << i << " residual = " << pv_settings.absolute_residual << "\n";
        if(check_expression_if_real<Scalar,false>(pv_settings.absolute_residual <= pv_settings.absolute_accuracy)) // In the case where Scalar is not double, this will iterate for max_it.
        {
            // std::cout << "Iterations made: " << i + 1 << std::endl;
            // outward sweep for joints not supporting a constraint
            for(JointIndex j=1; j<(JointIndex)model.njoints; ++j)
            {
              if (data.constraints_supported[j] == 0)
                Pass3::run(model.joints[j],data.joints[j],
                     typename Pass3::ArgsType(model,data));
            }
            break;
          }
        // data.lambda_c_prox.noalias() = data.lambdaA[0];

      }
      
    }
    
    return data.ddq;
  }



  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------

  // Check whether all masses are nonzero and diagonal of inertia is nonzero
  // The second test is overconstraining.
  // template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  // inline bool ABAChecker::checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const
  // {
  //   typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
  //   typedef typename Model::JointIndex JointIndex;
    
  //   for(JointIndex j=1;j<(JointIndex)model.njoints;j++)
  //     if(    (model.inertias[j].mass   ()           < 1e-5) 
  //         || (model.inertias[j].inertia().data()[0] < 1e-5)
  //         || (model.inertias[j].inertia().data()[3] < 1e-5)
  //         || (model.inertias[j].inertia().data()[5] < 1e-5) )
  //       return false;
  //   return true;
  // }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_algorithm_pv_hxx__
