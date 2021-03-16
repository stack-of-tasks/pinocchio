//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_autodiff_code_generator_algo_hpp__
#define __pinocchio_autodiff_code_generator_algo_hpp__

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/contact-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

namespace pinocchio
{
  namespace casadi
  {
    template<typename _Scalar>
    struct AutoDiffABA
    {
      typedef _Scalar Scalar;
      typedef ::casadi::SX ADScalar;
      typedef ::casadi::SXVector ADSVector;
      typedef ::casadi::DM DMMatrix;
      typedef ::casadi::DMVector DMVector;
      enum { Options = 0 };

      typedef pinocchio::ModelTpl<Scalar,Options> Model;
      typedef pinocchio::DataTpl<Scalar,Options> Data;
      typedef pinocchio::ModelTpl<ADScalar,Options> ADModel;
      typedef pinocchio::DataTpl<ADScalar,Options> ADData;
      
      typedef typename Model::ConfigVectorType ConfigVectorType;
      typedef typename Model::TangentVectorType TangentVectorType;
      typedef typename ADModel::ConfigVectorType ADConfigVectorType;
      typedef typename ADModel::TangentVectorType ADTangentVectorType;

      typedef typename Data::MatrixXs MatrixXs;
      typedef typename Data::VectorXs VectorXs;
      typedef typename Data::RowMatrixXs RowMatrixXs;
      
      typedef ::casadi::Function ADFun;
      
      explicit AutoDiffABA(const Model& model)
        : ad_model(model.template cast<ADScalar>())
        , ad_data(ad_model)
        , cs_q(ADScalar::sym("q", model.nq))
        , cs_v(ADScalar::sym("v", model.nv))
        , cs_tau(ADScalar::sym("tau", model.nv))
        , cs_v_int(ADScalar::sym("v_inc", model.nv))
        , q_ad(model.nq)
        , q_int_ad(model.nq)
        , v_ad(model.nv)
        , v_int_ad(model.nv)
        , tau_ad(model.nv)
        , cs_ddq(model.nv,1)
        , q_vec((size_t)model.nq)
        , v_vec((size_t)model.nv)
        , v_int_vec((size_t)model.nv)
        , tau_vec((size_t)model.nv)
        , ddq(model.nv)
        , ddq_dq(model.nv,model.nv)
        , ddq_dv(model.nv,model.nv)
        , ddq_dtau(model.nv,model.nv)
      {
        q_ad = Eigen::Map<ADConfigVectorType>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
        v_int_ad = Eigen::Map<ADConfigVectorType>(static_cast< std::vector<ADScalar> >(cs_v_int).data(),model.nv,1);
        v_ad = Eigen::Map<ADTangentVectorType>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
        tau_ad = Eigen::Map<ADTangentVectorType>(static_cast< std::vector<ADScalar> >(cs_tau).data(),model.nv,1);

        Eigen::Map<TangentVectorType>(v_int_vec.data(),model.nv,1).setZero();
       
        buildMap();
      }

      void buildMap()
      {
        //Integrate q + v_int = q_int
        pinocchio::integrate(ad_model,q_ad,v_int_ad,q_int_ad);
        //Run ABA with new q_int
        pinocchio::aba(ad_model,ad_data,q_int_ad,v_ad,tau_ad);
        //Copy Output
        pinocchio::casadi::copy(ad_data.ddq,cs_ddq);

        cs_ddq_dq = jacobian(cs_ddq, cs_v_int);
        cs_ddq_dv = jacobian(cs_ddq, cs_v);
        cs_ddq_dtau = jacobian(cs_ddq, cs_tau);

        ad_fun = ADFun("eval_aba",
                       ADSVector {cs_q, cs_v_int, cs_v, cs_tau},
                       ADSVector {cs_ddq});
        ad_fun_dq = ADFun("eval_ddq_dq",
                          ADSVector {cs_q,cs_v_int,cs_v,cs_tau},
                          ADSVector {cs_ddq_dq});
        ad_fun_dv = ADFun("eval_ddq_dv",
                          ADSVector {cs_q,cs_v_int, cs_v, cs_tau},
                          ADSVector {cs_ddq_dv});
        ad_fun_dtau = ADFun("eval_ddq_dtau",
                            ADSVector {cs_q,cs_v_int, cs_v, cs_tau},
                            ADSVector {cs_ddq_dtau});
    
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(const Eigen::MatrixBase<ConfigVectorType1> & q,
                        const Eigen::MatrixBase<TangentVectorType1> & v,
                        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(),ad_model.nq,1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(),ad_model.nv,1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(),ad_model.nv,1) = tau;
        fun_output = ad_fun(DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
        ddq =
          Eigen::Map<TangentVectorType>(static_cast< std::vector<Scalar> >
                                        (fun_output).data(),ad_model.nv,1);
      }

    template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
    void evalJacobian(const Eigen::MatrixBase<ConfigVectorType1> & q,
                      const Eigen::MatrixBase<TangentVectorType1> & v,
                      const Eigen::MatrixBase<TangentVectorType2> & tau)
    {
      Eigen::Map<ConfigVectorType1>(q_vec.data(),ad_model.nq,1) = q;
      Eigen::Map<TangentVectorType1>(v_vec.data(),ad_model.nv,1) = v;
      Eigen::Map<TangentVectorType2>(tau_vec.data(),ad_model.nv,1) = tau;
      fun_output_dq   = ad_fun_dq  (DMVector {q_vec,v_int_vec,v_vec,tau_vec})[0];
      fun_output_dv   = ad_fun_dv  (DMVector {q_vec,v_int_vec,v_vec,tau_vec})[0];
      fun_output_dtau = ad_fun_dtau(DMVector {q_vec,v_int_vec,v_vec,tau_vec})[0];

      ddq_dq =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                                (fun_output_dq).data(),ad_model.nv,ad_model.nv);
      ddq_dv =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                                (fun_output_dv).data(),ad_model.nv,ad_model.nv);
      ddq_dtau =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                                (fun_output_dtau).data(),ad_model.nv,ad_model.nv);
    }
      
      TangentVectorType ddq;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau;
      
    protected:
      ADModel ad_model;
      ADData ad_data;
      /// \brief Options to generate or not the source code for the evaluation function
      bool build_forward;      
      /// \brief Options to build or not the Jacobian of he function
      bool build_jacobian;
      
      ADScalar cs_q, cs_v, cs_tau, cs_v_int, cs_ddq;

      //Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau;

      ADConfigVectorType q_ad, q_int_ad;
      ADTangentVectorType v_ad, v_int_ad, tau_ad;
      ADFun ad_fun, ad_fun_dq, ad_fun_dv, ad_fun_dtau;

      DMMatrix fun_output, fun_output_dq, fun_output_dv, fun_output_dtau;
      
      std::vector<Scalar> q_vec, v_vec, v_int_vec, tau_vec;      
    };

    template<typename _Scalar>
    struct AutoDiffABADerivatives
    {
      typedef _Scalar Scalar;
      typedef ::casadi::SX ADScalar;
      typedef ::casadi::SXVector ADSVector;
      typedef ::casadi::DM DMMatrix;
      typedef ::casadi::DMVector DMVector;
      enum { Options = 0 };

      typedef pinocchio::ModelTpl<Scalar,Options> Model;
      typedef pinocchio::DataTpl<Scalar,Options> Data;
      typedef pinocchio::ModelTpl<ADScalar,Options> ADModel;
      typedef pinocchio::DataTpl<ADScalar,Options> ADData;
      
      typedef typename Model::ConfigVectorType ConfigVectorType;
      typedef typename Model::TangentVectorType TangentVectorType;
      typedef typename ADModel::ConfigVectorType ADConfigVectorType;
      typedef typename ADModel::TangentVectorType ADTangentVectorType;

      typedef typename Data::MatrixXs MatrixXs;
      typedef typename Data::VectorXs VectorXs;
      typedef typename Data::RowMatrixXs RowMatrixXs;
      
      typedef ::casadi::Function ADFun;
      
      explicit AutoDiffABADerivatives(const Model& model)
        : ad_model(model.template cast<ADScalar>())
        , ad_data(ad_model)
        , cs_q(ADScalar::sym("q", model.nq))
        , cs_v(ADScalar::sym("v", model.nv))
        , cs_tau(ADScalar::sym("tau", model.nv))
        , q_ad(model.nq)
        , v_ad(model.nv)
        , tau_ad(model.nv)
        , cs_ddq(model.nv,1)
        , q_vec((size_t)model.nq)
        , v_vec((size_t)model.nv)
        , tau_vec((size_t)model.nv)
        , ddq(model.nv)
        , ddq_dq(model.nv,model.nv)
        , ddq_dv(model.nv,model.nv)
        , ddq_dtau(model.nv,model.nv)
      {
        q_ad = Eigen::Map<ADConfigVectorType>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
        v_ad = Eigen::Map<ADTangentVectorType>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
        tau_ad = Eigen::Map<ADTangentVectorType>(static_cast< std::vector<ADScalar> >(cs_tau).data(),model.nv,1);
       
        buildMap();
      }

      void buildMap()
      {
        //Run ABA with new q_int
        pinocchio::computeABADerivatives(ad_model,ad_data,q_ad,v_ad,tau_ad);
        //Copy Output
        ad_data.Minv.template triangularView<Eigen::StrictlyLower>()
          = ad_data.Minv.transpose().template triangularView<Eigen::StrictlyLower>();
        pinocchio::casadi::copy(ad_data.ddq,cs_ddq);
        pinocchio::casadi::copy(ad_data.ddq_dq,cs_ddq_dq);
        pinocchio::casadi::copy(ad_data.ddq_dv,cs_ddq_dv);
        pinocchio::casadi::copy(ad_data.Minv,cs_ddq_dtau);

        ad_fun = ADFun("eval_aba_derivs",
                       ADSVector {cs_q, cs_v, cs_tau},
                       ADSVector {cs_ddq, cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau});
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(const Eigen::MatrixBase<ConfigVectorType1> & q,
                        const Eigen::MatrixBase<TangentVectorType1> & v,
                        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(),ad_model.nq,1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(),ad_model.nv,1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(),ad_model.nv,1) = tau;
        fun_output = ad_fun(DMVector {q_vec, v_vec, tau_vec});

        ddq =
          Eigen::Map<TangentVectorType>(static_cast< std::vector<Scalar> >
                                        (fun_output[0]).data(),ad_model.nv,1);
        ddq_dq =
          Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                               (fun_output[1]).data(),ad_model.nv,ad_model.nv);
        ddq_dv =
          Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                               (fun_output[2]).data(),ad_model.nv,ad_model.nv);
        ddq_dtau =
          Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                               (fun_output[3]).data(),ad_model.nv,ad_model.nv);
      }
      
      TangentVectorType ddq;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau;
      
    protected:
      ADModel ad_model;
      ADData ad_data;
      
      ADScalar cs_q, cs_v, cs_tau, cs_ddq;

      //Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau;

      ADConfigVectorType q_ad;
      ADTangentVectorType v_ad, tau_ad;
      ADFun ad_fun;

      std::vector<DMMatrix> fun_output;
      
      std::vector<Scalar> q_vec, v_vec, tau_vec;
    };


    template<typename _Scalar>
    struct AutoDiffContactDynamics
    {
      typedef _Scalar Scalar;
      typedef ::casadi::SX ADScalar;
      typedef ::casadi::SXVector ADSVector;
      typedef ::casadi::DM DMMatrix;
      typedef ::casadi::DMVector DMVector;
      enum { Options = 0 };

      typedef pinocchio::ModelTpl<Scalar,Options> Model;
      typedef pinocchio::DataTpl<Scalar,Options> Data;
      typedef typename pinocchio::RigidContactModelTpl<Scalar,Options> ContactModel;
      typedef Eigen::aligned_allocator<ContactModel> ContactModelAllocator;
      typedef typename std::vector<ContactModel, ContactModelAllocator> ContactModelVector;
      typedef typename pinocchio::RigidContactDataTpl<Scalar,Options> ContactData;
      typedef Eigen::aligned_allocator<ContactData> ContactDataAllocator;
      typedef typename std::vector<ContactData, ContactDataAllocator> ContactDataVector;
      
      typedef pinocchio::ModelTpl<ADScalar,Options> ADModel;
      typedef pinocchio::DataTpl<ADScalar,Options> ADData;
      typedef typename pinocchio::RigidContactModelTpl<ADScalar,Options> ADContactModel;
      typedef Eigen::aligned_allocator<ADContactModel> ADContactModelAllocator;
      typedef typename std::vector<ADContactModel, ADContactModelAllocator> ADContactModelVector;
      typedef typename pinocchio::RigidContactDataTpl<ADScalar,Options> ADContactData;
      typedef Eigen::aligned_allocator<ADContactData> ADContactDataAllocator;
      typedef typename std::vector<ADContactData, ADContactDataAllocator> ADContactDataVector;
      
      typedef typename Model::ConfigVectorType ConfigVectorType;
      typedef typename Model::TangentVectorType TangentVectorType;
      typedef typename ADModel::ConfigVectorType ADConfigVectorType;
      typedef typename ADModel::TangentVectorType ADTangentVectorType;

      typedef typename Data::MatrixXs MatrixXs;
      typedef typename Data::VectorXs VectorXs;
      typedef typename Data::RowMatrixXs RowMatrixXs;

      static Eigen::DenseIndex constraintDim(const ContactModelVector& contact_models)
      {
        Eigen::DenseIndex num_total_constraints = 0;
        for(typename ContactModelVector::const_iterator it = contact_models.begin();
            it != contact_models.end();
            ++it)
        {
          PINOCCHIO_CHECK_INPUT_ARGUMENT(it->size() > 0,
                                         "The dimension of the constraint must be positive");
          num_total_constraints += it->size();
        }
        return num_total_constraints;
      }

      typedef ::casadi::Function ADFun;
      
      explicit AutoDiffContactDynamics(const Model& model,
                                       const ContactModelVector& contact_models)
        : ad_model(model.template cast<ADScalar>())
        , ad_data(ad_model)
        , nc(constraintDim(contact_models))
        , cs_q(ADScalar::sym("q", model.nq))
        , cs_v(ADScalar::sym("v", model.nv))
        , cs_tau(ADScalar::sym("tau", model.nv))
        , cs_v_int(ADScalar::sym("v_inc", model.nv))
        , q_ad(model.nq)
        , q_int_ad(model.nq)
        , v_ad(model.nv)
        , v_int_ad(model.nv)
        , tau_ad(model.nv)
        , cs_ddq(model.nv,1)
        , cs_lambda_c(model.nv,1)
        , q_vec((size_t)model.nq)
        , v_vec((size_t)model.nv)
        , v_int_vec((size_t)model.nv)
        , tau_vec((size_t)model.nv)
        , ddq(model.nv)
        , lambda_c(nc)
        , ddq_dq(model.nv,model.nv)
        , ddq_dv(model.nv,model.nv)
        , ddq_dtau(model.nv,model.nv)
        , dlambda_dq(nc,model.nv)
        , dlambda_dv(nc,model.nv)
        , dlambda_dtau(nc,model.nv)
      {
        q_ad = Eigen::Map<ADConfigVectorType>(static_cast< std::vector<ADScalar> >(cs_q).data(),model.nq,1);
        v_int_ad = Eigen::Map<ADConfigVectorType>(static_cast< std::vector<ADScalar> >(cs_v_int).data(),model.nv,1);
        v_ad = Eigen::Map<ADTangentVectorType>(static_cast< std::vector<ADScalar> >(cs_v).data(),model.nv,1);
        tau_ad = Eigen::Map<ADTangentVectorType>(static_cast< std::vector<ADScalar> >(cs_tau).data(),model.nv,1);

        Eigen::Map<TangentVectorType>(v_int_vec.data(),model.nv,1).setZero();

        for(int k=0;k<contact_models.size();++k){
          ad_contact_models.push_back(contact_models[k].template cast<ADScalar>());
          ad_contact_datas.push_back(ADContactData(ad_contact_models[k]));
        }

        pinocchio::initContactDynamics(ad_model, ad_data, ad_contact_models);
        buildMap();
      }

      void buildMap()
      {
        //Integrate q + v_int = q_int
        pinocchio::integrate(ad_model,q_ad,v_int_ad,q_int_ad);
        //Run contact dynamics with new q_int
        pinocchio::contactDynamics(ad_model,ad_data,q_int_ad,v_ad,tau_ad,
                                   ad_contact_models,ad_contact_datas);
        //Copy Output
        pinocchio::casadi::copy(ad_data.ddq,cs_ddq);
        pinocchio::casadi::copy(ad_data.lambda_c,cs_lambda_c);

        cs_ddq_dq = jacobian(cs_ddq, cs_v_int);
        cs_ddq_dv = jacobian(cs_ddq, cs_v);
        cs_ddq_dtau = jacobian(cs_ddq, cs_tau);

        cs_lambda_dq = jacobian(cs_lambda_c, cs_v_int);
        cs_lambda_dv = jacobian(cs_lambda_c, cs_v);
        cs_lambda_dtau = jacobian(cs_lambda_c, cs_tau);
        
        ad_fun = ADFun("eval_contactDynamics",
                       ADSVector {cs_q, cs_v_int, cs_v, cs_tau},
                       ADSVector {cs_ddq, cs_lambda_c});
        ad_fun_derivs = ADFun("eval_contactDynamics_derivs",
                              ADSVector {cs_q,cs_v_int,cs_v,cs_tau},
                              ADSVector {cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau,
                                  cs_lambda_dq, cs_lambda_dv, cs_lambda_dtau});
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(const Eigen::MatrixBase<ConfigVectorType1> & q,
                        const Eigen::MatrixBase<TangentVectorType1> & v,
                        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(),ad_model.nq,1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(),ad_model.nv,1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(),ad_model.nv,1) = tau;
        fun_output = ad_fun(DMVector {q_vec, v_int_vec, v_vec, tau_vec});
        ddq =
          Eigen::Map<TangentVectorType>(static_cast< std::vector<Scalar> >
                                        (fun_output[0]).data(),ad_model.nv,1);
        lambda_c =
          Eigen::Map<TangentVectorType>(static_cast< std::vector<Scalar> >
                                        (fun_output[1]).data(),ad_model.nv,1);        
      }

    template<typename ConfigVectorType1, typename TangentVectorType1,
             typename TangentVectorType2>
    void evalJacobian(const Eigen::MatrixBase<ConfigVectorType1> & q,
                      const Eigen::MatrixBase<TangentVectorType1> & v,
                      const Eigen::MatrixBase<TangentVectorType2> & tau)
    {
      Eigen::Map<ConfigVectorType1>(q_vec.data(),ad_model.nq,1) = q;
      Eigen::Map<TangentVectorType1>(v_vec.data(),ad_model.nv,1) = v;
      Eigen::Map<TangentVectorType2>(tau_vec.data(),ad_model.nv,1) = tau;
      fun_output_derivs   = ad_fun_derivs (DMVector {q_vec,v_int_vec,v_vec,tau_vec});
      ddq_dq =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                             (fun_output_derivs[0]).data(),ad_model.nv,ad_model.nv);
      ddq_dv =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                             (fun_output_derivs[1]).data(),ad_model.nv,ad_model.nv);
      ddq_dtau =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                             (fun_output_derivs[2]).data(),ad_model.nv,ad_model.nv);
      dlambda_dq =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                             (fun_output_derivs[3]).data(),nc,ad_model.nv);
      dlambda_dv =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                             (fun_output_derivs[4]).data(),nc,ad_model.nv);
      dlambda_dtau =
        Eigen::Map<MatrixXs>(static_cast< std::vector<Scalar> >
                             (fun_output_derivs[5]).data(),nc,ad_model.nv);
      
    }
      
      TangentVectorType ddq, lambda_c;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau,
        dlambda_dq, dlambda_dv, dlambda_dtau;
      
    protected:
      ADModel ad_model;
      ADData ad_data;
      ADContactModelVector ad_contact_models;
      ADContactDataVector ad_contact_datas;
      
      Eigen::DenseIndex nc;
      ADScalar cs_q, cs_v, cs_tau, cs_v_int, cs_ddq, cs_lambda_c;

      //Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau,
        cs_lambda_dq, cs_lambda_dv, cs_lambda_dtau;

      ADConfigVectorType q_ad, q_int_ad;
      ADTangentVectorType v_ad, v_int_ad, tau_ad;
      ADFun ad_fun, ad_fun_derivs;

      std::vector<DMMatrix> fun_output, fun_output_derivs;
      
      std::vector<Scalar> q_vec, v_vec, v_int_vec, tau_vec;      
    };


    
    
  } //namespace casadi

} // namespace pinocchio

#endif // ifndef __pinocchio_autodiff_code_generator_algo_hpp__
