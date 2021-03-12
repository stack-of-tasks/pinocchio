//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_autodiff_code_generator_algo_hpp__
#define __pinocchio_autodiff_code_generator_algo_hpp__

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

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
  } //namespace casadi

} // namespace pinocchio

#endif // ifndef __pinocchio_autodiff_code_generator_algo_hpp__
