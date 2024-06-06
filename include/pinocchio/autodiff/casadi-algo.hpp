//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_autodiff_code_generator_algo_hpp__
#define __pinocchio_autodiff_code_generator_algo_hpp__

#include <casadi/core/casadi_types.hpp>
#include <casadi/core/code_generator.hpp>

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"

namespace pinocchio
{
  namespace casadi
  {

    template<typename _Scalar>
    struct AutoDiffAlgoBase
    {
      typedef _Scalar Scalar;
      typedef ::casadi::SX ADScalar;
      typedef ::casadi::SXVector ADSVector;
      typedef ::casadi::DM DMMatrix;
      typedef ::casadi::DMVector DMVector;
      enum
      {
        Options = 0
      };

      typedef pinocchio::ModelTpl<Scalar, Options> Model;
      typedef pinocchio::DataTpl<Scalar, Options> Data;
      typedef pinocchio::ModelTpl<ADScalar, Options> ADModel;
      typedef pinocchio::DataTpl<ADScalar, Options> ADData;

      typedef typename Model::ConfigVectorType ConfigVectorType;
      typedef typename Model::TangentVectorType TangentVectorType;
      typedef typename ADModel::ConfigVectorType ADConfigVectorType;
      typedef typename ADModel::TangentVectorType ADTangentVectorType;

      typedef typename Data::MatrixXs MatrixXs;
      typedef typename Data::VectorXs VectorXs;
      typedef typename Data::RowMatrixXs RowMatrixXs;

      typedef ::casadi::Function ADFun;

      AutoDiffAlgoBase(
        const Model & model,
        const std::string & filename,
        const std::string & libname,
        const std::string & fun_name)
      : ad_model(model.template cast<ADScalar>())
      , ad_data(ad_model)
      , filename(filename)
      , libname(libname)
      , fun_name(fun_name)
      , cg_generated(filename)
      , build_forward(true)
      , build_jacobian(true)
      {
      }

      virtual ~AutoDiffAlgoBase()
      {
      }

      /// \brief build the mapping Y = f(X)
      virtual void buildMap() = 0;

      void initLib()
      {
        buildMap();
        // Generated code;
        cg_generated.add(ad_fun);
        fun_operation_count = ad_fun.n_instructions() - ad_fun.nnz_in() - ad_fun.nnz_out();
        if (build_jacobian)
        {
          cg_generated.add(ad_fun_derivs);
          fun_derivs_operation_count =
            ad_fun_derivs.n_instructions() - ad_fun_derivs.nnz_in() - ad_fun_derivs.nnz_out();
        }
        cg_generated.generate();
      }

      bool existLib() const
      {
        std::ifstream file((libname + ".so").c_str());
        return file.good();
      }

      void compileLib()
      {
        std::string compile_command =
          "clang -fPIC -shared -Ofast -DNDEBUG " + filename + ".c -o " + libname + ".so";

#ifdef __APPLE__
        compile_command += " -isysroot "
                           "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/"
                           "Developer/SDKs/MacOSX.sdk";
#endif

        const int flag = system(compile_command.c_str());
        if (flag != 0)
        {
          std::cerr << "Compilation failed" << std::endl; // TODO: raised here
        }
      }

      void loadLib(const bool generate_if_not_exist = true)
      {
        if (!existLib() && generate_if_not_exist)
          compileLib();

        fun = ::casadi::external(fun_name, "./" + libname + ".so");
        if (build_jacobian)
        {
          fun_derivs = ::casadi::external(fun_name + "_derivs", "./" + libname + ".so");
        }
      }

      casadi_int getFunOperationCount() const
      {
        return fun_operation_count;
      }

      casadi_int getFunDerivsOperationCount() const
      {
        return fun_derivs_operation_count;
      }

    protected:
      ADModel ad_model;
      ADData ad_data;
      std::string filename, libname, fun_name;
      ::casadi::CodeGenerator cg_generated;

      /// \brief Options to generate or not the source code for the evaluation function
      bool build_forward;
      /// \brief Options to build or not the Jacobian of he function
      bool build_jacobian;

      ADFun ad_fun, ad_fun_derivs;
      ADFun fun, fun_derivs;
      std::vector<DMMatrix> fun_output, fun_output_derivs;
      casadi_int fun_operation_count, fun_derivs_operation_count;
    };

    template<typename _Scalar>
    struct AutoDiffABA : public AutoDiffAlgoBase<_Scalar>
    {
      typedef AutoDiffAlgoBase<_Scalar> Base;
      typedef typename Base::Scalar Scalar;

      typedef typename Base::TangentVectorType TangentVectorType;
      typedef typename Base::RowMatrixXs RowMatrixXs;
      typedef typename Base::VectorXs VectorXs;
      typedef typename Base::MatrixXs MatrixXs;

      typedef typename Base::ADFun ADFun;
      typedef typename Base::DMVector DMVector;
      typedef typename Base::DMMatrix DMMatrix;
      typedef typename Base::ADScalar ADScalar;
      typedef typename Base::ADSVector ADSVector;
      typedef typename Base::ADConfigVectorType ADConfigVectorType;
      typedef typename Base::ADTangentVectorType ADTangentVectorType;

      explicit AutoDiffABA(
        const Model & model,
        const std::string & filename = "casadi_aba",
        const std::string & libname = "libcasadi_cg_aba",
        const std::string & fun_name = "eval_f")
      : Base(model, filename, libname, fun_name)
      , ddq(model.nv)
      , ddq_dq(model.nv, model.nv)
      , ddq_dv(model.nv, model.nv)
      , ddq_dtau(model.nv, model.nv)
      , cs_q(ADScalar::sym("q", model.nq))
      , cs_v(ADScalar::sym("v", model.nv))
      , cs_tau(ADScalar::sym("tau", model.nv))
      , cs_v_int(ADScalar::sym("v_inc", model.nv))
      , cs_ddq(model.nv, 1)
      , q_ad(model.nq)
      , q_int_ad(model.nq)
      , v_ad(model.nv)
      , v_int_ad(model.nv)
      , tau_ad(model.nv)
      , q_vec((size_t)model.nq)
      , v_vec((size_t)model.nv)
      , v_int_vec((size_t)model.nv)
      , tau_vec((size_t)model.nv)
      {
        q_ad = Eigen::Map<ADConfigVectorType>(
          static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);
        v_int_ad = Eigen::Map<ADConfigVectorType>(
          static_cast<std::vector<ADScalar>>(cs_v_int).data(), model.nv, 1);
        v_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);
        tau_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

        Eigen::Map<TangentVectorType>(v_int_vec.data(), model.nv, 1).setZero();
      }

      virtual ~AutoDiffABA()
      {
      }

      virtual void buildMap()
      {
        // Integrate q + v_int = q_int
        pinocchio::integrate(ad_model, q_ad, v_int_ad, q_int_ad);
        // Run ABA with new q_int
        pinocchio::aba(ad_model, ad_data, q_int_ad, v_ad, tau_ad, Convention::WORLD);
        // Copy Output
        pinocchio::casadi::copy(ad_data.ddq, cs_ddq);

        cs_ddq_dq = jacobian(cs_ddq, cs_v_int);
        cs_ddq_dv = jacobian(cs_ddq, cs_v);
        cs_ddq_dtau = jacobian(cs_ddq, cs_tau);

        ad_fun = ADFun(fun_name, ADSVector{cs_q, cs_v_int, cs_v, cs_tau}, ADSVector{cs_ddq});

        ad_fun_derivs = ADFun(
          fun_name + "_derivs", ADSVector{cs_q, cs_v_int, cs_v, cs_tau},
          ADSVector{cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau});
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(
        const Eigen::MatrixBase<ConfigVectorType1> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(), ad_model.nq, 1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(), ad_model.nv, 1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(), ad_model.nv, 1) = tau;
        fun_output = fun(DMVector{q_vec, v_int_vec, v_vec, tau_vec});
        ddq = Eigen::Map<TangentVectorType>(
          static_cast<std::vector<Scalar>>(fun_output[0]).data(), ad_model.nv, 1);
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalJacobian(
        const Eigen::MatrixBase<ConfigVectorType1> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(), ad_model.nq, 1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(), ad_model.nv, 1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(), ad_model.nv, 1) = tau;
        fun_output_derivs = fun_derivs(DMVector{q_vec, v_int_vec, v_vec, tau_vec});

        ddq_dq = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[0]).data(), ad_model.nv, ad_model.nv);
        ddq_dv = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[1]).data(), ad_model.nv, ad_model.nv);
        ddq_dtau = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[2]).data(), ad_model.nv, ad_model.nv);
      }

      TangentVectorType ddq;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau;

    protected:
      using Base::ad_data;
      using Base::ad_model;
      using Base::cg_generated;
      using Base::filename;
      using Base::fun_name;
      using Base::libname;

      using Base::ad_fun;
      using Base::ad_fun_derivs;
      using Base::fun;
      using Base::fun_derivs;
      using Base::fun_output;
      using Base::fun_output_derivs;

      ADScalar cs_q, cs_v, cs_tau, cs_v_int, cs_ddq;

      // Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau;

      ADConfigVectorType q_ad, q_int_ad;
      ADTangentVectorType v_ad, v_int_ad, tau_ad;

      std::vector<Scalar> q_vec, v_vec, v_int_vec, tau_vec;
    };

    template<typename _Scalar>
    struct AutoDiffABADerivatives : public AutoDiffAlgoBase<_Scalar>
    {

      typedef AutoDiffAlgoBase<_Scalar> Base;
      typedef typename Base::Scalar Scalar;

      typedef typename Base::TangentVectorType TangentVectorType;
      typedef typename Base::RowMatrixXs RowMatrixXs;
      typedef typename Base::VectorXs VectorXs;
      typedef typename Base::MatrixXs MatrixXs;

      typedef typename Base::ADFun ADFun;
      typedef typename Base::DMVector DMVector;
      typedef typename Base::DMMatrix DMMatrix;
      typedef typename Base::ADScalar ADScalar;
      typedef typename Base::ADSVector ADSVector;
      typedef typename Base::ADConfigVectorType ADConfigVectorType;
      typedef typename Base::ADTangentVectorType ADTangentVectorType;

      explicit AutoDiffABADerivatives(
        const Model & model,
        const std::string & filename = "casadi_abaDerivs",
        const std::string & libname = "libcasadi_cg_abaDerivs",
        const std::string & fun_name = "eval_f")
      : Base(model, filename, libname, fun_name)
      , cs_q(ADScalar::sym("q", model.nq))
      , cs_v(ADScalar::sym("v", model.nv))
      , cs_tau(ADScalar::sym("tau", model.nv))
      , q_ad(model.nq)
      , v_ad(model.nv)
      , tau_ad(model.nv)
      , cs_ddq(model.nv, 1)
      , q_vec((size_t)model.nq)
      , v_vec((size_t)model.nv)
      , tau_vec((size_t)model.nv)
      , ddq(model.nv)
      , ddq_dq(model.nv, model.nv)
      , ddq_dv(model.nv, model.nv)
      , ddq_dtau(model.nv, model.nv)
      {
        q_ad = Eigen::Map<ADConfigVectorType>(
          static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);
        v_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);
        tau_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

        Base::build_jacobian = false;
      }

      virtual ~AutoDiffABADerivatives()
      {
      }

      virtual void buildMap()
      {
        // Run ABA with new q_int
        pinocchio::computeABADerivatives(ad_model, ad_data, q_ad, v_ad, tau_ad);
        // Copy Output
        ad_data.Minv.template triangularView<Eigen::StrictlyLower>() =
          ad_data.Minv.transpose().template triangularView<Eigen::StrictlyLower>();
        pinocchio::casadi::copy(ad_data.ddq, cs_ddq);
        pinocchio::casadi::copy(ad_data.ddq_dq, cs_ddq_dq);
        pinocchio::casadi::copy(ad_data.ddq_dv, cs_ddq_dv);
        pinocchio::casadi::copy(ad_data.Minv, cs_ddq_dtau);

        ad_fun = ADFun(
          fun_name, ADSVector{cs_q, cs_v, cs_tau},
          ADSVector{cs_ddq, cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau});
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(
        const Eigen::MatrixBase<ConfigVectorType1> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(), ad_model.nq, 1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(), ad_model.nv, 1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(), ad_model.nv, 1) = tau;
        fun_output = fun(DMVector{q_vec, v_vec, tau_vec});

        ddq = Eigen::Map<TangentVectorType>(
          static_cast<std::vector<Scalar>>(fun_output[0]).data(), ad_model.nv, 1);
        ddq_dq = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[1]).data(), ad_model.nv, ad_model.nv);
        ddq_dv = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[2]).data(), ad_model.nv, ad_model.nv);
        ddq_dtau = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[3]).data(), ad_model.nv, ad_model.nv);
      }

      TangentVectorType ddq;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau;

    protected:
      using Base::ad_data;
      using Base::ad_model;
      using Base::cg_generated;
      using Base::filename;
      using Base::fun_name;
      using Base::libname;

      using Base::ad_fun;
      using Base::ad_fun_derivs;
      using Base::fun;
      using Base::fun_derivs;
      using Base::fun_output;
      using Base::fun_output_derivs;

      ADScalar cs_q, cs_v, cs_tau, cs_ddq;
      // Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau;

      ADConfigVectorType q_ad;
      ADTangentVectorType v_ad, tau_ad;
      std::vector<Scalar> q_vec, v_vec, tau_vec;
    };

    template<typename _Scalar>
    struct AutoDiffConstraintDynamics : public AutoDiffAlgoBase<_Scalar>
    {
      typedef AutoDiffAlgoBase<_Scalar> Base;
      typedef typename Base::Scalar Scalar;
      typedef typename Base::ADScalar ADScalar;
      typedef typename Base::ADSVector ADSVector;

      typedef typename Base::TangentVectorType TangentVectorType;
      typedef typename Base::RowMatrixXs RowMatrixXs;
      typedef typename Base::VectorXs VectorXs;
      typedef typename Base::MatrixXs MatrixXs;

      typedef typename Base::ADFun ADFun;
      typedef typename Base::DMVector DMVector;
      typedef typename Base::DMMatrix DMMatrix;
      typedef typename Base::ADConfigVectorType ADConfigVectorType;
      typedef typename Base::ADTangentVectorType ADTangentVectorType;

      typedef typename pinocchio::RigidConstraintModelTpl<Scalar, Base::Options> ConstraintModel;
      typedef Eigen::aligned_allocator<ConstraintModel> ConstraintModelAllocator;
      typedef typename std::vector<ConstraintModel, ConstraintModelAllocator> ConstraintModelVector;
      typedef typename pinocchio::RigidConstraintDataTpl<Scalar, Base::Options> ConstraintData;
      typedef Eigen::aligned_allocator<ConstraintData> ConstraintDataAllocator;
      typedef typename std::vector<ConstraintData, ConstraintDataAllocator> ConstraintDataVector;

      typedef typename pinocchio::RigidConstraintModelTpl<ADScalar, Base::Options>
        ADConstraintModel;
      typedef Eigen::aligned_allocator<ADConstraintModel> ADConstraintModelAllocator;
      typedef typename std::vector<ADConstraintModel, ADConstraintModelAllocator>
        ADConstraintModelVector;
      typedef typename pinocchio::RigidConstraintDataTpl<ADScalar, Base::Options> ADConstraintData;
      typedef Eigen::aligned_allocator<ADConstraintData> ADConstraintDataAllocator;
      typedef typename std::vector<ADConstraintData, ADConstraintDataAllocator>
        ADConstraintDataVector;

      static Eigen::DenseIndex constraintDim(const ConstraintModelVector & contact_models)
      {
        Eigen::DenseIndex num_total_constraints = 0;
        for (typename ConstraintModelVector::const_iterator it = contact_models.begin();
             it != contact_models.end(); ++it)
        {
          PINOCCHIO_CHECK_INPUT_ARGUMENT(
            it->size() > 0, "The dimension of the constraint must be positive");
          num_total_constraints += it->size();
        }
        return num_total_constraints;
      }

      explicit AutoDiffConstraintDynamics(
        const Model & model,
        const ConstraintModelVector & contact_models,
        const std::string & filename = "casadi_contactDyn",
        const std::string & libname = "libcasadi_cg_contactDyn",
        const std::string & fun_name = "eval_f")
      : Base(model, filename, libname, fun_name)
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
      , cs_ddq(model.nv, 1)
      , cs_lambda_c(model.nv, 1)
      , q_vec((size_t)model.nq)
      , v_vec((size_t)model.nv)
      , v_int_vec((size_t)model.nv)
      , tau_vec((size_t)model.nv)
      , ddq(model.nv)
      , ddq_dq(model.nv, model.nv)
      , ddq_dv(model.nv, model.nv)
      , ddq_dtau(model.nv, model.nv)
      {
        lambda_c.resize(nc);
        lambda_c.setZero();
        dlambda_dq.resize(nc, model.nv);
        dlambda_dq.setZero();
        dlambda_dv.resize(nc, model.nv);
        dlambda_dv.setZero();
        dlambda_dtau.resize(nc, model.nv);
        dlambda_dtau.setZero();
        q_ad = Eigen::Map<ADConfigVectorType>(
          static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);
        v_int_ad = Eigen::Map<ADConfigVectorType>(
          static_cast<std::vector<ADScalar>>(cs_v_int).data(), model.nv, 1);
        v_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);
        tau_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

        Eigen::Map<TangentVectorType>(v_int_vec.data(), model.nv, 1).setZero();

        for (int k = 0; k < contact_models.size(); ++k)
        {
          ad_contact_models.push_back(contact_models[k].template cast<ADScalar>());
          ad_contact_datas.push_back(ADConstraintData(ad_contact_models[k]));
        }
      }

      virtual ~AutoDiffConstraintDynamics()
      {
      }

      virtual void buildMap()
      {
        pinocchio::initConstraintDynamics(ad_model, ad_data, ad_contact_models);
        // Integrate q + v_int = q_int
        pinocchio::integrate(ad_model, q_ad, v_int_ad, q_int_ad);
        // Run contact dynamics with new q_int
        pinocchio::constraintDynamics(
          ad_model, ad_data, q_int_ad, v_ad, tau_ad, ad_contact_models, ad_contact_datas);
        // Copy Output
        pinocchio::casadi::copy(ad_data.ddq, cs_ddq);
        pinocchio::casadi::copy(ad_data.lambda_c, cs_lambda_c);

        cs_ddq_dq = jacobian(cs_ddq, cs_v_int);
        cs_ddq_dv = jacobian(cs_ddq, cs_v);
        cs_ddq_dtau = jacobian(cs_ddq, cs_tau);

        cs_lambda_dq = jacobian(cs_lambda_c, cs_v_int);
        cs_lambda_dv = jacobian(cs_lambda_c, cs_v);
        cs_lambda_dtau = jacobian(cs_lambda_c, cs_tau);

        ad_fun =
          ADFun(fun_name, ADSVector{cs_q, cs_v_int, cs_v, cs_tau}, ADSVector{cs_ddq, cs_lambda_c});
        ad_fun_derivs = ADFun(
          fun_name + "_derivs", ADSVector{cs_q, cs_v_int, cs_v, cs_tau},
          ADSVector{cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau, cs_lambda_dq, cs_lambda_dv, cs_lambda_dtau});
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(
        const Eigen::MatrixBase<ConfigVectorType1> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(), ad_model.nq, 1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(), ad_model.nv, 1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(), ad_model.nv, 1) = tau;
        fun_output = fun(DMVector{q_vec, v_int_vec, v_vec, tau_vec});
        ddq = Eigen::Map<TangentVectorType>(
          static_cast<std::vector<Scalar>>(fun_output[0]).data(), ad_model.nv, 1);
        lambda_c = Eigen::Map<TangentVectorType>(
          static_cast<std::vector<Scalar>>(fun_output[1]).data(), nc, 1);
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalJacobian(
        const Eigen::MatrixBase<ConfigVectorType1> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(), ad_model.nq, 1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(), ad_model.nv, 1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(), ad_model.nv, 1) = tau;
        fun_output_derivs = fun_derivs(DMVector{q_vec, v_int_vec, v_vec, tau_vec});
        ddq_dq = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[0]).data(), ad_model.nv, ad_model.nv);
        ddq_dv = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[1]).data(), ad_model.nv, ad_model.nv);
        ddq_dtau = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[2]).data(), ad_model.nv, ad_model.nv);
        dlambda_dq = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[3]).data(), nc, ad_model.nv);
        dlambda_dv = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[4]).data(), nc, ad_model.nv);
        dlambda_dtau = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output_derivs[5]).data(), nc, ad_model.nv);
      }

      TangentVectorType ddq;
      VectorXs lambda_c;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau, dlambda_dq, dlambda_dv, dlambda_dtau;

    protected:
      using Base::ad_data;
      using Base::ad_model;
      using Base::cg_generated;
      using Base::filename;
      using Base::fun_name;
      using Base::libname;

      using Base::ad_fun;
      using Base::ad_fun_derivs;
      using Base::fun;
      using Base::fun_derivs;
      using Base::fun_output;
      using Base::fun_output_derivs;

      ADConstraintModelVector ad_contact_models;
      ADConstraintDataVector ad_contact_datas;

      Eigen::DenseIndex nc;
      ADScalar cs_q, cs_v, cs_tau, cs_v_int, cs_ddq, cs_lambda_c;

      // Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau, cs_lambda_dq, cs_lambda_dv, cs_lambda_dtau;

      ADConfigVectorType q_ad, q_int_ad;
      ADTangentVectorType v_ad, v_int_ad, tau_ad;

      std::vector<Scalar> q_vec, v_vec, v_int_vec, tau_vec;
    };

    template<typename _Scalar>
    struct AutoDiffConstraintDynamicsDerivatives : public AutoDiffAlgoBase<_Scalar>
    {
      typedef AutoDiffAlgoBase<_Scalar> Base;
      typedef typename Base::Scalar Scalar;
      typedef typename Base::ADScalar ADScalar;
      typedef typename Base::ADSVector ADSVector;

      typedef typename Base::TangentVectorType TangentVectorType;
      typedef typename Base::RowMatrixXs RowMatrixXs;
      typedef typename Base::VectorXs VectorXs;
      typedef typename Base::MatrixXs MatrixXs;

      typedef typename Base::ADFun ADFun;
      typedef typename Base::DMVector DMVector;
      typedef typename Base::DMMatrix DMMatrix;
      typedef typename Base::ADConfigVectorType ADConfigVectorType;
      typedef typename Base::ADTangentVectorType ADTangentVectorType;

      typedef typename pinocchio::RigidConstraintModelTpl<Scalar, Base::Options> ConstraintModel;
      typedef Eigen::aligned_allocator<ConstraintModel> ConstraintModelAllocator;
      typedef typename std::vector<ConstraintModel, ConstraintModelAllocator> ConstraintModelVector;
      typedef typename pinocchio::RigidConstraintDataTpl<Scalar, Base::Options> ConstraintData;
      typedef Eigen::aligned_allocator<ConstraintData> ConstraintDataAllocator;
      typedef typename std::vector<ConstraintData, ConstraintDataAllocator> ConstraintDataVector;

      typedef typename pinocchio::RigidConstraintModelTpl<ADScalar, Base::Options>
        ADConstraintModel;
      typedef Eigen::aligned_allocator<ADConstraintModel> ADConstraintModelAllocator;
      typedef typename std::vector<ADConstraintModel, ADConstraintModelAllocator>
        ADConstraintModelVector;
      typedef typename pinocchio::RigidConstraintDataTpl<ADScalar, Base::Options> ADConstraintData;
      typedef Eigen::aligned_allocator<ADConstraintData> ADConstraintDataAllocator;
      typedef typename std::vector<ADConstraintData, ADConstraintDataAllocator>
        ADConstraintDataVector;

      static Eigen::DenseIndex constraintDim(const ConstraintModelVector & contact_models)
      {
        Eigen::DenseIndex num_total_constraints = 0;
        for (typename ConstraintModelVector::const_iterator it = contact_models.begin();
             it != contact_models.end(); ++it)
        {
          PINOCCHIO_CHECK_INPUT_ARGUMENT(
            it->size() > 0, "The dimension of the constraint must be positive");
          num_total_constraints += it->size();
        }
        return num_total_constraints;
      }

      explicit AutoDiffConstraintDynamicsDerivatives(
        const Model & model,
        const ConstraintModelVector & contact_models,
        const std::string & filename = "casadi_contactDynDerivs",
        const std::string & libname = "libcasadi_cg_contactDynDerivs",
        const std::string & fun_name = "eval_f")
      : Base(model, filename, libname, fun_name)
      , nc(constraintDim(contact_models))
      , cs_q(ADScalar::sym("q", model.nq))
      , cs_v(ADScalar::sym("v", model.nv))
      , cs_tau(ADScalar::sym("tau", model.nv))
      , q_ad(model.nq)
      , v_ad(model.nv)
      , tau_ad(model.nv)
      , cs_ddq(model.nv, 1)
      , cs_lambda_c(model.nv, 1)
      , q_vec((size_t)model.nq)
      , v_vec((size_t)model.nv)
      , tau_vec((size_t)model.nv)
      , ddq(model.nv)
      , ddq_dq(model.nv, model.nv)
      , ddq_dv(model.nv, model.nv)
      , ddq_dtau(model.nv, model.nv)
      {
        lambda_c.resize(nc);
        lambda_c.setZero();
        dlambda_dq.resize(nc, model.nv);
        dlambda_dq.setZero();
        dlambda_dv.resize(nc, model.nv);
        dlambda_dv.setZero();
        dlambda_dtau.resize(nc, model.nv);
        dlambda_dtau.setZero();

        q_ad = Eigen::Map<ADConfigVectorType>(
          static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);
        v_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);
        tau_ad = Eigen::Map<ADTangentVectorType>(
          static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

        for (int k = 0; k < contact_models.size(); ++k)
        {
          ad_contact_models.push_back(contact_models[k].template cast<ADScalar>());
          ad_contact_datas.push_back(ADConstraintData(ad_contact_models[k]));
        }

        Base::build_jacobian = false;
      }

      virtual ~AutoDiffConstraintDynamicsDerivatives()
      {
      }

      virtual void buildMap()
      {
        pinocchio::initConstraintDynamics(ad_model, ad_data, ad_contact_models);
        pinocchio::constraintDynamics(
          ad_model, ad_data, q_ad, v_ad, tau_ad, ad_contact_models, ad_contact_datas);
        pinocchio::computeConstraintDynamicsDerivatives(
          ad_model, ad_data, ad_contact_models, ad_contact_datas);
        // Copy Output
        pinocchio::casadi::copy(ad_data.ddq, cs_ddq);
        pinocchio::casadi::copy(ad_data.lambda_c, cs_lambda_c);
        pinocchio::casadi::copy(ad_data.ddq_dq, cs_ddq_dq);
        pinocchio::casadi::copy(ad_data.ddq_dv, cs_ddq_dv);
        pinocchio::casadi::copy(ad_data.ddq_dtau, cs_ddq_dtau);
        pinocchio::casadi::copy(ad_data.dlambda_dq, cs_lambda_dq);
        pinocchio::casadi::copy(ad_data.dlambda_dv, cs_lambda_dv);
        pinocchio::casadi::copy(ad_data.dlambda_dtau, cs_lambda_dtau);

        ad_fun = ADFun(
          fun_name, ADSVector{cs_q, cs_v, cs_tau},
          ADSVector{
            cs_ddq, cs_lambda_c, cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau, cs_lambda_dq, cs_lambda_dv,
            cs_lambda_dtau});
      }

      template<typename ConfigVectorType1, typename TangentVectorType1, typename TangentVectorType2>
      void evalFunction(
        const Eigen::MatrixBase<ConfigVectorType1> & q,
        const Eigen::MatrixBase<TangentVectorType1> & v,
        const Eigen::MatrixBase<TangentVectorType2> & tau)
      {
        Eigen::Map<ConfigVectorType1>(q_vec.data(), ad_model.nq, 1) = q;
        Eigen::Map<TangentVectorType1>(v_vec.data(), ad_model.nv, 1) = v;
        Eigen::Map<TangentVectorType2>(tau_vec.data(), ad_model.nv, 1) = tau;
        fun_output = fun(DMVector{q_vec, v_vec, tau_vec});
        ddq = Eigen::Map<TangentVectorType>(
          static_cast<std::vector<Scalar>>(fun_output[0]).data(), ad_model.nv, 1);
        lambda_c = Eigen::Map<TangentVectorType>(
          static_cast<std::vector<Scalar>>(fun_output[1]).data(), nc, 1);
        ddq_dq = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[2]).data(), ad_model.nv, ad_model.nv);
        ddq_dv = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[3]).data(), ad_model.nv, ad_model.nv);
        ddq_dtau = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[4]).data(), ad_model.nv, ad_model.nv);
        dlambda_dq = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[5]).data(), nc, ad_model.nv);
        dlambda_dv = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[6]).data(), nc, ad_model.nv);
        dlambda_dtau = Eigen::Map<MatrixXs>(
          static_cast<std::vector<Scalar>>(fun_output[7]).data(), nc, ad_model.nv);
      }

      TangentVectorType ddq, lambda_c;
      RowMatrixXs ddq_dq, ddq_dv, ddq_dtau, dlambda_dq, dlambda_dv, dlambda_dtau;

    protected:
      using Base::ad_data;
      using Base::ad_model;
      using Base::cg_generated;
      using Base::filename;
      using Base::fun_name;
      using Base::libname;

      using Base::ad_fun;
      using Base::ad_fun_derivs;
      using Base::fun;
      using Base::fun_derivs;
      using Base::fun_output;
      using Base::fun_output_derivs;
      ADConstraintModelVector ad_contact_models;
      ADConstraintDataVector ad_contact_datas;

      Eigen::DenseIndex nc;
      ADScalar cs_q, cs_v, cs_tau, cs_ddq, cs_lambda_c;

      // Derivatives
      ADScalar cs_ddq_dq, cs_ddq_dv, cs_ddq_dtau, cs_lambda_dq, cs_lambda_dv, cs_lambda_dtau;

      ADConfigVectorType q_ad;
      ADTangentVectorType v_ad, tau_ad;

      std::vector<Scalar> q_vec, v_vec, tau_vec;
    };

  } // namespace casadi

} // namespace pinocchio

#endif // ifndef __pinocchio_autodiff_code_generator_algo_hpp__
