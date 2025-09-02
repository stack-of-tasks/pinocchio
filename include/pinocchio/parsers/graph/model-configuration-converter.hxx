//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_configuration_converter_hxx__
#define __pinocchio_parsers_graph_model_configuration_converter_hxx__

#include "pinocchio/macros.hpp"

#include "pinocchio/math/sincos.hpp"

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/joint/joints.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-configuration-converter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/variant.hpp>

#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace pinocchio
{

  namespace graph
  {

    namespace internal
    {
      /// Compute the ModelConfigurationConverter mapping vector.
      /// This structure use recursive methods to handle composite joint flattening.
      template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
      struct CreateConverterAlgo
      {
        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;
        typedef typename ModelConfigurationConverter::JointMapping JointMapping;
        typedef JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> JointModelComposite;
        typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;

        /// Add all the joints from a Model to the mapping.
        void addJointFromModel(
          const ModelTpl<Scalar, Options, JointCollectionTpl> & model_source,
          const ModelTpl<Scalar, Options, JointCollectionTpl> & model_target,
          const std::unordered_map<std::string, bool> & joint_direction_source,
          const std::unordered_map<std::string, bool> & joint_direction_target,
          std::size_t index_source)
        {
          for (; index_source < model_source.joints.size(); ++index_source)
          {
            const auto & joint_model_source = model_source.joints[index_source];
            const std::string & joint_name = model_source.names[index_source];
            auto index_target = model_target.getJointId(joint_name);
            bool same_direction =
              joint_direction_source.at(joint_name) == joint_direction_target.at(joint_name);
            const JointModelComposite * joint_composite_source =
              boost::get<JointModelComposite>(&joint_model_source);
            if (joint_composite_source == nullptr)
            {
              ConfigurationMapping configuration;
              TangentMapping tangent;

              configuration.idx_qs_source = model_source.idx_qs[index_source];
              configuration.idx_qs_target = model_target.idx_qs[index_target];
              configuration.nq = model_source.nqs[index_source];

              tangent.idx_vs_source = model_source.idx_vs[index_source];
              tangent.idx_vs_target = model_target.idx_vs[index_target];
              tangent.nv = model_source.nvs[index_source];

              configuration_mapping.push_back(configuration);
              tangent_mapping.push_back(tangent);
              joint_mapping.emplace_back(joint_model_source, same_direction);
            }
            else
            {
              const auto & joint_model_target = model_target.joints[index_target];
              const JointModelComposite & joint_composite_target =
                boost::get<JointModelComposite>(joint_model_target);
              addJointFromComposite(
                joint_composite_source->joints, joint_composite_target.joints, same_direction);
            }
          }
        }

        /// Add all the joints from a JointComposite to the mapping.
        void addJointFromComposite(
          const typename JointModelComposite::JointModelVector & joints_source,
          const typename JointModelComposite::JointModelVector & joints_target,
          bool same_direction)
        {
          if (same_direction)
          {
            for (std::size_t index_source = 0; index_source < joints_source.size(); ++index_source)
            {
              const auto & joint_model_source = joints_source[index_source];
              const auto & joint_model_target = joints_target[index_source];
              addJointModel(joint_model_source, joint_model_target, same_direction);
            }
          }
          else
          {
            for (std::size_t index_source = 0, index_target = joints_target.size() - 1;
                 index_source < joints_source.size(); ++index_source, --index_target)
            {
              const auto & joint_model_source = joints_source[index_source];
              const auto & joint_model_target = joints_target[index_target];
              addJointModel(joint_model_source, joint_model_target, same_direction);
            }
          }
        }

        /// Add a JointModel to the mapping.
        void addJointModel(
          const JointModel & joint_model_source,
          const JointModel & joint_model_target,
          bool same_direction)
        {
          const JointModelComposite * joint_composite_source =
            boost::get<JointModelComposite>(&joint_model_source);
          if (joint_composite_source == nullptr)
          {
            ConfigurationMapping configuration;
            TangentMapping tangent;
            configuration.idx_qs_source = joint_model_source.idx_q();
            configuration.idx_qs_target = joint_model_target.idx_q();
            configuration.nq = joint_model_source.nq();

            tangent.idx_vs_source = joint_model_source.idx_v();
            tangent.idx_vs_target = joint_model_target.idx_v();
            tangent.nv = joint_model_source.nv();

            configuration_mapping.push_back(configuration);
            tangent_mapping.push_back(tangent);
            joint_mapping.emplace_back(joint_model_source, same_direction);
          }
          else
          {
            const JointModelComposite & joint_composite_target =
              boost::get<JointModelComposite>(joint_model_target);
            addJointFromComposite(
              joint_composite_source->joints, joint_composite_target.joints, same_direction);
          }
        }

        std::vector<ConfigurationMapping> configuration_mapping;
        std::vector<TangentMapping> tangent_mapping;
        std::vector<JointMapping> joint_mapping;
      };
    } // namespace internal

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl> createConverter(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_source,
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_target,
      const ModelGraphBuildInfo & build_info_source,
      const ModelGraphBuildInfo & build_info_target)
    {
      typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
        ModelConfigurationConverter;
      typedef typename ModelConfigurationConverter::JointMapping JointMapping;

      std::vector<internal::ConfigurationMapping> configuration_mapping;
      std::vector<internal::TangentMapping> tangent_mapping;
      std::vector<JointMapping> joint_mapping;

      // Construct the mapping between source and target configuration and tangent vector.
      // If source model doesn't have a fixed base, we skip the first joint (usually a FF joint)
      // that can not be in the target model.
      std::size_t index_source = 1;
      if (!build_info_source._is_fixed)
      {
        index_source = 2;
      }
      internal::CreateConverterAlgo<Scalar, Options, JointCollectionTpl> algo;
      algo.addJointFromModel(
        model_source, model_target, build_info_source._joint_forward,
        build_info_target._joint_forward, index_source);

      return ModelConfigurationConverter(
        algo.configuration_mapping, algo.tangent_mapping, algo.joint_mapping, model_source.nq,
        model_source.nv, model_target.nq, model_target.nv);
    }

    namespace internal
    {
      template<typename Scalar, int Options, bool value = is_floating_point<Scalar>::value>
      struct ReverseZYXEulerAngle
      {
        typedef Eigen::Vector<Scalar, 3> Vector3;

        template<typename ConfigVectorType>
        static Vector3 run(const Eigen::MatrixBase<ConfigVectorType> & q_source)
        {
          JointModelSphericalZYXTpl<Scalar, Options> jmodel;
          jmodel.setIndexes(0, 0, 0);
          JointDataSphericalZYXTpl<Scalar, Options> jdata;
          jmodel.calc(jdata, q_source);
          return jdata.M.rotation().transpose().eulerAngles(2, 1, 0);
        }
      };

      template<typename Scalar, int Options>
      struct ReverseZYXEulerAngle<Scalar, Options, false>
      {
        typedef Eigen::Vector<Scalar, 3> Vector3;

        template<typename ConfigVectorType>
        static Vector3 run(const Eigen::MatrixBase<ConfigVectorType> &)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "convertConfiguration - JointModelSphericalZYX can only be reversed with real values");
        }
      };

      /// Reverse ZYX joint.
      /// This implementation avoid compilation issues with Casadi scalar type.
      template<typename Scalar, int Options, typename ConfigVectorType>
      static Eigen::Vector<Scalar, 3>
      reverseZYXEulerAngle(const Eigen::MatrixBase<ConfigVectorType> & q_source)
      {
        return ReverseZYXEulerAngle<Scalar, Options>::run(q_source);
      }

      template<typename Scalar, int Options, bool value = is_floating_point<Scalar>::value>
      struct ReverseZYXEulerAngleMotion
      {
        typedef Eigen::Vector<Scalar, 3> Vector3;
        typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

        template<typename ConfigVectorType1, typename ConfigVectorType2>
        static Vector3 run(
          const Eigen::MatrixBase<ConfigVectorType1> & q_source,
          const Eigen::MatrixBase<ConfigVectorType2> & v_source)
        {
          // JointModelSphericalZYXTpl velocity is computed with
          // v_j^{source} = S_{source} \alpha_{source}.
          // We want the reverse joint to compute
          // v_j^{target} = -X_{source-to-target} v_j^{source}.
          // Since v_j^{target} = S_{target} \alpha_{target}, then
          // \alpha_{target} = -S_{target}^{-1} X_{source-to-target} v_j^{source}.
          // Compute v_target in the right frame
          JointModelSphericalZYXTpl<Scalar, Options> jmodel;
          jmodel.setIndexes(0, 0, 0);
          JointDataSphericalZYXTpl<Scalar, Options> jdata;
          jmodel.calc(jdata, q_source, v_source);
          Vector3 eulers = jdata.M.rotation().transpose().eulerAngles(2, 1, 0);

          // Compute S_target^{-1}
          Scalar c1_target, s1_target;
          SINCOS(eulers[1], &s1_target, &c1_target);
          Scalar c2_target, s2_target;
          SINCOS(eulers[2], &s2_target, &c2_target);
          Matrix3 S_target;
          S_target << -s1_target, Scalar(0), Scalar(1), c1_target * s2_target, c2_target, Scalar(0),
            c1_target * c2_target, -s2_target, Scalar(0);
          Matrix3 S_target_inv = S_target.inverse();

          // Compute v_target in the right frame
          return -S_target_inv * jdata.M.rotation() * jdata.v.angular();
        }
      };

      template<typename Scalar, int Options>
      struct ReverseZYXEulerAngleMotion<Scalar, Options, false>
      {
        typedef Eigen::Vector<Scalar, 3> Vector3;

        template<typename ConfigVectorType1, typename ConfigVectorType2>
        static Vector3 run(
          const Eigen::MatrixBase<ConfigVectorType1> &,
          const Eigen::MatrixBase<ConfigVectorType2> &)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "convertConfiguration - JointModelSphericalZYX can only be reversed with real values");
        }
      };

      /// Reverse ZYX joint configuration and motion.
      /// This implementation avoid compilation issues with Casadi scalar type.
      template<typename Scalar, int Options, typename ConfigVectorType1, typename ConfigVectorType2>
      static Eigen::Vector<Scalar, 3> reverseZYXEulerAngleMotion(
        const Eigen::MatrixBase<ConfigVectorType1> & q_source,
        const Eigen::MatrixBase<ConfigVectorType2> & v_source)
      {
        return ReverseZYXEulerAngleMotion<Scalar, Options>::run(q_source, v_source);
      }

      template<
        typename _Scalar,
        int _Options,
        template<typename, int> class JointCollectionTpl,
        typename ConfigVectorType1,
        typename ConfigVectorType2>
      struct ConfigurationConverterVisitor : public boost::static_visitor<void>
      {
        typedef _Scalar Scalar;
        enum
        {
          Options = _Options
        };

        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;
        typedef typename ModelConfigurationConverter::JointMapping JointMapping;

        typedef Eigen::Vector<Scalar, 2> Vector2;
        typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
        typedef Eigen::Vector<Scalar, 3> Vector3;
        typedef Eigen::Quaternion<Scalar> Quaternion;

        typedef void ReturnType;

        const Eigen::MatrixBase<ConfigVectorType1> & q_source;
        ConfigVectorType2 & q_target;
        const ConfigurationMapping & configuration;
        const JointMapping & joint;

        ConfigurationConverterVisitor(
          const Eigen::MatrixBase<ConfigVectorType1> & q_source,
          const Eigen::MatrixBase<ConfigVectorType2> & q_target,
          const ConfigurationMapping & configuration,
          const JointMapping & joint)
        : q_source(q_source)
        , q_target(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType2, q_target))
        , configuration(configuration)
        , joint(joint)
        {
        }

        // Manage Revolute, Prismatic, Translation and Helical joint.
        template<typename JointType>
        ReturnType operator()(const JointType &) const
        {
          // Apply direction_sign on each configuration values.
          q_target.segment(configuration.idx_qs_target, configuration.nq).noalias() =
            joint.direction_sign * q_source.segment(configuration.idx_qs_source, configuration.nq);
        }

        template<int axis>
        ReturnType operator()(const JointModelRevoluteUnboundedTpl<Scalar, Options, axis> &) const
        {
          // Apply direction_sign on sinus to inverse the rotation
          q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source];
          q_target[configuration.idx_qs_target + 1] =
            joint.direction_sign * q_source[configuration.idx_qs_source + 1];
        }

        ReturnType
        operator()(const JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> &) const
        {
          // Apply direction_sign on sinus to inverse the rotation
          q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source];
          q_target[configuration.idx_qs_target + 1] =
            joint.direction_sign * q_source[configuration.idx_qs_source + 1];
        }

        ReturnType operator()(const JointModelFreeFlyerTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy tx, ty, tz, qx, qy, qz, qw
            q_target.template segment<7>(configuration.idx_qs_target) =
              q_source.template segment<7>(configuration.idx_qs_source);
          }
          else
          {
            // Apply inverse rotation on translation and copy inverse rotation
            Vector3 translation_source(q_source.template segment<3>(configuration.idx_qs_source));
            Quaternion rotation_source(
              q_source.template segment<4>(configuration.idx_qs_source + 3));
            Quaternion rotation_source_inv(rotation_source.conjugate());

            Vector3 translation_target(-(rotation_source_inv * translation_source));
            q_target.template segment<3>(configuration.idx_qs_target) = translation_target;
            q_target.template segment<4>(configuration.idx_qs_target + 3) =
              rotation_source_inv.coeffs();
          }
        }

        ReturnType operator()(const JointModelSphericalTpl<Scalar, Options> &) const
        {
          // Copy qx, qy, qz with direction_sign apply to it
          q_target.template segment<3>(configuration.idx_qs_target).noalias() =
            joint.direction_sign * q_source.template segment<3>(configuration.idx_qs_source);
          // Copy qw
          q_target[configuration.idx_qs_target + 3] = q_source[configuration.idx_qs_source + 3];
        }

        ReturnType operator()(const JointModelSphericalZYXTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy zyx
            q_target.template segment<3>(configuration.idx_qs_target) =
              q_source.template segment<3>(configuration.idx_qs_source);
          }
          else
          {
            // Compute the inverse rotation and extract the ZYX euler angles
            JointModelSphericalZYXTpl<Scalar, Options> jmodel;
            jmodel.setIndexes(0, 0, 0);
            JointDataSphericalZYXTpl<Scalar, Options> jdata;
            jmodel.calc(jdata, q_source.template segment<3>(configuration.idx_qs_source));
            q_target.template segment<3>(configuration.idx_qs_target) =
              reverseZYXEulerAngle<Scalar, Options>(
                q_source.template segment<3>(configuration.idx_qs_source));
          }
        }

        ReturnType operator()(const JointModelPlanarTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy x, y, cos_theta, sin_theta
            q_target.template segment<4>(configuration.idx_qs_target) =
              q_source.template segment<4>(configuration.idx_qs_source);
          }
          else
          {
            Scalar c_theta_source = q_source[configuration.idx_qs_source + 2];
            Scalar s_theta_source = q_source[configuration.idx_qs_source + 3];
            Matrix2 rotation_source_inv;
            rotation_source_inv << c_theta_source, s_theta_source, -s_theta_source, c_theta_source;
            q_target.template segment<2>(configuration.idx_qs_target).noalias() =
              -rotation_source_inv * q_source.template segment<2>(configuration.idx_qs_source);
            q_target[configuration.idx_qs_target + 2] = c_theta_source;
            q_target[configuration.idx_qs_target + 3] = -s_theta_source;
          }
        }

        ReturnType operator()(const JointModelUniversalTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy q
            q_target.template segment<2>(configuration.idx_qs_target) =
              q_source.template segment<2>(configuration.idx_qs_source);
          }
          else
          {
            // Axes are inversed so swap q
            q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source + 1];
            q_target[configuration.idx_qs_target + 1] = q_source[configuration.idx_qs_source];
          }
        }

        ReturnType operator()(const JointModelMimicTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          // Nothing to do, q conversion is managed in mimicked joint.
        }

        ReturnType
        operator()(const JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          assert(false && "This must never happened");
        }
      };
    } // namespace internal

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    template<typename ConfigVectorType1, typename ConfigVectorType2>
    void
    ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>::convertConfigurationVector(
      const Eigen::MatrixBase<ConfigVectorType1> & q_source,
      const Eigen::MatrixBase<ConfigVectorType2> & q_target) const
    {
      if (_source_configuration_size != q_source.size())
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "convertConfiguration - wrong source configuration size");
      }
      if (_target_configuration_size != q_target.size())
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "convertConfiguration - wrong target configuration size");
      }

      for (std::size_t i = 0; i < _joint_mapping.size(); ++i)
      {
        const auto & configuration = _configuration_mapping[i];
        const auto & joint = _joint_mapping[i];
        boost::apply_visitor(
          internal::ConfigurationConverterVisitor<
            Scalar, Options, JointCollectionTpl, ConfigVectorType1, ConfigVectorType2>(
            q_source, q_target, configuration, joint),
          joint.joint);
      }
    }

    namespace internal
    {
      template<typename Vector6ArgType>
      MotionRef<Vector6ArgType> makeMotionRef(const Vector6ArgType & vector)
      {
        return MotionRef<Vector6ArgType>(vector);
      }

      template<
        typename _Scalar,
        int _Options,
        template<typename, int> class JointCollectionTpl,
        typename ConfigVectorType,
        typename TangentVectorType1,
        typename TangentVectorType2>
      struct TangentConverterVisitor : public boost::static_visitor<void>
      {
        typedef _Scalar Scalar;
        enum
        {
          Options = _Options
        };

        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;
        typedef typename ModelConfigurationConverter::JointMapping JointMapping;

        typedef Eigen::Vector<Scalar, 2> Vector2;
        typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
        typedef Eigen::Vector<Scalar, 3> Vector3;
        typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
        typedef Eigen::Quaternion<Scalar> Quaternion;

        typedef SE3Tpl<Scalar, Options> SE3;
        typedef MotionSphericalTpl<Scalar, Options> MotionSpherical;
        typedef MotionTpl<Scalar, Options> Motion;

        typedef void ReturnType;

        const Eigen::MatrixBase<ConfigVectorType> & q_source;
        const Eigen::MatrixBase<TangentVectorType1> & v_source;
        TangentVectorType2 & v_target;
        const ConfigurationMapping & configuration;
        const TangentMapping & tangent;
        const JointMapping & joint;

        TangentConverterVisitor(
          const Eigen::MatrixBase<ConfigVectorType> & q_source,
          const Eigen::MatrixBase<TangentVectorType1> & v_source,
          const Eigen::MatrixBase<TangentVectorType2> & v_target,
          const ConfigurationMapping & configuration,
          const TangentMapping & tangent,
          const JointMapping & joint)
        : q_source(q_source)
        , v_source(v_source)
        , v_target(PINOCCHIO_EIGEN_CONST_CAST(TangentVectorType2, v_target))
        , configuration(configuration)
        , tangent(tangent)
        , joint(joint)
        {
        }

        // Manage Revolute, RevoluteUnbounded, Prismatic, Translation and Helical joint.
        template<typename JointType>
        ReturnType operator()(const JointType &) const
        {
          // Apply direction_sign on each configuration values.
          v_target.template segment<JointType::NV>(tangent.idx_vs_target).noalias() =
            joint.direction_sign * v_source.template segment<JointType::NV>(tangent.idx_vs_source);
        }

        ReturnType operator()(const JointModelFreeFlyerTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy vx, vy, vz, wx, wy, wz
            v_target.template segment<6>(tangent.idx_vs_target) =
              v_source.template segment<6>(tangent.idx_vs_source);
          }
          else
          {
            // Velocities must be expressed in target model joint successor frame (F_s)
            // (after joint transformation).
            // Here, \alpha_{source} is in source model F_s, but in target model joint
            // predecessor frame (F_p).
            // With X_{p_to_s}^{target} the transformation from the predecessor to the successor
            // frame in target model, then \alpha_{target} = -X_{p_to_s}^{target} \alpha_{source}
            Quaternion rotation_source(
              q_source.template segment<4>(configuration.idx_qs_source + 3));
            Vector3 translation_source(q_source.template segment<3>(configuration.idx_qs_source));
            SE3 transform_source(rotation_source, translation_source);

            v_target.template segment<6>(tangent.idx_vs_target).noalias() =
              -transform_source
                 .act(makeMotionRef(v_source.template segment<6>(tangent.idx_vs_source)))
                 .toVector();
          }
        }

        ReturnType operator()(const JointModelSphericalTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // vz, wx, wy, wz
            v_target.template segment<3>(tangent.idx_vs_target) =
              v_source.template segment<3>(tangent.idx_vs_source);
          }
          else
          {
            // Velocities must be expressed in target model joint successor frame (F_s)
            // (after joint transformation).
            // Here, \alpha_{source} is in source model F_s, but in target model joint
            // predecessor frame (F_p).
            // With X_{p_to_s}^{target} the transformation from the predecessor to the successor
            // frame in target model, then \alpha_{target} = -X_{p_to_s}^{target} \alpha_{source}
            Quaternion rotation_source(q_source.template segment<4>(configuration.idx_qs_source));
            SE3 transform_source(rotation_source, Vector3::Zero());

            v_target.template segment<3>(tangent.idx_vs_target).noalias() =
              -transform_source
                 .act(MotionSpherical(v_source.template segment<3>(tangent.idx_vs_source)))
                 .angular();
          }
        }

        ReturnType operator()(const JointModelSphericalZYXTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy vz, vy, vx
            v_target.template segment<3>(tangent.idx_vs_target) =
              v_source.template segment<3>(tangent.idx_vs_source);
          }
          else
          {
            v_target.template segment<3>(tangent.idx_vs_target).noalias() =
              reverseZYXEulerAngleMotion<Scalar, Options>(
                q_source.template segment<3>(configuration.idx_qs_source),
                v_source.template segment<3>(tangent.idx_vs_source));
          }
        }

        ReturnType operator()(const JointModelPlanarTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy vx, vy, wz
            v_target.template segment<3>(tangent.idx_vs_target) =
              v_source.template segment<3>(tangent.idx_vs_source);
          }
          else
          {
            // Velocities must be expressed in target model joint successor frame (F_s)
            // (after joint transformation).
            // Here, \alpha_{source} is in source model F_s, but in target model joint
            // predecessor frame (F_p).
            // With X_{p_to_s}^{target} the transformation from the predecessor to the successor
            // frame in target model, then \alpha_{target} = -X_{p_to_s}^{target} \alpha_{source}
            JointModelPlanarTpl<Scalar, Options> jmodel;
            jmodel.setIndexes(0, 0, 0);
            JointDataPlanarTpl<Scalar, Options> jdata;
            jmodel.calc(
              jdata, q_source.template segment<4>(configuration.idx_qs_source),
              v_source.template segment<3>(tangent.idx_vs_source));
            Motion res = -jdata.M.act(jdata.v);
            v_target.template segment<3>(tangent.idx_vs_target) << res.linear().x(),
              res.linear().y(), res.angular().z();
          }
        }

        ReturnType operator()(const JointModelUniversalTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy v
            v_target.template segment<2>(tangent.idx_vs_target) =
              v_source.template segment<2>(tangent.idx_vs_source);
          }
          else
          {
            // Axes are inversed so swap v
            v_target[tangent.idx_vs_target] = v_source[tangent.idx_vs_source + 1];
            v_target[tangent.idx_vs_target + 1] = v_source[tangent.idx_vs_source];
          }
        }

        ReturnType operator()(const JointModelMimicTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          // Nothing to do, v conversion is managed in mimicked joint.
        }

        ReturnType
        operator()(const JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          assert(false && "This must never happened");
        }
      };
    } // namespace internal

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    template<typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
    void ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>::convertTangentVector(
      const Eigen::MatrixBase<ConfigVectorType> & q_source,
      const Eigen::MatrixBase<TangentVectorType1> & v_source,
      const Eigen::MatrixBase<TangentVectorType2> & v_target) const
    {
      if (_source_configuration_size != q_source.size())
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "convertTangent - wrong source configuration size");
      }
      if (_source_tangent_size != v_source.size())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "convertTangent - wrong source tangent size");
      }
      if (_target_tangent_size != v_target.size())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "convertTangent - wrong target tangent size");
      }

      for (std::size_t i = 0; i < _joint_mapping.size(); ++i)
      {
        const auto & configuration = _configuration_mapping[i];
        const auto & tangent = _tangent_mapping[i];
        const auto & joint = _joint_mapping[i];
        boost::apply_visitor(
          internal::TangentConverterVisitor<
            Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
            TangentVectorType2>(q_source, v_source, v_target, configuration, tangent, joint),
          joint.joint);
      }
    }

  } // namespace graph

} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_model_configuration_converter_hxx__
