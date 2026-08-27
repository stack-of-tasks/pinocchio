//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace buildModels
  {
    /** \brief Create a 6-DOF kinematic chain shoulder-elbow-wrist.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void
    manipulator(ModelTpl<Scalar, Options, JointCollectionTpl> & model, const bool mimic = false);

#ifdef PINOCCHIO_WITH_COLLISION
    /** \brief Create the geometries on top of the kinematic model created by manipulator function.
     *
     * \param model, const, kinematic chain typically produced by the function manipulator(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after manipulator(model).
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void manipulatorGeometries(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model, GeometryModel & geom);
#endif

    /** \brief Create a 28-DOF kinematic chain of a floating humanoid robot.
     *
     * The kinematic chain has four 6-DOF limbs shoulder-elbow-wrist, one 2-DOF torso, one
     * 2-DOF neck. The float joint is either a JointModelFreeFloating (with nq=7 and nv=6),
     * or a composite joint with a JointModelTranslation
     * and a roll-pitch-yaw joint JointModelSphericalZYX (with total nq=nv=6).
     * Using a free-floating or a composite joint is decided by the boolean usingFF.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint. This changes the size of the configuration space (35 vs 34).
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void humanoid(ModelTpl<Scalar, Options, JointCollectionTpl> & model, bool usingFF = true);

#ifdef PINOCCHIO_WITH_COLLISION
    /** \brief Create the geometries on top of the kinematic model created by humanoid function.
     *
     * \param model, const, kinematic chain typically produced by the function humanoid(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after humanoid(model).
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void humanoidGeometries(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model, GeometryModel & geom);
#endif

    /** \brief Create a humanoid kinematic tree with 6-DOF limbs and random joint placements.
     *
     * This method is only meant to be used in unittest. Due to random placement and masses,
     * the resulting model is likely to not correspond to any physically-plausible model.
     * You may want to consider pinocchio::humanoid and pinocchio::humanoidGeometries functions that
     * rather define a plain and non-random model.
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint translation + roll-pitch-yaw.
     * This changes the size of the configuration space (33 vs 32).
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void humanoidRandom(
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      bool usingFF = true,
      bool mimic = false);

  } // namespace buildModels
} // namespace pinocchio

namespace pinocchio
{
  namespace buildModels
  {
    namespace details
    {
      template<
        typename Scalar,
        int Options,
        template<typename, int> class JointCollectionTpl,
        typename JointModel>
      static JointIndex addJointAndBody(
        ModelTpl<Scalar, Options, JointCollectionTpl> & model,
        const JointModelBase<JointModel> & joint,
        const std::string & parent_name,
        const std::string & name,
        const typename ModelTpl<Scalar, Options, JointCollectionTpl>::SE3 & placement =
          ModelTpl<Scalar, Options, JointCollectionTpl>::SE3::Random(),
        bool setRandomLimits = true)
      {
        typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
        typedef typename Model::Inertia Inertia;
        typedef typename Model::SE3 SE3;
        typedef typename JointModel::ConfigVector_t CV;
        typedef typename JointModel::TangentVector_t TV;

        CV qmin = CV::Constant(joint.nq(), -3.14), qmax = CV::Constant(joint.nq(), 3.14);
        TV vmax = TV::Constant(joint.nv(), 10), taumax = TV::Constant(joint.nv(), 10);

        JointIndex idx;

        if (setRandomLimits)
          idx = model.addJoint(
            model.getJointId(parent_name), joint, placement, name + "_joint",
            TV::Random(joint.nv(), 1) - TV::Constant(joint.nv(), 1, 1), // min_effort
            TV::Random(joint.nv(), 1) + TV::Constant(joint.nv(), 1, 1), // max_effort

            TV::Random(joint.nv(), 1) - TV::Constant(joint.nv(), 1, 1), // min_velocity
            TV::Random(joint.nv(), 1) + TV::Constant(joint.nv(), 1, 1), // max_velocity

            CV::Random(joint.nq(), 1) - CV::Constant(joint.nq(), 1, 1), // min_config
            CV::Random(joint.nq(), 1) + CV::Constant(joint.nq(), 1, 1), // max_config

            CV::Random(joint.nq(), 1) + CV::Constant(joint.nq(), 1, 1), // config_limit_margin
            TV::Random(joint.nv(), 1) - TV::Constant(joint.nv(), 1, 1), // min_friction
            TV::Random(joint.nv(), 1) + TV::Constant(joint.nv(), 1, 1), // max_friction
            TV::Constant(joint.nv(), 0),                                // damping

            TV::Random(joint.nv(), 1) - TV::Constant(joint.nv(), 1, 1), // min_acceleration
            TV::Random(joint.nv(), 1) + TV::Constant(joint.nv(), 1, 1), // max_acceleration

            TV::Random(joint.nv(), 1) - TV::Constant(joint.nv(), 1, 1), // min_jerk
            TV::Random(joint.nv(), 1) + TV::Constant(joint.nv(), 1, 1)  // max_jerk
          );
        else
          idx = model.addJoint(
            model.getJointId(parent_name), joint, placement, name + "_joint", taumax, vmax, qmin,
            qmax);

        model.addJointFrame(idx);

        model.appendBodyToJoint(idx, Inertia::Random(), SE3::Identity());
        model.addBodyFrame(name + "_body", idx);
        return idx;
      }

      template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
      static void addManipulator(
        ModelTpl<Scalar, Options, JointCollectionTpl> & model,
        const bool mimic = false,
        typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex root_joint_idx = 0,
        const typename ModelTpl<Scalar, Options, JointCollectionTpl>::SE3 & Mroot =
          ModelTpl<Scalar, Options, JointCollectionTpl>::SE3::Identity(),
        const std::string & pre = "")
      {
        typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
        typedef typename Model::JointIndex JointIndex;
        typedef typename Model::SE3 SE3;
        typedef typename Model::Inertia Inertia;

        typedef JointCollectionTpl<Scalar, Options> JC;
        static const SE3 Marm(SE3::Matrix3::Identity(), SE3::Vector3::UnitZ());
        static const SE3 Id4 = SE3::Identity();
        static const Inertia Ijoint(
          .1, Inertia::Vector3::Zero(), Inertia::Matrix3::Identity() * .01);
        static const Inertia Iarm(
          1., typename Inertia::Vector3(0, 0, .5), Inertia::Matrix3::Identity());
        static const Scalar qmin = -3.14, qmax = 3.14;
        static const Scalar vmax = 10., taumax = 10.;

        JointIndex joint_id;

        const std::string & root_joint_name = model.names[root_joint_idx];
        joint_id = addJointAndBody(
          model, typename JC::JointModelRX(), root_joint_name, pre + "shoulder1", Mroot);
        model.inertias[joint_id] = Ijoint;
        const JointIndex root_joint_id = joint_id;

        joint_id = addJointAndBody(
          model, typename JC::JointModelRY(), model.names[joint_id], pre + "shoulder2", Id4);
        model.inertias[joint_id] = Ijoint;

        joint_id = addJointAndBody(
          model, typename JC::JointModelRZ(), model.names[joint_id], pre + "shoulder3", Id4);
        model.inertias[joint_id] = Iarm;
        model.addBodyFrame(pre + "upperarm_body", joint_id);

        joint_id = addJointAndBody(
          model, typename JC::JointModelRY(), model.names[joint_id], pre + "elbow", Marm);
        model.inertias[joint_id] = Iarm;
        model.addBodyFrame(pre + "lowerarm_body", joint_id);
        model.addBodyFrame(pre + "elbow_body", joint_id);

        joint_id = addJointAndBody(
          model, typename JC::JointModelRX(), model.names[joint_id], pre + "wrist1", Marm);
        model.inertias[joint_id] = Ijoint;

        if (mimic)
        {
          // Scalar multiplier = JC::JointModelRX::ConfigVector_t::Random(1)(0);
          // Scalar offset = JC::JointModelRX::ConfigVector_t::Random(1)(0);

          Scalar multiplier = 2.5;
          Scalar offset = 0.75;
          joint_id = addJointAndBody(
            model,
            typename JC::JointModelMimic(
              typename JC::JointModelRY(), model.joints[joint_id].derived(), multiplier, offset),
            model.names[joint_id], pre + "wrist1_mimic", Id4);
        }
        else
        {
          joint_id = addJointAndBody(
            model, typename JC::JointModelRY(), model.names[joint_id], pre + "wrist2", Id4);
        }

        model.inertias[joint_id] = Iarm;
        model.addBodyFrame(pre + "effector_body", joint_id);
        const int nq = mimic ? 5 : 6;

        const typename Model::JointModel & base_joint = model.joints[root_joint_id];
        const int idx_q = base_joint.idx_q();
        const int idx_v = base_joint.idx_v();

        model.lowerPositionLimit.segment(idx_q, nq).fill(qmin);
        model.upperPositionLimit.segment(idx_q, nq).fill(qmax);
        model.positionLimitMargin.segment(idx_q, nq).fill(0);
        model.lowerVelocityLimit.segment(idx_v, nq).fill(vmax);
        model.upperVelocityLimit.segment(idx_v, nq).fill(vmax);
        model.lowerEffortLimit.segment(idx_v, nq).fill(taumax);
        model.upperEffortLimit.segment(idx_v, nq).fill(taumax);
        model.lowerDryFrictionLimit.segment(idx_v, nq).fill(0);
        model.upperDryFrictionLimit.segment(idx_v, nq).fill(0);
        model.damping.segment(idx_v, nq).fill(0);
        const Scalar inf = std::numeric_limits<Scalar>::infinity();
        model.lowerAccelerationLimit.segment(idx_v, nq).fill(-inf);
        model.upperAccelerationLimit.segment(idx_v, nq).fill(inf);
        model.lowerJerkLimit.segment(idx_v, nq).fill(-inf);
        model.upperJerkLimit.segment(idx_v, nq).fill(inf);
      }

#ifdef PINOCCHIO_WITH_COLLISION
      /* Add a 6DOF manipulator shoulder-elbow-wrist geometries to an existing model.
       * <model> is the the kinematic chain, constant.
       * <geom> is the geometry model where the new geoms are added.
       * <pre> is the prefix (string) before every name in the model.
       */
      template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
      static void addManipulatorGeometries(
        const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
        GeometryModel & geom,
        const std::string & pre = "")
      {
        typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
        typedef typename Model::FrameIndex FrameIndex;
        typedef typename GeometryObject::SE3 SE3;

        const Eigen::Vector4d meshColor(1., 1., 0.78, 1.0);

        FrameIndex parentFrame;

        parentFrame = model.getBodyId(pre + "shoulder1_body");
        GeometryObject shoulderBall(
          pre + "shoulder_object", model.frames[parentFrame].parentJoint, parentFrame,
          SE3::Identity(), std::shared_ptr<coal::Sphere>(new coal::Sphere(0.05)), "SPHERE",
          Eigen::Vector3d::Ones(), false, meshColor);
        geom.addGeometryObject(shoulderBall);

        parentFrame = model.getBodyId(pre + "elbow_body");
        GeometryObject elbowBall(
          pre + "elbow_object", model.frames[parentFrame].parentJoint, parentFrame, SE3::Identity(),
          std::shared_ptr<coal::Sphere>(new coal::Sphere(0.05)), "SPHERE", Eigen::Vector3d::Ones(),
          false, meshColor);
        geom.addGeometryObject(elbowBall);

        parentFrame = model.getBodyId(pre + "wrist1_body");
        GeometryObject wristBall(
          pre + "wrist_object", model.frames[parentFrame].parentJoint, parentFrame, SE3::Identity(),
          std::shared_ptr<coal::Sphere>(new coal::Sphere(0.05)), "SPHERE", Eigen::Vector3d::Ones(),
          false, meshColor);
        geom.addGeometryObject(wristBall);

        parentFrame = model.getBodyId(pre + "upperarm_body");
        GeometryObject upperArm(
          pre + "upperarm_object", model.frames[parentFrame].parentJoint, parentFrame,
          SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0, 0, 0.5)),
          std::shared_ptr<coal::Capsule>(new coal::Capsule(0.05, .8)), "CAPSULE",
          Eigen::Vector3d::Ones(), false, meshColor);
        geom.addGeometryObject(upperArm);

        parentFrame = model.getBodyId(pre + "lowerarm_body");
        GeometryObject lowerArm(
          pre + "lowerarm_object", model.frames[parentFrame].parentJoint, parentFrame,
          SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0, 0, 0.5)),
          std::shared_ptr<coal::Capsule>(new coal::Capsule(0.05, .8)), "CAPSULE",
          Eigen::Vector3d::Ones(), false, meshColor);
        geom.addGeometryObject(lowerArm);

        parentFrame = model.getBodyId(pre + "effector_body");
        GeometryObject effectorArm(
          pre + "effector_object", model.frames[parentFrame].parentJoint, parentFrame,
          SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0, 0, 0.1)),
          std::shared_ptr<coal::Capsule>(new coal::Capsule(0.05, .2)), "CAPSULE",
          Eigen::Vector3d::Ones(), false, meshColor);
        geom.addGeometryObject(effectorArm);
      }
#endif

      template<typename Vector3Like>
      static typename Eigen::AngleAxis<typename Vector3Like::Scalar>::Matrix3
      rotate(const typename Vector3Like::Scalar angle, const Eigen::MatrixBase<Vector3Like> & axis)
      {
        typedef typename Vector3Like::Scalar Scalar;
        typedef Eigen::AngleAxis<Scalar> AngleAxis;

        return AngleAxis(angle, axis).toRotationMatrix();
      }

    } // namespace details

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void manipulator(ModelTpl<Scalar, Options, JointCollectionTpl> & model, const bool mimic)
    {
      details::addManipulator(model, mimic);
    }

#ifdef PINOCCHIO_WITH_COLLISION
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void manipulatorGeometries(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model, GeometryModel & geom)
    {
      details::addManipulatorGeometries(model, geom);
    }
#endif

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void
    humanoidRandom(ModelTpl<Scalar, Options, JointCollectionTpl> & model, bool usingFF, bool mimic)
    {
      typedef JointCollectionTpl<Scalar, Options> JC;
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::SE3 SE3;
      using details::addJointAndBody;
      static const SE3 Id = SE3::Identity();

      // root
      if (!usingFF)
      {
        typename JC::JointModelComposite jff((typename JC::JointModelTranslation()));
        jff.addJoint(typename JC::JointModelSphericalZYX());
        addJointAndBody(model, jff, "universe", "root", Id);
      }
      else
      {
        addJointAndBody(model, typename JC::JointModelFreeFlyer(), "universe", "root", Id);
        model.lowerPositionLimit.template segment<4>(3).fill(-1.);
        model.upperPositionLimit.template segment<4>(3).fill(1.);
      }

      // lleg
      addJointAndBody(model, typename JC::JointModelRX(), "root_joint", "lleg1");
      addJointAndBody(model, typename JC::JointModelRY(), "lleg1_joint", "lleg2");
      addJointAndBody(model, typename JC::JointModelRZ(), "lleg2_joint", "lleg3");
      addJointAndBody(model, typename JC::JointModelRY(), "lleg3_joint", "lleg4");
      addJointAndBody(model, typename JC::JointModelRY(), "lleg4_joint", "lleg5");
      addJointAndBody(model, typename JC::JointModelRX(), "lleg5_joint", "lleg6");

      // rleg
      addJointAndBody(model, typename JC::JointModelRX(), "root_joint", "rleg1");
      addJointAndBody(model, typename JC::JointModelRY(), "rleg1_joint", "rleg2");
      addJointAndBody(model, typename JC::JointModelRZ(), "rleg2_joint", "rleg3");
      addJointAndBody(model, typename JC::JointModelRY(), "rleg3_joint", "rleg4");
      addJointAndBody(model, typename JC::JointModelRY(), "rleg4_joint", "rleg5");
      addJointAndBody(model, typename JC::JointModelRX(), "rleg5_joint", "rleg6");

      // trunc
      addJointAndBody(model, typename JC::JointModelRY(), "root_joint", "torso1");
      addJointAndBody(model, typename JC::JointModelRZ(), "torso1_joint", "chest");

      // rarm
      addJointAndBody(model, typename JC::JointModelRX(), "chest_joint", "rarm1");
      addJointAndBody(model, typename JC::JointModelRY(), "rarm1_joint", "rarm2");
      addJointAndBody(model, typename JC::JointModelRZ(), "rarm2_joint", "rarm3");
      addJointAndBody(model, typename JC::JointModelRY(), "rarm3_joint", "rarm4");
      addJointAndBody(model, typename JC::JointModelRY(), "rarm4_joint", "rarm5");
      addJointAndBody(model, typename JC::JointModelRX(), "rarm5_joint", "rarm6");

      // larm
      addJointAndBody(model, typename JC::JointModelRX(), "chest_joint", "larm1");
      addJointAndBody(model, typename JC::JointModelRY(), "larm1_joint", "larm2");
      addJointAndBody(model, typename JC::JointModelRZ(), "larm2_joint", "larm3");
      addJointAndBody(model, typename JC::JointModelRY(), "larm3_joint", "larm4");
      Index joint_id = addJointAndBody(model, typename JC::JointModelRY(), "larm4_joint", "larm5");

      if (mimic)
      {
        Scalar multiplier = 2.5;
        Scalar offset = 0.75;
        addJointAndBody(
          model,
          typename JC::JointModelMimic(
            typename JC::JointModelRX(), model.joints[joint_id].derived(), multiplier, offset),
          "larm5_joint", "larm6");
      }
      else
      {
        addJointAndBody(model, typename JC::JointModelRX(), "larm5_joint", "larm6");
      }
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void humanoid(ModelTpl<Scalar, Options, JointCollectionTpl> & model, bool usingFF)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef JointCollectionTpl<Scalar, Options> JC;
      typedef typename Model::SE3 SE3;
      typedef typename Model::Inertia Inertia;

      typedef typename JC::JointModelRX::ConfigVector_t CV;
      typedef typename JC::JointModelRX::TangentVector_t TV;

      typename Model::JointIndex idx, chest, ffidx;

      static const Scalar pi = PI<Scalar>();

      SE3 Marm(SE3::Matrix3::Identity(), SE3::Vector3::UnitZ());
      SE3 I4 = SE3::Identity();
      Inertia Ijoint(.1, Inertia::Vector3::Zero(), Inertia::Matrix3::Identity() * .01);
      Inertia Iarm(1., typename Inertia::Vector3(0, 0, .5), Inertia::Matrix3::Identity());
      CV qmin = CV::Constant(-3.14), qmax = CV::Constant(3.14);
      TV vmax = TV::Constant(10), taumax = TV::Constant(10);

      /* --- Free flyer --- */
      if (usingFF)
      {
        ffidx =
          model.addJoint(0, typename JC::JointModelFreeFlyer(), SE3::Identity(), "root_joint");
        model.lowerPositionLimit.template segment<4>(3).fill(-1.);
        model.upperPositionLimit.template segment<4>(3).fill(1.);
      }
      else
      {
        typename JC::JointModelComposite jff((typename JC::JointModelTranslation()));
        jff.addJoint(typename JC::JointModelSphericalZYX());
        ffidx = model.addJoint(0, jff, SE3::Identity(), "root_joint");
      }
      model.appendBodyToJoint(ffidx, Ijoint);
      model.addJointFrame(ffidx);

      /* --- Lower limbs --- */

      details::addManipulator(
        model, false, ffidx,
        SE3(details::rotate(pi, SE3::Vector3::UnitX()), typename SE3::Vector3(0, -0.2, -.1)),
        "rleg_");
      details::addManipulator(
        model, false, ffidx,
        SE3(details::rotate(pi, SE3::Vector3::UnitX()), typename SE3::Vector3(0, 0.2, -.1)),
        "lleg_");

      model.jointPlacements[7].rotation() =
        details::rotate(pi / 2, SE3::Vector3::UnitY()); // rotate right foot
      model.jointPlacements[13].rotation() =
        details::rotate(pi / 2, SE3::Vector3::UnitY()); // rotate left  foot

      /* --- Chest --- */
      idx = model.addJoint(
        ffidx, typename JC::JointModelRX(), I4, "chest1_joint", taumax, vmax, qmin, qmax);
      model.appendBodyToJoint(idx, Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("chest1_body", idx);

      idx = model.addJoint(
        idx, typename JC::JointModelRY(), I4, "chest2_joint", taumax, vmax, qmin, qmax);
      model.appendBodyToJoint(idx, Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("chest2_body", idx);

      chest = idx;

      /* --- Head --- */
      idx = model.addJoint(
        idx, typename JC::JointModelRX(), SE3(SE3::Matrix3::Identity(), SE3::Vector3::UnitZ()),
        "head1_joint", taumax, vmax, qmin, qmax);
      model.appendBodyToJoint(idx, Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("head1_body", idx);

      idx = model.addJoint(
        idx, typename JC::JointModelRY(), I4, "head2_joint", taumax, vmax, qmin, qmax);
      model.appendBodyToJoint(idx, Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("head2_body", idx);

      /* --- Upper Limbs --- */
      details::addManipulator(
        model, false, chest,
        SE3(details::rotate(pi, SE3::Vector3::UnitX()), typename SE3::Vector3(0, -0.3, 1.)),
        "rarm_");
      details::addManipulator(
        model, false, chest,
        SE3(details::rotate(pi, SE3::Vector3::UnitX()), typename SE3::Vector3(0, 0.3, 1.)),
        "larm_");
    }

#ifdef PINOCCHIO_WITH_COLLISION
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void humanoidGeometries(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model, GeometryModel & geom)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::FrameIndex FrameIndex;
      typedef typename GeometryObject::SE3 SE3;

      details::addManipulatorGeometries(model, geom, "rleg_");
      details::addManipulatorGeometries(model, geom, "lleg_");
      details::addManipulatorGeometries(model, geom, "rarm_");
      details::addManipulatorGeometries(model, geom, "larm_");

      FrameIndex parentFrame;

      const Eigen::Vector4d meshColor(1., 1., 0.78, 1.0);

      parentFrame = model.getBodyId("chest1_body");
      GeometryObject chestBall(
        "chest_object", model.frames[parentFrame].parentJoint, parentFrame, SE3::Identity(),
        std::shared_ptr<coal::Sphere>(new coal::Sphere(0.05)), "SPHERE", Eigen::Vector3d::Ones(),
        false, meshColor);
      geom.addGeometryObject(chestBall);

      parentFrame = model.getBodyId("head2_body");
      GeometryObject headBall(
        "head_object", model.frames[parentFrame].parentJoint, parentFrame,
        SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0, 0, 0.5)),
        std::shared_ptr<coal::Sphere>(new coal::Sphere(0.25)), "SPHERE", Eigen::Vector3d::Ones(),
        false, meshColor);
      geom.addGeometryObject(headBall);

      parentFrame = model.getBodyId("chest2_body");
      GeometryObject chestArm(
        "chest2_object", model.frames[parentFrame].parentJoint, parentFrame,
        SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0, 0, 0.5)),
        std::shared_ptr<coal::Capsule>(new coal::Capsule(0.05, .8)), "SPHERE",
        Eigen::Vector3d::Ones(), false, meshColor);
      geom.addGeometryObject(chestArm);
    }
#endif

  } // namespace buildModels

} // namespace pinocchio

#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

  #ifndef PINOCCHIO_SKIP_MULTIBODY_SAMPLE_MODELS

namespace pinocchio
{
  namespace buildModels
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    manipulator<context::Scalar, context::Options, JointCollectionDefaultTpl>(Model &, bool);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    humanoid<context::Scalar, context::Options, JointCollectionDefaultTpl>(Model &, bool);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    humanoidRandom<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      Model &, bool, bool);
  } // namespace buildModels
} // namespace pinocchio

  #endif // PINOCCHIO_SKIP_MULTIBODY_SAMPLE_MODELS

#endif // ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
