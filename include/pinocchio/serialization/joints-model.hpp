//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_model_hpp__
#define __pinocchio_serialization_joints_model_hpp__

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct Serialize<JointModelCompositeTpl<Scalar, Options, JointCollectionTpl>>
  {
    template<typename Archive>
    static void
    run(Archive & ar, JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> & joint)
    {
      using boost::serialization::make_nvp;

      ar & make_nvp("m_nq", joint.m_nq);
      ar & make_nvp("m_nv", joint.m_nv);
      ar & make_nvp("m_nvExtended", joint.m_nvExtended);
      ar & make_nvp("m_idx_q", joint.m_idx_q);
      ar & make_nvp("m_nqs", joint.m_nqs);
      ar & make_nvp("m_idx_v", joint.m_idx_v);
      ar & make_nvp("m_nvs", joint.m_nvs);
      ar & make_nvp("m_idx_vExtended", joint.m_idx_vExtended);
      ar & make_nvp("m_nvExtendeds", joint.m_nvExtendeds);
      ar & make_nvp("njoints", joint.njoints);

      ar & make_nvp("joints", joint.joints);
      ar & make_nvp("jointPlacements", joint.jointPlacements);
    }
  };
} // namespace pinocchio

namespace boost
{
  namespace serialization
  {

    // For some older version of gcc, we have to rely on an additional namespace
    // to avoid ambiguous call to boost::serialization::serialize
    namespace fix
    {
      template<class Archive, typename Derived>
      void serialize(
        Archive & ar, pinocchio::JointModelBase<Derived> & joint, const unsigned int version)
      {
        split_free(ar, joint, version);
      }
    } // namespace fix

    template<class Archive, typename Derived>
    void save(
      Archive & ar,
      const pinocchio::JointModelBase<Derived> & joint,
      const unsigned int /*version*/)
    {
      const pinocchio::JointIndex i_id = joint.id();
      const int i_q = joint.idx_q(), i_v = joint.idx_v(), i_vExtended = joint.idx_vExtended();

      ar & make_nvp("i_id", i_id);
      ar & make_nvp("i_q", i_q);
      ar & make_nvp("i_v", i_v);
      ar & make_nvp("i_vExtended", i_vExtended);
    }

    template<typename Derived>
    class SetJointIndexes
    {
      Derived & joint;

    public:
      explicit SetJointIndexes(Derived & joint)
      : joint(joint) {};

      void run(pinocchio::JointIndex i_id, int i_q, int i_v, int i_vExtended)
      {
        joint.setIndexes(i_id, i_q, i_v, i_vExtended);
      }
    };

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    class SetJointIndexes<pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
    {
      pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl> & joint_m;

    public:
      explicit SetJointIndexes(
        pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl> & joint)
      : joint_m(joint) {};

      void run(pinocchio::JointIndex i_id, int i_q, int i_v, int i_vExtended)
      {
        joint_m.setMimicIndexes(joint_m.jmodel().id(), i_q, i_v, joint_m.jmodel().idx_vExtended());
        joint_m.setIndexes(i_id, 0, 0, i_vExtended);
      }
    };

    template<class Archive, typename Derived>
    void
    load(Archive & ar, pinocchio::JointModelBase<Derived> & joint, const unsigned int /*version*/)
    {
      pinocchio::JointIndex i_id;
      int i_q, i_v, i_vExtended;

      ar & make_nvp("i_id", i_id);
      ar & make_nvp("i_q", i_q);
      ar & make_nvp("i_v", i_v);
      ar & make_nvp("i_vExtended", i_vExtended);

      SetJointIndexes<Derived>(joint.derived()).run(i_id, i_q, i_v, i_vExtended);
    }

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & ar,
      pinocchio::JointModelRevoluteTpl<Scalar, Options, axis> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, axis> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & ar,
      pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, axis> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, axis> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & ar,
      pinocchio::JointModelPrismaticTpl<Scalar, Options, axis> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelPrismaticTpl<Scalar, Options, axis> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & ar,
      pinocchio::JointModelHelicalTpl<Scalar, Options, axis> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelHelicalTpl<Scalar, Options, axis> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
      ar & make_nvp("m_pitch", joint.m_pitch);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
      ar & make_nvp("axis", joint.axis);
      ar & make_nvp("m_pitch", joint.m_pitch);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelFreeFlyerTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelFreeFlyerTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelPlanarTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelPlanarTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelSphericalTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelSphericalTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelSphericalZYXTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelSphericalZYXTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelTranslationTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelTranslationTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
      ar & make_nvp("axis", joint.axis);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
      ar & make_nvp("axis", joint.axis);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
      ar & make_nvp("axis", joint.axis);
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointModelUniversalTpl<Scalar, Options> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelUniversalTpl<Scalar, Options> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
      ar & make_nvp("axis1", joint.axis1);
      ar & make_nvp("axis2", joint.axis2);
    }

    template<
      class Archive,
      typename Scalar,
      int Options,
      template<typename S, int O> class JointCollectionTpl>
    void serialize(
      Archive & ar,
      pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);

      ::pinocchio::Serialize<JointType>::run(ar, joint);
    }

    template<
      class Archive,
      typename Scalar,
      int Options,
      template<typename S, int O> class JointCollectionTpl>
    void serialize(
      Archive & ar,
      pinocchio::JointModelTpl<Scalar, Options, JointCollectionTpl> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelTpl<Scalar, Options, JointCollectionTpl> JointType;
      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);

      typedef typename JointCollectionTpl<Scalar, Options>::JointModelVariant JointModelVariant;
      ar & make_nvp("base_variant", base_object<JointModelVariant>(joint));
    }

    template<
      class Archive,
      typename Scalar,
      int Options,
      template<typename S, int O> class JointCollectionTpl>
    void serialize(
      Archive & ar,
      pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl> & joint,
      const unsigned int version)
    {
      typedef pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointType;
      ar & make_nvp("jmodel", joint.jmodel());
      ar & make_nvp("scaling", joint.scaling());
      ar & make_nvp("offset", joint.offset());

      fix::serialize(ar, *static_cast<pinocchio::JointModelBase<JointType> *>(&joint), version);
    }

  } // namespace serialization
} // namespace boost

#endif // ifndef __pinocchio_serialization_joints_model_hpp__
