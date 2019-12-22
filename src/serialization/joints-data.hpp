//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_data_hpp__
#define __pinocchio_serialization_joints_data_hpp__

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct Serialize< JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> >
  {
    template<typename Archive>
    static void run(Archive & ar,
                    JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> & joint_data)
    {
      using boost::serialization::make_nvp;
      
      ar & make_nvp("joints",joint_data.joints);
      ar & make_nvp("iMlast",joint_data.iMlast);
      ar & make_nvp("pjMi",joint_data.pjMi);

      ar & make_nvp("S",joint_data.S);
      ar & make_nvp("M",joint_data.M);
      ar & make_nvp("v",joint_data.v);
      ar & make_nvp("c",joint_data.c);

      ar & make_nvp("U",joint_data.U);
      ar & make_nvp("Dinv",joint_data.Dinv);
      ar & make_nvp("UDinv",joint_data.UDinv);
      ar & make_nvp("StU",joint_data.StU);
    }
  };
}

namespace boost
{
  namespace serialization
  {
    
    // For some older version of gcc, we have to rely on an additional namespace
    // to avoid ambiguous call to boost::serialization::serialize
    namespace fix
    {
      template <class Archive, typename Derived>
      void serialize(Archive & ar,
                     pinocchio::JointDataBase<Derived> & joint_data,
                     const unsigned int /*version*/)
      {
        ar & make_nvp("S",joint_data.S());
        ar & make_nvp("M",joint_data.M());
        ar & make_nvp("v",joint_data.v());
        ar & make_nvp("c",joint_data.c());
        
        ar & make_nvp("U",joint_data.U());
        ar & make_nvp("Dinv",joint_data.Dinv());
        ar & make_nvp("UDinv",joint_data.UDinv());
      }
    }
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::JointDataRevoluteTpl<Scalar,Options,axis> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataRevoluteTpl<Scalar,Options,axis> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,axis> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,axis> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::JointDataPrismaticTpl<Scalar,Options,axis> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataPrismaticTpl<Scalar,Options,axis> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataFreeFlyerTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataFreeFlyerTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataPlanarTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataPlanarTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataSphericalTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataSphericalTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataSphericalZYXTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataSphericalZYXTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataTranslationTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataTranslationTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataRevoluteUnalignedTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataRevoluteUnalignedTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointDataPrismaticUnalignedTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataPrismaticUnalignedTpl<Scalar,Options> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
    }
    
    template <class Archive, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    void serialize(Archive & ar,
                   pinocchio::JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
      
      ::pinocchio::Serialize<JointType>::run(ar,joint);
    }
    
    template <class Archive, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    void serialize(Archive & ar,
                   pinocchio::JointDataTpl<Scalar,Options,JointCollectionTpl> & joint,
                   const unsigned int /*version*/)
    {
      typedef typename JointCollectionTpl<Scalar,Options>::JointDataVariant JointDataVariant;
      ar & make_nvp("base_variant",base_object<JointDataVariant>(joint));
    }
    
    template <class Archive, typename JointData>
    void serialize(Archive & ar,
                   pinocchio::JointDataMimic<JointData> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointDataMimic<JointData> JointType;
      fix::serialize(ar,static_cast<pinocchio::JointDataBase<JointType>&>(joint),version);
      
      ar & make_nvp("jdata",joint.jdata());
      ar & make_nvp("scaling",joint.scaling());
      ar & make_nvp("jointConfiguration",joint.jointConfiguration());
      ar & make_nvp("jointVelocity",joint.jointVelocity());
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_joints_data_hpp__
