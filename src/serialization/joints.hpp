//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_hpp__
#define __pinocchio_serialization_joints_hpp__

#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/joint/joint-collection.hpp"

#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/eigen.hpp"
#include "pinocchio/serialization/vector.hpp"
#include "pinocchio/serialization/aligned-vector.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/variant.hpp>

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct Serialize< JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> >
  {
    template<typename Archive>
    static void run(Archive & ar,
                    JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> & joint)
    {
      using boost::serialization::make_nvp;
      
      ar & make_nvp("m_nq",joint.m_nq);
      ar & make_nvp("m_nv",joint.m_nv);
      ar & make_nvp("m_idx_q",joint.m_idx_q);
      ar & make_nvp("m_nqs",joint.m_nqs);
      ar & make_nvp("m_idx_v",joint.m_idx_v);
      ar & make_nvp("m_nvs",joint.m_nvs);
      ar & make_nvp("njoints",joint.njoints);
      
      ar & make_nvp("joints",joint.joints);
      ar & make_nvp("jointPlacements",joint.jointPlacements);
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
                     pinocchio::JointModelBase<Derived> & joint,
                     const unsigned int /*version*/)
      {
        ar & make_nvp("i_id",joint.i_id);
        ar & make_nvp("i_q",joint.i_q);
        ar & make_nvp("i_v",joint.i_v);
      }
    }
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::JointModelRevoluteTpl<Scalar,Options,axis> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,axis> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,axis> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,axis> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::JointModelPrismaticTpl<Scalar,Options,axis> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelPrismaticTpl<Scalar,Options,axis> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelFreeFlyerTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelFreeFlyerTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelPlanarTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelPlanarTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelSphericalTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelSphericalTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelSphericalZYXTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelSphericalZYXTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelTranslationTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelTranslationTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
      ar & make_nvp("axis",joint.axis);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
      ar & make_nvp("axis",joint.axis);
    }
    
    template <class Archive, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    void serialize(Archive & ar,
                   pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
      
      ::pinocchio::Serialize<JointType>::run(ar,joint);
    }
    
    template <class Archive, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    void serialize(Archive & ar,
                   pinocchio::JointModelTpl<Scalar,Options,JointCollectionTpl> & joint,
                   const unsigned int version)
    {
      typedef pinocchio::JointModelTpl<Scalar,Options,JointCollectionTpl> JointType;
//      ar & make_nvp("base_class",base_object< pinocchio::JointModelBase<JointType> >(joint));
      fix::serialize(ar,*static_cast<pinocchio::JointModelBase<JointType> *>(&joint),version);
      
      typedef typename JointCollectionTpl<Scalar,Options>::JointModelVariant JointModelVariant;
      ar & make_nvp("base_variant",base_object<JointModelVariant>(joint));
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_joints_hpp__
