#ifndef __se3_joint_variant_hpp__
#define __se3_joint_variant_hpp__

#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"

namespace se3
{
  typedef boost::variant< JointModelRX,JointModelRY,JointModelRZ , JointModelFreeFlyer> JointModelVariant;
  typedef boost::variant< JointDataRX, JointDataRY, JointDataRZ  , JointDataFreeFlyer > JointDataVariant;

  typedef std::vector<JointModelVariant> JointModelVector;
  typedef std::vector<JointDataVariant> JointDataVector;

  class CreateJointData: public boost::static_visitor<JointDataVariant>
  {
  public:
    template<typename D>
    JointDataVariant operator()(const JointModelBase<D> & jmodel) const
    { return JointDataVariant(jmodel.createData()); }
    
    static JointDataVariant run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( CreateJointData(), jmodel ); }
  };

  class Joint_nv: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & ) const
    { return D::NV; }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_nv(), jmodel ); }
  };
  inline int nv(const JointModelVariant & jmodel) { return Joint_nv::run(jmodel); }

  class Joint_nq: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & ) const
    { return D::NQ; }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_nq(), jmodel ); }
  };
  inline int nq(const JointModelVariant & jmodel) { return Joint_nq::run(jmodel); }

  class Joint_idx_q: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.idx_q(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_idx_q(), jmodel ); }
  };
  inline int idx_q(const JointModelVariant & jmodel) { return Joint_idx_q::run(jmodel); }

  class Joint_idx_v: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.idx_v(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_idx_v(), jmodel ); }
  };
  inline int idx_v(const JointModelVariant & jmodel) { return Joint_idx_v::run(jmodel); }


} // namespace se3

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointModelVariant)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointDataVariant)

#endif // ifndef __se3_joint_variant_hpp__
