#ifndef __se3_joint_variant_hpp__
#define __se3_joint_variant_hpp__

#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"

namespace se3
{
  typedef boost::variant< JointModelRX,JointModelRY,JointModelRZ,JointModelFreeFlyer> JointModelVariant;
  typedef boost::variant< JointDataRX, JointDataRY, JointDataRZ, JointDataFreeFlyer > JointDataVariant;

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

} // namespace se3

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointModelVariant);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointDataVariant);

#endif // ifndef __se3_joint_variant_hpp__
