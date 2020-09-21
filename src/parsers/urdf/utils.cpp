#include "pinocchio/parsers/urdf/utils.hpp"

namespace pinocchio
{
  namespace urdf
  {
    namespace details
    {
    
      SE3 convertFromUrdf(const ::urdf::Pose & M)
      {
        const ::urdf::Vector3 & p = M.position;
        const ::urdf::Rotation & q = M.rotation;
        return SE3(SE3::Quaternion(q.w,q.x,q.y,q.z).matrix(),
                   SE3::Vector3(p.x,p.y,p.z));
      }
    }
  }
}
