#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#include <boost/variant.hpp>


using namespace se3;
typedef boost::variant< JointModelRX,JointModelFreeFlyer> JointModel;
typedef boost::variant< JointDataRX,JointDataFreeFlyer> JointData;


class CreateJointData: public boost::static_visitor<JointData>
{
public:
  template<typename D>
  JointData operator()(const se3::JointModelBase<D> & jmodel) const
  { return JointData(jmodel.createData()); }

  static JointData run( const JointModel & jmodel)
  { return boost::apply_visitor( CreateJointData(), jmodel ); }
};


int main()
{
  using namespace Eigen;


  JointModel jmodel = JointModelRX(0,0);
  const JointData & jdata = CreateJointData::run(jmodel); //x boost::apply_visitor( CreateJointData(), jmodel );


  se3::Model model;
}
