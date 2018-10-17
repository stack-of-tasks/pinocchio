//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/parsers/sample-models.hpp"

namespace se3
{
  namespace buildModels
  {
    const SE3 Id = SE3::Identity();

    
    template<typename JointModel>
    static void addJointAndBody(Model & model,
                                const JointModelBase<JointModel> & joint,
                                const std::string & parent_name,
                                const std::string & name,
                                const SE3 placement = SE3::Random(),
                                bool setRandomLimits = true)
    {
      typedef typename JointModel::ConfigVector_t CV;
      typedef typename JointModel::TangentVector_t TV;
      
      Model::JointIndex idx;
      
      if(setRandomLimits)
        idx = model.addJoint(model.getJointId(parent_name),joint,
                             placement, name + "_joint",
                             TV::Random() + TV::Constant(1), // effort
                             TV::Random() + TV::Constant(1), // vel
                             CV::Random() - CV::Constant(1), // qmin
                             CV::Random() + CV::Constant(1)  // qmax
                             );
      else
        idx = model.addJoint(model.getJointId(parent_name),joint,
                             placement, name + "_joint");
        
      model.addJointFrame(idx);
      
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
      model.addBodyFrame(name + "_body", idx);
    }

    void humanoid2d(Model & model)
    {
      // root
      addJointAndBody(model, JointModelRX(), "universe", "ff1", Id, false);
      addJointAndBody(model, JointModelRY(), "ff1_joint", "root", Id, false);

      // lleg
      addJointAndBody(model, JointModelRZ(), "root_joint", "lleg1", Id, false);
      addJointAndBody(model, JointModelRY(), "lleg1_joint", "lleg2", Id, false);

      // rlgg
      addJointAndBody(model, JointModelRZ(), "root_joint", "rleg1", Id, false);
      addJointAndBody(model, JointModelRY(), "lleg1_joint", "rleg2", Id, false);

      // torso
      addJointAndBody(model, JointModelRY(), "root_joint", "torso1", Id, false);
      addJointAndBody(model, JointModelRZ(), "torso1_joint", "chest", Id, false);

      // rarm
      addJointAndBody(model, JointModelRX(), "chest_joint", "rarm1", Id, false);
      addJointAndBody(model, JointModelRZ(), "rarm1_joint", "rarm2", Id, false);

      // larm
      addJointAndBody(model, JointModelRX(), "root_joint", "larm1", Id, false);
      addJointAndBody(model, JointModelRZ(), "larm1_joint", "larm2", Id, false);

    }

    void humanoidSimple(Model & model, bool usingFF)
    {
      // root
      if(! usingFF )
      {
        addJointAndBody(model, JointModelPX(), "universe", "ff1", Id);
        addJointAndBody(model, JointModelPY(), "ff1_joint", "ff2", Id);
        addJointAndBody(model, JointModelPZ(), "ff2_joint", "ff3", Id);
        addJointAndBody(model, JointModelRZ(), "ff3_joint", "ff4", Id);
        addJointAndBody(model, JointModelRY(), "ff4_joint", "ff5", Id);
        addJointAndBody(model, JointModelRX(), "ff5_joint", "root", Id);
      }
      else
        addJointAndBody(model, JointModelFreeFlyer(), "universe", "root", Id);

      // lleg
      addJointAndBody(model,JointModelRX(),"root_joint","lleg1");
      addJointAndBody(model,JointModelRY(),"lleg1_joint","lleg2");
      addJointAndBody(model,JointModelRZ(),"lleg2_joint","lleg3");
      addJointAndBody(model,JointModelRY(),"lleg3_joint","lleg4");
      addJointAndBody(model,JointModelRY(),"lleg4_joint","lleg5");
      addJointAndBody(model,JointModelRX(),"lleg5_joint","lleg6");
      
      // rleg
      addJointAndBody(model,JointModelRX(),"root_joint","rleg1");
      addJointAndBody(model,JointModelRY(),"rleg1_joint","rleg2");
      addJointAndBody(model,JointModelRZ(),"rleg2_joint","rleg3");
      addJointAndBody(model,JointModelRY(),"rleg3_joint","rleg4");
      addJointAndBody(model,JointModelRY(),"rleg4_joint","rleg5");
      addJointAndBody(model,JointModelRX(),"rleg5_joint","rleg6");

      // trunc
      addJointAndBody(model,JointModelRY(),"root_joint","torso1");
      addJointAndBody(model,JointModelRZ(),"torso1_joint","chest");
      
      // rarm
      addJointAndBody(model,JointModelRX(),"chest_joint","rarm1");
      addJointAndBody(model,JointModelRY(),"rarm1_joint","rarm2");
      addJointAndBody(model,JointModelRZ(),"rarm2_joint","rarm3");
      addJointAndBody(model,JointModelRY(),"rarm3_joint","rarm4");
      addJointAndBody(model,JointModelRY(),"rarm4_joint","rarm5");
      addJointAndBody(model,JointModelRX(),"rarm5_joint","rarm6");
      
      // larm
      addJointAndBody(model,JointModelRX(),"chest_joint","larm1");
      addJointAndBody(model,JointModelRY(),"larm1_joint","larm2");
      addJointAndBody(model,JointModelRZ(),"larm2_joint","larm3");
      addJointAndBody(model,JointModelRY(),"larm3_joint","larm4");
      addJointAndBody(model,JointModelRY(),"larm4_joint","larm5");
      addJointAndBody(model,JointModelRX(),"larm5_joint","larm6");

    }

    void manipulator(Model & model)
    {
      typedef typename JointModelRX::ConfigVector_t CV;
      typedef typename JointModelRX::TangentVector_t TV;

      Model::JointIndex idx = 0;

      SE3 Marm(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,1));
      SE3 I4 = SE3::Identity();
      Inertia Ijoint = Inertia(.1,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity()*.01);
      Inertia Iarm   = Inertia(1.,Eigen::Vector3d(0,0,.5),Eigen::Matrix3d::Identity()*.1 );
      CV qmin = CV::Constant(-3.14), qmax   = CV::Constant(3.14);
      TV vmax = TV::Constant(-10),   taumax = TV::Constant(10);

      idx = model.addJoint(idx,JointModelRX(),I4  ,"shoulder1_joint", vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("shoulder1_body",idx);

      idx = model.addJoint(idx,JointModelRY(),I4  ,"shoulder2_joint", vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("shoulder2_body",idx);

      idx = model.addJoint(idx,JointModelRZ(),I4  ,"shoulder3_joint", vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("upperarm_body",idx);

      idx = model.addJoint(idx,JointModelRX(),Marm,"elbow_joint",     vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("lowerarm_body",idx);

      idx = model.addJoint(idx,JointModelRX(),Marm,"wrist1_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("wrist1_body",idx);

      idx = model.addJoint(idx,JointModelRZ(),I4  ,"wrist2_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("effector_body",idx);

    }

    void manipulatorGeometries(const Model& model, GeometryModel & geom)
    {
      GeometryObject upperArm("upperarm_object",
                              model.getBodyId("upperarm_body"),0,
                              boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(0.1, 1)),
                              SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(.5,0,0)) );
      geom.addGeometryObject(upperArm,model,true);

    }


    
  } // namespace buildModels
  
} // namespace se3
