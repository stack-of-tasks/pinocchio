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
    
    template<typename JointModel>
    static void addJointAndBody(Model & model,
                                const JointModelBase<JointModel> & joint,
                                const std::string & parent_name,
                                const std::string & name)
    {
      typedef typename JointModel::ConfigVector_t CV;
      typedef typename JointModel::TangentVector_t TV;
      
      Model::JointIndex idx;
      
      idx = model.addJoint(model.getJointId(parent_name),joint,
                           SE3::Random(),
                           TV::Random() + TV::Constant(1),
                           TV::Random() + TV::Constant(1),
                           CV::Random() - CV::Constant(1),
                           CV::Random() + CV::Constant(1),
			   name + "_joint");
      
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),name + "_body");
    }

    void humanoid2d(Model & model)
    {
      Model::JointIndex idx;
      
      // root
      idx = model.addJoint(model.getJointId("universe"),JointModelRX(),SE3::Identity(),"ff1_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"ff1_body");
      
      idx = model.addJoint(model.getJointId("ff1_joint"),JointModelRY(),SE3::Identity(),"root_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"root_body");

      // lleg
      idx = model.addJoint(model.getJointId("root_joint"),JointModelRZ(),SE3::Identity(),"lleg1_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"lleg1_body");
      
      idx = model.addJoint(model.getJointId("lleg1_joint"),JointModelRY(),SE3::Identity(),"lleg2_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"lleg2_body");

      // rlgg
      idx = model.addJoint(model.getJointId("root_joint"),JointModelRZ(),SE3::Identity(),"rleg1_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"rleg1_body");

      idx = model.addJoint(model.getJointId("rleg1_joint"),JointModelRY(),SE3::Identity(),"rleg2_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"rleg2_body");

      // torso
      idx = model.addJoint(model.getJointId("root_joint"),JointModelRY(),SE3::Identity(),"torso1_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"torso1_body");

      idx = model.addJoint(model.getJointId("torso1_joint"),JointModelRZ(),SE3::Identity(),"chest_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"chest_body");

      // rarm
      idx = model.addJoint(model.getJointId("chest_joint"),JointModelRX(),SE3::Identity(),"rarm1_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"rarm1_body");

      idx = model.addJoint(model.getJointId("rarm1_joint"),JointModelRZ(),SE3::Identity(),"rarm2_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"rarm2_body");

      // larm
      idx = model.addJoint(model.getJointId("chest_joint"),JointModelRX(),SE3::Identity(),"larm1_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"larm1_body");

      idx = model.addJoint(model.getJointId("larm1_joint"),JointModelRZ(),SE3::Identity(),"larm2_joint");
      model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"larm2_body");

    }

    void humanoidSimple(Model & model, bool usingFF)
    {
      Model::JointIndex idx;
      
      // root
      if(! usingFF )
      {
        idx = model.addJoint(model.getJointId("universe"),JointModelRX(),SE3::Identity(),"ff1_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"ff1_body");
        
        idx = model.addJoint(model.getJointId("ff1_joint"),JointModelRY(),SE3::Identity(),"ff2_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"ff2_body");
        
        idx = model.addJoint(model.getJointId("ff2_joint"),JointModelRZ(),SE3::Identity(),"ff3_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"ff3_body");
        
        idx = model.addJoint(model.getJointId("ff3_joint"),JointModelRZ(),SE3::Random(),"ff4_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"ff4_body");
        
        idx = model.addJoint(model.getJointId("ff4_joint"),JointModelRY(),SE3::Identity(),"ff5_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"ff5_body");
        
        idx = model.addJoint(model.getJointId("ff5_joint"),JointModelRX(),SE3::Identity(),"root_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"root_body");
      }
      else
      {
        typedef JointModelFreeFlyer::ConfigVector_t CV;
        typedef JointModelFreeFlyer::TangentVector_t TV;
        
        idx = model.addJoint(model.getJointId("universe"),JointModelFreeFlyer(),
                             SE3::Identity(),
                             TV::Zero(), 1e3 * (TV::Random() + TV::Constant(1.)),
                             1e3 * (CV::Random() - CV::Constant(1)),
                             1e3 * (CV::Random() + CV::Constant(1)),
			     "root_joint");
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"root_body");
      }

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

  } // namespace buildModels
  
} // namespace se3
