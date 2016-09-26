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
      
      if (setRandomLimits)
        idx = model.addJoint(model.getJointId(parent_name),joint,
                           SE3::Random(),
                             name + "_joint",
                           TV::Random() + TV::Constant(1),
                           TV::Random() + TV::Constant(1),
                           CV::Random() - CV::Constant(1),
                           CV::Random() + CV::Constant(1)
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
        addJointAndBody(model, JointModelRX(), "universe", "ff1", Id, false);
        addJointAndBody(model, JointModelRY(), "ff1_joint", "ff2", Id, false);
        addJointAndBody(model, JointModelRZ(), "ff2_joint", "ff3", Id, false);
        addJointAndBody(model, JointModelRZ(), "ff3_joint", "ff4", Id, false);
        addJointAndBody(model, JointModelRY(), "ff4_joint", "ff5", Id, false);
        addJointAndBody(model, JointModelRX(), "ff5_joint", "root", Id, false);
      }
      else
      {
        // typedef JointModelFreeFlyer::ConfigVector_t CV;
        // typedef JointModelFreeFlyer::TangentVector_t TV;
        
        addJointAndBody(model, JointModelFreeFlyer(), "universe", "root", Id, false);
        // idx = model.addJoint(model.getJointId("universe"),JointModelFreeFlyer(),
                             // SE3::Identity(),"root_joint",
                             // TV::Zero(), 1e3 * (TV::Random() + TV::Constant(1.)),
                             // 1e3 * (CV::Random() - CV::Constant(1)),
                             // 1e3 * (CV::Random() + CV::Constant(1)));
        // model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"root_body");
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
