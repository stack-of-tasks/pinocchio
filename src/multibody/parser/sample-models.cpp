//
// Copyright (c) 2015 CNRS
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

#include "pinocchio/multibody/parser/sample-models.hpp"

namespace se3
{
  namespace buildModels
  {

    void humanoid2d(Model& model)
    {
      model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                    "ff1_joint", "ff1_body");
      model.addBody(model.getBodyId("ff1_body"),JointModelRY(),SE3::Identity(),Inertia::Random(),
                    "root_joint", "root_body");

      model.addBody(model.getBodyId("root_body"),JointModelRZ(),SE3::Random(),Inertia::Random(),
                    "lleg1_joint", "lleg1_body");
      model.addBody(model.getBodyId("lleg1_body"),JointModelRY(),SE3::Random(),Inertia::Random(),
                    "lleg2_joint", "lleg2_body");

      model.addBody(model.getBodyId("root_body"),JointModelRZ(),SE3::Random(),Inertia::Random(),
                    "rleg1_joint", "rleg1_body");
      model.addBody(model.getBodyId("rleg1_body"),JointModelRY(),SE3::Random(),Inertia::Random(),
                    "rleg2_joint", "rleg2_body");

      model.addBody(model.getBodyId("root_body"),JointModelRY(),SE3::Random(),Inertia::Random(),
                    "torso1_joint", "torso1_body");
      model.addBody(model.getBodyId("torso1_body"),JointModelRZ(),SE3::Random(),Inertia::Random(),
                    "chest_joint", "chest_body");

      model.addBody(model.getBodyId("chest_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm1_joint", "rarm1_body");
      model.addBody(model.getBodyId("rarm1_body"),JointModelRZ(),SE3::Random(),Inertia::Random(),
                    "rarm2_joint", "rarm2_body");

      model.addBody(model.getBodyId("chest_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm1_joint", "larm1_body");
      model.addBody(model.getBodyId("larm1_body"),JointModelRZ(),SE3::Random(),Inertia::Random(),
                    "larm2_joint", "larm2_body");
    }

    void humanoidSimple(Model& model, bool usingFF)
    {
      if(! usingFF )
      {
        model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                      "ff1_joint", "ff1_body");
        model.addBody(model.getBodyId("ff1_body"),JointModelRY(),SE3::Identity(),Inertia::Random(),
                      "ff2_joint", "ff2_body");
        model.addBody(model.getBodyId("ff2_body"),JointModelRZ(),SE3::Identity(),Inertia::Random(),
                      "ff3_joint", "ff3_body");
        model.addBody(model.getBodyId("ff3_body"),JointModelRZ(),SE3::Random(),Inertia::Random(),
                      "ff4_joint", "ff4_body");
        model.addBody(model.getBodyId("ff4_body"),JointModelRY(),SE3::Identity(),Inertia::Random(),
                      "ff5_joint", "ff5_body");
        model.addBody(model.getBodyId("ff5_body"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                      "root_joint", "root_body");
      }
      else
      {
        model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),
                      Inertia::Random(),"root_joint", "root_body");
      }

      model.addBody(model.getBodyId("root_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "lleg1_joint", "lleg1_body");
      model.addBody(model.getBodyId("lleg1_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "lleg2_joint", "lleg2_body");
      model.addBody(model.getBodyId("lleg2_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "lleg3_joint", "lleg3_body");
      model.addBody(model.getBodyId("lleg3_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "lleg4_joint", "lleg4_body");
      model.addBody(model.getBodyId("lleg4_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "lleg5_joint", "lleg5_body");
      model.addBody(model.getBodyId("lleg5_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "lleg6_joint", "lleg6_body");

      model.addBody(model.getBodyId("root_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rleg1_joint", "rleg1_body");
      model.addBody(model.getBodyId("rleg1_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rleg2_joint", "rleg2_body");
      model.addBody(model.getBodyId("rleg2_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rleg3_joint", "rleg3_body");
      model.addBody(model.getBodyId("rleg3_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rleg4_joint", "rleg4_body");
      model.addBody(model.getBodyId("rleg4_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rleg5_joint", "rleg5_body");
      model.addBody(model.getBodyId("rleg5_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rleg6_joint", "rleg6_body");

      model.addBody(model.getBodyId("root_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "torso1_joint", "torso1_body");
      model.addBody(model.getBodyId("torso1_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "chest_joint", "chest_body");

      model.addBody(model.getBodyId("chest_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm1_joint", "rarm1_body");
      model.addBody(model.getBodyId("rarm1_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm2_joint", "rarm2_body");
      model.addBody(model.getBodyId("rarm2_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm3_joint", "rarm3_body");
      model.addBody(model.getBodyId("rarm3_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm4_joint", "rarm4_body");
      model.addBody(model.getBodyId("rarm4_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm5_joint", "rarm5_body");
      model.addBody(model.getBodyId("rarm5_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "rarm6_joint", "rarm6_body");

      model.addBody(model.getBodyId("chest_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm1_joint", "larm1_body");
      model.addBody(model.getBodyId("larm1_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm2_joint", "larm2_body");
      model.addBody(model.getBodyId("larm2_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm3_joint", "larm3_body");
      model.addBody(model.getBodyId("larm3_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm4_joint", "larm4_body");
      model.addBody(model.getBodyId("larm4_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm5_joint", "larm5_body");
      model.addBody(model.getBodyId("larm5_body"),JointModelRX(),SE3::Random(),Inertia::Random(),
                    "larm6_joint", "larm6_body");
    }

  } // namespace buildModels
} // namespace se3
