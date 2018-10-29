//
// Copyright (c) 2015-2018 CNRS
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
                                const SE3 & placement = SE3::Random(),
                                bool setRandomLimits = true)
    {
      typedef typename JointModel::ConfigVector_t CV;
      typedef typename JointModel::TangentVector_t TV;
      
      Model::JointIndex idx;
      
      if(setRandomLimits)
        idx = model.addJoint(model.getJointId(parent_name),joint,
                             placement, name + "_joint",
                             TV::Random(joint.nv(),1) + TV::Constant(joint.nv(),1,1), // effort
                             TV::Random(joint.nv(),1) + TV::Constant(joint.nv(),1,1), // vel
                             CV::Random(joint.nq(),1) - CV::Constant(joint.nq(),1,1), // qmin
                             CV::Random(joint.nq(),1) + CV::Constant(joint.nq(),1,1)  // qmax
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

    void humanoidRandom(Model & model, bool usingFF)
    {
      // root
      if(! usingFF )
      {
        JointModelComposite jff((JointModelTranslation()));
        jff.addJoint(JointModelSphericalZYX());
        addJointAndBody(model, jff, "universe", "root", Id);
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

    static void addManipulator(Model & model,
                               Model::JointIndex rootJoint = 0,
                               const SE3 & Mroot = SE3::Identity(),
                               const std::string & pre = "")
    {
      typedef JointModelRX::ConfigVector_t CV;
      typedef JointModelRX::TangentVector_t TV;

      Model::JointIndex idx = rootJoint;

      SE3 Marm(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,1));
      SE3 I4 = SE3::Identity();
      Inertia Ijoint = Inertia(.1,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity()*.01);
      Inertia Iarm   = Inertia(1.,Eigen::Vector3d(0,0,.5),Eigen::Matrix3d::Identity()*.1 );
      CV qmin = CV::Constant(-3.14), qmax   = CV::Constant(3.14);
      TV vmax = TV::Constant(-10),   taumax = TV::Constant(10);

      idx = model.addJoint(idx,JointModelRX(),Mroot,pre+"shoulder1_joint", vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame(pre+"shoulder1_body",idx);

      idx = model.addJoint(idx,JointModelRY(),I4   ,pre+"shoulder2_joint", vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame(pre+"shoulder2_body",idx);

      idx = model.addJoint(idx,JointModelRZ(),I4   ,pre+"shoulder3_joint", vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame(pre+"upperarm_body",idx);

      idx = model.addJoint(idx,JointModelRY(),Marm ,pre+"elbow_joint",     vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame(pre+"lowerarm_body",idx);
      model.addBodyFrame(pre+"elbow_body",idx);

      idx = model.addJoint(idx,JointModelRX(),Marm ,pre+"wrist1_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame(pre+"wrist1_body",idx);

      idx = model.addJoint(idx,JointModelRY(),I4   ,pre+"wrist2_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame(pre+"effector_body",idx);

    }

#ifdef WITH_HPP_FCL
    /* Add a 6DOF manipulator shoulder-elbow-wrist geometries to an existing model. 
     * <model> is the the kinematic chain, constant.
     * <geom> is the geometry model where the new geoms are added.
     * <pre> is the prefix (string) before every name in the model.
     */
    static void addManipulatorGeometries(const Model & model,
                                         GeometryModel & geom,
                                         const std::string & pre = "")
    {
      GeometryObject shoulderBall(pre+"shoulder_object",
                                  model.getBodyId(pre+"shoulder1_body"),/*NR*/0,
                                  boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                                  SE3::Identity());
      geom.addGeometryObject(shoulderBall,model,true);
      
      GeometryObject elbowBall(pre+"elbow_object",
                               model.getBodyId(pre+"elbow_body"),/*NR*/0,
                               boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                               SE3::Identity());
      geom.addGeometryObject(elbowBall,model,true);
      
      GeometryObject wristBall(pre+"wrist_object",
                               model.getBodyId(pre+"wrist1_body"),/*NR*/0,
                               boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                               SE3::Identity());
      geom.addGeometryObject(wristBall,model,true);

      GeometryObject upperArm(pre+"upperarm_object",
                              model.getBodyId(pre+"upperarm_body"),/*NR*/0,
                              boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .8)),
                              SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0.5)) );
      geom.addGeometryObject(upperArm,model,true);

      GeometryObject lowerArm(pre+"lowerarm_object",
                              model.getBodyId(pre+"lowerarm_body"),/*NR*/0,
                              boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .8)),
                              SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0.5)) );
      geom.addGeometryObject(lowerArm,model,true);

      GeometryObject effectorArm(pre+"effector_object",
                                 model.getBodyId(pre+"effector_body"),/*NR*/0,
                                 boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .2)),
                                 SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0.1)) );
      geom.addGeometryObject(effectorArm,model,true);
    }
#endif

    void manipulator(Model & model) { addManipulator(model); }
    
#ifdef WITH_HPP_FCL
    void manipulatorGeometries(const Model & model, GeometryModel & geom)
    { addManipulatorGeometries(model,geom); }
#endif

    static Eigen::Matrix3d rotate(const double angle, const Eigen::Vector3d & axis)
    { return Eigen::AngleAxisd(angle,axis).toRotationMatrix(); }
    
    void humanoid(Model & model, bool usingFF)
    {
      using namespace Eigen;
      typedef JointModelRX::ConfigVector_t CV;
      typedef JointModelRX::TangentVector_t TV;
     
      Model::JointIndex idx,chest,ffidx;

      SE3 Marm(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,1));
      SE3 I4 = SE3::Identity();
      Inertia Ijoint = Inertia(.1,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity()*.01);
      Inertia Iarm   = Inertia(1.,Eigen::Vector3d(0,0,.5),Eigen::Matrix3d::Identity()*.1 );
      CV qmin = CV::Constant(-3.14), qmax   = CV::Constant(3.14);
      TV vmax = TV::Constant(-10),   taumax = TV::Constant(10);

      /* --- Free flyer --- */
      if(usingFF)
          ffidx = model.addJoint(0,JointModelFreeFlyer(),SE3::Identity(),"freeflyer_joint");
      else
        {
          JointModelComposite jff((JointModelTranslation()));
          jff.addJoint(JointModelSphericalZYX());
          ffidx = model.addJoint(0,jff,SE3::Identity(),"freeflyer_joint");
        }
      model.addJointFrame(ffidx);

      /* --- Lower limbs --- */

      AngleAxisd(M_PI,Vector3d(1,0,0)).toRotationMatrix();

      addManipulator(model,ffidx,SE3(rotate(M_PI,Vector3d::UnitX()),Vector3d(0,-0.2,-.1)),"rleg");
      addManipulator(model,ffidx,SE3(rotate(M_PI,Vector3d::UnitX()),Vector3d(0, 0.2,-.1)),"lleg");

      model.jointPlacements[7 ].rotation() = rotate(M_PI/2,Vector3d::UnitY()); // rotate right foot
      model.jointPlacements[13].rotation() = rotate(M_PI/2,Vector3d::UnitY()); // rotate left  foot
      
      /* --- Chest --- */
      idx = model.addJoint(ffidx,JointModelRX(),I4 ,"chest1_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("chest1_body",idx);

      idx = model.addJoint(idx,JointModelRY(),I4 ,"chest2_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("chest2_body",idx);

      chest = idx;

      /* --- Head --- */
      idx = model.addJoint(idx,JointModelRX(),
                           SE3(Matrix3d::Identity(),Vector3d(0.,0.,1.)),
                           "head1_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("head1_body",idx);

      idx = model.addJoint(idx,JointModelRY(),I4 ,"head2_joint",    vmax,taumax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("head2_body",idx);

      /* --- Upper Limbs --- */
      addManipulator(model,chest,SE3(rotate(M_PI,Vector3d::UnitX()),Vector3d(0,-0.3, 1.)),"rarm");
      addManipulator(model,chest,SE3(rotate(M_PI,Vector3d::UnitX()),Vector3d(0, 0.3, 1.)),"larm");
    }

#ifdef WITH_HPP_FCL
    void humanoidGeometries(const Model & model, GeometryModel & geom)
    {
      addManipulatorGeometries(model,geom,"rleg");
      addManipulatorGeometries(model,geom,"lleg");
      addManipulatorGeometries(model,geom,"rarm");
      addManipulatorGeometries(model,geom,"larm");

      GeometryObject chestBall("chest_object",
                               model.getBodyId("chest1_body"),/*NR*/0,
                               boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                               SE3::Identity());
      geom.addGeometryObject(chestBall,model,true);

      GeometryObject headBall("head_object",
                               model.getBodyId("head2_body"),/*NR*/0,
                               boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(0.25)),
                               SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0.5)) );
      geom.addGeometryObject(headBall,model,true);

      GeometryObject chestArm("chest2_object",
                              model.getBodyId("chest2_body"),/*NR*/0,
                              boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .8)),
                              SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0.5)) );
      geom.addGeometryObject(chestArm,model,true);
    }
#endif

  } // namespace buildModels
  
} // namespace se3
