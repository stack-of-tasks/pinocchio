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

#ifndef __se3_model_hpp__
#define __se3_model_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/joint/joint.hpp"
#include "pinocchio/deprecated.hh"
#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>
#include <Eigen/Cholesky>

namespace se3
{
  struct Model
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef se3::Index Index;
    typedef se3::JointIndex JointIndex;
    typedef se3::GeomIndex GeomIndex;
    typedef se3::FrameIndex FrameIndex;
    typedef std::vector<Index> IndexVector;

    /// \brief Dimension of the configuration vector representation.
    int nq;
    
    /// \brief Dimension of the velocity vector space.
    int nv;
    
    /// \brief Number of joints.
    int njoints;

    /// \brief Number of bodies.
    int nbodies;
    
    /// \brief Number of operational frames.
    int nframes;

    /// \brief Spatial inertias of the body *i* expressed in the supporting joint frame *i*.
    container::aligned_vector<Inertia> inertias;
    
    /// \brief Placement (SE3) of the input of joint *i* regarding to the parent joint output *li*.
    container::aligned_vector<SE3> jointPlacements;

    /// \brief Model of joint *i*, encapsulated in a JointModelAccessor.
    JointModelVector joints;
    
    /// \brief Joint parent of joint *i*, denoted *li* (li==parents[i]).
    std::vector<JointIndex> parents;

    /// \brief Name of joint *i*
    std::vector<std::string> names;
    
    /// \brief Vector of joint's neutral configurations
    Eigen::VectorXd neutralConfiguration;

    /// \brief Vector of rotor inertia parameters
    Eigen::VectorXd rotorInertia;
    
    /// \brief Vector of rotor gear ratio parameters
    Eigen::VectorXd rotorGearRatio;
    
    /// \brief Vector of maximal joint torques
    Eigen::VectorXd effortLimit;
    /// \brief Vector of maximal joint velocities
    Eigen::VectorXd velocityLimit;

    /// \brief Lower joint configuration limit
    Eigen::VectorXd lowerPositionLimit;
    /// \brief Upper joint configuration limit
    Eigen::VectorXd upperPositionLimit;

    /// \brief Vector of operational frames registered on the model.
    container::aligned_vector<Frame> frames;
    
    /// \brief Vector of subtrees.
    /// subtree[j] corresponds to the subtree supported by the joint *j*.
    /// The first element of subtree[j] is the index of the joint *j* itself.
    std::vector<IndexVector> subtrees;

    /// \brief Spatial gravity of the model.
    Motion gravity;
    
    /// \brief Default 3D gravity vector (=(0,0,-9.81)).
    static const Eigen::Vector3d gravity981;

    /// \brief Model name;
    std::string name;

    /// \brief Default constructor. Builds an empty model with no joints.
    Model()
      : nq(0)
      , nv(0)
      , njoints(1)
      , nbodies(1)
      , nframes(0)
      , inertias(1)
      , jointPlacements(1, SE3::Identity())
      , joints(1)
      , parents(1, 0)
      , names(1)
      , subtrees(1)
      , gravity( gravity981,Eigen::Vector3d::Zero() )
    {
      names[0]     = "universe";     // Should be "universe joint (trivial)"
      // FIXME Should the universe joint be a FIXED_JOINT even if it is
      // in the list of joints ? See comment in definition of
      // Model::addJointFrame and Model::addBodyFrame
      addFrame(Frame("universe", 0, 0, SE3::Identity(), FIXED_JOINT));
      // Inertia of universe has no sense.
      inertias[0].mass() = std::numeric_limits<double>::quiet_NaN();
      inertias[0].lever().fill (std::numeric_limits<double>::quiet_NaN());
      inertias[0].inertia().fill (std::numeric_limits<double>::quiet_NaN());
    }
    ~Model() {} // std::cout << "Destroy model" << std::endl; }
    
    ///
    /// \brief Add a joint to the kinematic tree with given bounds.
    ///
    /// \remark This method does not add a Frame of same name to the vector of frames.
    ///         Use Model::addJointFrame.
    /// \remark The inertia supported by the joint is set to Zero.
    ///
    /// \tparam JointModelDerived The type of the joint model.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] joint_model The joint model.
    /// \param[in] joint_placement Placement of the joint inside its parent joint.
    /// \param[in] joint_name Name of the joint. If empty, the name is random.
    /// \param[in] max_effort Maximal joint torque.
    /// \param[in] max_velocity Maximal joint velocity.
    /// \param[in] min_config Lower joint configuration.
    /// \param[in] max_config Upper joint configuration.
    ///
    /// \return The index of the new joint.
    ///
    /// \sa Model::appendBodyToJoint, Model::addJointFrame
    ///
    template<typename JointModelDerived>
    JointIndex addJoint(const JointIndex parent, const JointModelBase<JointModelDerived> & joint_model, const SE3 & joint_placement,
                        const std::string & joint_name,
                        const Eigen::VectorXd & max_effort,
                        const Eigen::VectorXd & max_velocity,
                        const Eigen::VectorXd & min_config,
                        const Eigen::VectorXd & max_config
                        );

    ///
    /// \brief Add a joint to the kinematic tree with infinite bounds.
    ///
    /// \remark This method also adds a Frame of same name to the vector of frames.
    /// \remark The inertia supported by the joint is set to Zero.
    ///
    /// \tparam JointModelDerived The type of the joint model.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] joint_model The joint model.
    /// \param[in] joint_placement Placement of the joint inside its parent joint.
    /// \param[in] joint_name Name of the joint. If empty, the name is random.
    ///
    /// \return The index of the new joint.
    ///
    /// \sa Model::appendBodyToJoint
    ///
    template<typename JointModelDerived>
    JointIndex addJoint(const JointIndex parent, const JointModelBase<JointModelDerived> & joint_model, const SE3 & joint_placement,
                        const std::string & joint_name
                        );

    ///
    /// \brief Add a joint to the frame tree.
    ///
    /// \param[in] jointIndex Index of the joint.
    /// \param[in] frameIndex Index of the parent frame. If negative,
    ///            the parent frame is the frame of the parent joint.
    ///
    /// \return The index of the new frame or -1 in case of error.
    ///
    int addJointFrame (const JointIndex& jointIndex,
                             int         frameIndex = -1);

    ///
    /// \brief Append a body to a given joint of the kinematic tree.
    ///
    /// \param[in] joint_index Index of the supporting joint.
    /// \param[in] Y Spatial inertia of the body.
    /// \param[in] body_placement The relative placement of the body regarding to the parent joint. Set default to the Identity placement.
    ///
    /// \sa Model::addJoint
    ///
    void appendBodyToJoint(const JointIndex joint_index, const Inertia & Y,
                           const SE3 & body_placement = SE3::Identity());

    ///
    /// \brief Add a body to the frame tree.
    ///
    /// \param[in] body_name Name of the body.
    /// \param[in] parentJoint Index of the parent joint.
    /// \param[in] body_placement The relative placement of the body regarding to the parent joint. Set default to the Identity placement.
    /// \param[in] previousFrame Index of the parent frame. If negative,
    ///            the parent frame is the frame of the parent joint.
    ///
    /// \return The index of the new frame or -1 in case of error.
    ///
    int addBodyFrame (const std::string & body_name,
                      const JointIndex  & parentJoint,
                      const SE3         & body_placement = SE3::Identity(),
                            int           previousFrame  = -1);

    ///
    /// \brief Return the index of a body given by its name.
    /// 
    /// \warning If no body is found, return the number of elements at time T.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent body)
    /// 
    /// \param[in] name Name of the body.
    ///
    /// \return Index of the body.
    ///
    JointIndex getBodyId(const std::string & name) const;
    
    ///
    /// \brief Check if a body given by its name exists.
    ///
    /// \param[in] name Name of the body.
    ///
    /// \return True if the body exists in the kinematic tree.
    ///
    bool existBodyName(const std::string & name) const;
    
    
    ///
    /// \brief Return the index of a joint given by its name.
    ///
    /// \warning If no joint is found, return the number of elements at time T.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent joint)
    /// \param[in] name Name of the joint.
    ///
    /// \return name Name of the joint.
    ///
    JointIndex getJointId(const std::string & name) const;
    
    ///
    /// \brief Check if a joint given by its name exists.
    ///
    /// \param[in] name Name of the joint.
    ///
    /// \return True if the joint exists in the kinematic tree.
    ///
    bool existJointName(const std::string & name) const;
    
    ///
    /// \brief Get the name of a joint given by its index.
    ///
    /// \param[in] index Index of the joint.
    ///
    /// \return Name of the joint.
    ///
    PINOCCHIO_DEPRECATED const std::string & getJointName(const JointIndex index) const;

    ///
    /// \brief Returns the index of a frame given by its name.
    ///        \sa Model::existFrame to check if the frame exists or not.
    ///
    /// \warning If no frame is found, returns the size of the vector of Model::frames.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent frame).
    /// 
    /// \param[in] name Name of the frame.
    /// \param[in] type Type of the frame.
    ///
    /// \return Index of the frame.
    ///
    FrameIndex getFrameId (const std::string & name, const FrameType& type = (FrameType) (JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR )) const;
    
    ///
    /// \brief Checks if a frame given by its name exists.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] type Type of the frame.
    ///
    /// \return Returns true if the frame exists.
    ///
    bool existFrame (const std::string & name, const FrameType& type = (FrameType) (JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR )) const;
    
    
    ///
    /// \brief Adds a frame to the kinematic tree.
    ///
    /// \param[in] frame The frame to add to the kinematic tree.
    ///
    /// \return Returns the index of the frame if it has been successfully added,
    ///         -1 otherwise.
    ///
    int addFrame(const Frame & frame);

    /// Check the validity of the attributes of Model with respect to the specification of some
    /// algorithms.
    ///
    /// The method is a template so that the checkers can be defined in each algorithms.
    /// \param[in] checker a class, typically defined in the algorithm module, that 
    /// validates the attributes of model.
    /// \return true if the Model is valid, false otherwise.
    template<typename D>
    inline bool check(const AlgorithmCheckerBase<D> & checker = AlgorithmCheckerBase<D>()) const
    { return checker.checkModel(*this); }

    /// Run check(fusion::list) with DEFAULT_CHECKERS as argument.
    inline bool check() const;
    
    /// Run checkData on data and current model.
    ///
    /// \param[in] data to be checked wrt *this.
    ///
    /// \return true if the data is valid, false otherwise.
    inline bool check(const Data & data) const;

  protected:
    
    /// \brief Add the joint_id to its parent subtrees.
    ///
    /// \param[in] joint_id The id of the joint to add to the subtrees
    ///
    void addJointIndexToParentSubtrees(const JointIndex joint_id);
  };

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/model.hxx"

#endif // ifndef __se3_model_hpp__
