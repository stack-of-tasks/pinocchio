//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_model_hpp__
#define __pinocchio_multibody_model_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"

#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/serialization/serializable.hpp"

#include <iostream>
#include <map>
#include <iterator>

namespace pinocchio
{
  
  template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
  struct ModelTpl
  : serialization::Serializable< ModelTpl<_Scalar,_Options,JointCollectionTpl> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef InertiaTpl<Scalar,Options> Inertia;
    typedef FrameTpl<Scalar,Options> Frame;
    
    typedef pinocchio::Index Index;
    typedef pinocchio::JointIndex JointIndex;
    typedef pinocchio::GeomIndex GeomIndex;
    typedef pinocchio::FrameIndex FrameIndex;
    typedef std::vector<Index> IndexVector;
    
    typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModel;
    typedef JointDataTpl<Scalar,Options,JointCollectionTpl> JointData;
    
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(JointModel) JointModelVector;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(JointData) JointDataVector;
    
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(Frame) FrameVector;
    
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    
    /// \brief Dense vectorized version of a joint configuration vector.
    typedef VectorXs ConfigVectorType;

    /// \brief Map between a string (key) and a configuration vector
    typedef std::map<std::string, ConfigVectorType>  ConfigVectorMap;
    
    /// \brief Dense vectorized version of a joint tangent vector (e.g. velocity, acceleration, etc).
    ///        It also handles the notion of co-tangent vector (e.g. torque, etc).
    typedef VectorXs TangentVectorType;

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
    PINOCCHIO_ALIGNED_STD_VECTOR(Inertia) inertias;
    
    /// \brief Placement (SE3) of the input of joint *i* regarding to the parent joint output *li*.
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) jointPlacements;

    /// \brief Model of joint *i*, encapsulated in a JointModelAccessor.
    JointModelVector joints;
    
    /// \brief Starting index of the joint *i* in the configuration space
    std::vector<int> idx_qs;
    
    /// \brief Dimension of the joint *i* configuration subspace
    std::vector<int> nqs;
    
    /// \brief Starting index of the joint *i* in the tangent configuration space
    std::vector<int> idx_vs;
    
    /// \brief Dimension of the joint *i* tangent subspace
    std::vector<int> nvs;
    
    /// \brief Joint parent of joint *i*, denoted *li* (li==parents[i]).
    std::vector<JointIndex> parents;

    /// \brief Name of joint *i*
    std::vector<std::string> names;

    /// \brief Map of reference configurations, indexed by user given names.
    ConfigVectorMap referenceConfigurations;

    /// \brief Vector of rotor inertia parameters
    TangentVectorType rotorInertia;
    
    /// \brief Vector of rotor gear ratio parameters
    TangentVectorType rotorGearRatio;
    
    /// \brief Vector of joint friction parameters
    TangentVectorType friction;
    
    /// \brief Vector of joint damping parameters
    TangentVectorType damping;

    /// \brief Vector of maximal joint torques
    TangentVectorType effortLimit;
    
    /// \brief Vector of maximal joint velocities
    TangentVectorType velocityLimit;

    /// \brief Lower joint configuration limit
    ConfigVectorType lowerPositionLimit;
    
    /// \brief Upper joint configuration limit
    ConfigVectorType upperPositionLimit;

    /// \brief Vector of operational frames registered on the model.
    FrameVector frames;
    
    /// \brief Vector of joint supports.
    /// supports[j] corresponds to the collection of all joints located on the path between body *j*  and the root.
    /// The last element of supports[j] is the index of the joint *j* itself.
    std::vector<IndexVector> supports;
    
    /// \brief Vector of joint subtrees.
    /// subtree[j] corresponds to the subtree supported by the joint *j*.
    /// The first element of subtree[j] is the index of the joint *j* itself.
    std::vector<IndexVector> subtrees;

    /// \brief Spatial gravity of the model.
    Motion gravity;
    
    /// \brief Default 3D gravity vector (=(0,0,-9.81)).
    static const Vector3 gravity981;

    /// \brief Model name;
    std::string name;

    /// \brief Default constructor. Builds an empty model with no joints.
    ModelTpl()
    : nq(0)
    , nv(0)
    , njoints(1)
    , nbodies(1)
    , nframes(0)
    , inertias(1, Inertia::Zero())
    , jointPlacements(1, SE3::Identity())
    , joints(1)
    , idx_qs(1,0)
    , nqs(1,0)
    , idx_vs(1,0)
    , nvs(1,0)
    , parents(1, 0)
    , names(1)
    , supports(1,IndexVector(1,0))
    , subtrees(1)
    , gravity(gravity981,Vector3::Zero())
    {
      names[0]     = "universe";     // Should be "universe joint (trivial)"
      // FIXME Should the universe joint be a FIXED_JOINT even if it is
      // in the list of joints ? See comment in definition of
      // Model::addJointFrame and Model::addBodyFrame
      addFrame(Frame("universe", 0, 0, SE3::Identity(), FIXED_JOINT));
    }
    ~ModelTpl() {} // std::cout << "Destroy model" << std::endl; }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    ModelTpl<NewScalar,Options,JointCollectionTpl> cast() const
    {
      typedef ModelTpl<NewScalar,Options,JointCollectionTpl> ReturnType;
      ReturnType res;
      res.nq = nq; res.nv = nv;
      res.njoints = njoints;
      res.nbodies = nbodies;
      res.nframes = nframes;
      res.parents = parents;
      res.names = names;
      res.subtrees = subtrees;
      res.gravity = gravity.template cast<NewScalar>();
      res.name = name;
      
      res.idx_qs = idx_qs;
      res.nqs = nqs;
      res.idx_vs = idx_vs;
      res.nvs = nvs;
      
      // Eigen Vectors
      res.rotorInertia = rotorInertia.template cast<NewScalar>();
      res.rotorGearRatio = rotorGearRatio.template cast<NewScalar>();
      res.friction = friction.template cast<NewScalar>();
      res.damping = damping.template cast<NewScalar>();
      res.effortLimit = effortLimit.template cast<NewScalar>();
      res.velocityLimit = velocityLimit.template cast<NewScalar>();
      res.lowerPositionLimit = lowerPositionLimit.template cast<NewScalar>();
      res.upperPositionLimit = upperPositionLimit.template cast<NewScalar>();

      typename ConfigVectorMap::const_iterator it;
      for (it = referenceConfigurations.begin();
           it != referenceConfigurations.end(); it++ )
      {
        res.referenceConfigurations.insert(std::make_pair(it->first, it->second.template cast<NewScalar>()));
      }
        
      // reserve vectors
      res.inertias.resize(inertias.size());
      res.jointPlacements.resize(jointPlacements.size());
      res.joints.resize(joints.size());
      res.frames.resize(frames.size());

      /// copy into vectors
      for(size_t k = 0; k < joints.size(); ++k)
      {
        res.inertias[k] = inertias[k].template cast<NewScalar>();
        res.jointPlacements[k] = jointPlacements[k].template cast<NewScalar>();
        res.joints[k] = joints[k].template cast<NewScalar>();
      }
      
      for(size_t k = 0; k < frames.size(); ++k)
      {
        res.frames[k] = frames[k].template cast<NewScalar>();
      }
      
      return res;
    }
    
    ///
    /// \brief Equality comparison operator.
    ///
    /// \returns true if *this is equal to other.
    ///
    bool operator==(const ModelTpl & other) const
    {
      bool res =
         other.nq == nq
      && other.nv == nv
      && other.njoints == njoints
      && other.nbodies == nbodies
      && other.nframes == nframes
      && other.parents == parents
      && other.names == names
      && other.subtrees == subtrees
      && other.gravity == gravity
      && other.name == name;
      
      res &=
         other.idx_qs == idx_qs
      && other.nqs == nqs
      && other.idx_vs == idx_vs
      && other.nvs == nvs;

      if(other.referenceConfigurations.size() != referenceConfigurations.size())
        return false;
      
      typename ConfigVectorMap::const_iterator it = referenceConfigurations.begin();
      typename ConfigVectorMap::const_iterator it_other = other.referenceConfigurations.begin();
      for(long k = 0; k < (long)referenceConfigurations.size(); ++k)
      {
        std::advance(it,k); std::advance(it_other,k);
        
        if(it->second.size() != it_other->second.size())
          return false;
        if(it->second != it_other->second)
          return false;
      }

      if(other.rotorInertia.size() != rotorInertia.size())
        return false;
      res &= other.rotorInertia == rotorInertia;
      if(!res) return res;

      if(other.friction.size() != friction.size())
        return false;
      res &= other.friction == friction;
      if(!res) return res;

      if(other.damping.size() != damping.size())
        return false;
      res &= other.damping == damping;
      if(!res) return res;

      if(other.rotorGearRatio.size() != rotorGearRatio.size())
        return false;
      res &= other.rotorGearRatio == rotorGearRatio;
      if(!res) return res;

      if(other.effortLimit.size() != effortLimit.size())
        return false;
      res &= other.effortLimit == effortLimit;
      if(!res) return res;

      if(other.velocityLimit.size() != velocityLimit.size())
        return false;
      res &= other.velocityLimit == velocityLimit;
      if(!res) return res;

      if(other.lowerPositionLimit.size() != lowerPositionLimit.size())
        return false;
      res &= other.lowerPositionLimit == lowerPositionLimit;
      if(!res) return res;

      if(other.upperPositionLimit.size() != upperPositionLimit.size())
        return false;
      res &= other.upperPositionLimit == upperPositionLimit;
      if(!res) return res;

      for(size_t k = 1; k < inertias.size(); ++k)
      {
        res &= other.inertias[k] == inertias[k];
        if(!res) return res;
      }

      for(size_t k = 1; k < other.jointPlacements.size(); ++k)
      {
        res &= other.jointPlacements[k] == jointPlacements[k];
        if(!res) return res;
      }

      res &=
         other.joints == joints
      && other.frames == frames;
      
      return res;
    }
    
    ///
    /// \returns true if *this is NOT equal to other.
    ///
    bool operator!=(const ModelTpl & other) const
    { return !(*this == other); }

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
    JointIndex addJoint(const JointIndex parent,
                        const JointModel & joint_model,
                        const SE3 & joint_placement,
                        const std::string & joint_name);
    
    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const std::string &)
    ///
    /// \param[in] max_effort Maximal joint torque.
    /// \param[in] max_velocity Maximal joint velocity.
    /// \param[in] min_config Lower joint configuration.
    /// \param[in] max_config Upper joint configuration.
    ///
    JointIndex addJoint(const JointIndex parent,
                        const JointModel & joint_model,
                        const SE3 & joint_placement,
                        const std::string & joint_name,
                        const VectorXs & max_effort,
                        const VectorXs & max_velocity,
                        const VectorXs & min_config,
                        const VectorXs & max_config);
    
    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const std::string &,const VectorXs &,const VectorXs &,const VectorXs &,const VectorXs &)
    ///
    /// \param[in] friction Joint friction parameters.
    /// \param[in] damping Joint damping parameters.
    ///
    JointIndex addJoint(const JointIndex parent,
                        const JointModel & joint_model,
                        const SE3 & joint_placement,
                        const std::string & joint_name,
                        const VectorXs & max_effort,
                        const VectorXs & max_velocity,
                        const VectorXs & min_config,
                        const VectorXs & max_config,
                        const VectorXs & friction,
                        const VectorXs & damping);

    ///
    /// \brief Add a joint to the frame tree.
    ///
    /// \param[in] jointIndex Index of the joint.
    /// \param[in] frameIndex Index of the parent frame. If negative,
    ///            the parent frame is the frame of the parent joint.
    ///
    /// \return The index of the new frame
    ///
    FrameIndex addJointFrame(const JointIndex & joint_index,
                             int previous_frame_index = -1);

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
    /// \return The index of the new frame
    ///
    FrameIndex addBodyFrame(const std::string & body_name,
                            const JointIndex  & parentJoint,
                            const SE3         & body_placement = SE3::Identity(),
                            int                 previousFrame  = -1);

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
    FrameIndex getBodyId(const std::string & name) const;
    
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
    /// \return Index of the joint.
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
    FrameIndex getFrameId(const std::string & name,
                          const FrameType & type = (FrameType) (JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR )) const;
    
    ///
    /// \brief Checks if a frame given by its name exists.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] type Type of the frame.
    ///
    /// \return Returns true if the frame exists.
    ///
    bool existFrame(const std::string & name,
                    const FrameType& type = (FrameType) (JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR )) const;
    
    ///
    /// \brief Adds a frame to the kinematic tree.
    ///
    /// \param[in] frame The frame to add to the kinematic tree.
    ///
    /// \return Returns the index of the frame if it has been successfully added or if it already exists in the kinematic tree.
    ///
    FrameIndex addFrame(const Frame & frame);

    ///
    /// \brief Check the validity of the attributes of Model with respect to the specification of some
    /// algorithms.
    ///
    /// The method is a template so that the checkers can be defined in each algorithms.
    /// \param[in] checker a class, typically defined in the algorithm module, that 
    /// validates the attributes of model.
    ///
    /// \return true if the Model is valid, false otherwise.
    ///
    template<typename D>
    inline bool check(const AlgorithmCheckerBase<D> & checker = AlgorithmCheckerBase<D>()) const
    { return checker.checkModel(*this); }

    /// Run check(fusion::list) with DEFAULT_CHECKERS as argument.
    inline bool check() const;
    
    ///
    /// \brief Run checkData on data and current model.
    ///
    /// \param[in] data to be checked wrt *this.
    ///
    /// \return true if the data is valid, false otherwise.
    ///
    inline bool check(const Data & data) const;

  protected:
    
    ///
    /// \brief Add the joint_id to its parent subtrees.
    ///
    /// \param[in] joint_id The id of the joint to add to the subtrees
    ///
    void addJointIndexToParentSubtrees(const JointIndex joint_id);
  };

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/model.hxx"

#endif // ifndef __pinocchio_multibody_model_hpp__
