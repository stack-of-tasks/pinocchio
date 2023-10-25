//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_sample_models_hpp__
#define __pinocchio_sample_models_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{
  namespace buildModels
  {  
    /** \brief Create a 6-DOF kinematic chain shoulder-elbow-wrist.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     */
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void manipulator(ModelTpl<Scalar,Options,JointCollectionTpl> & model);
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    /** \brief Create the geometries on top of the kinematic model created by manipulator function.
     *
     * \param model, const, kinematic chain typically produced by the function manipulator(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after manipulator(model).
     */
    template<typename Scalar, int Options,
    template<typename,int> class JointCollectionTpl>
    void manipulatorGeometries(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               GeometryModel & geom);
#endif

    /** \brief Create a 28-DOF kinematic chain of a floating humanoid robot.
     * 
     * The kinematic chain has four 6-DOF limbs shoulder-elbow-wrist, one 2-DOF torso, one
     * 2-DOF neck. The float joint is either a JointModelFreeFloating (with nq=7 and nv=6),
     * or a composite joint with a JointModelTranslation
     * and a roll-pitch-yaw joint JointModelSphericalZYX (with total nq=nv=6).
     * Using a free-floating or a composite joint is decided by the boolean usingFF.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint. This changes the size of the configuration space (35 vs 34).
     */
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void humanoid(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  bool usingFF=true);
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    /** \brief Create the geometries on top of the kinematic model created by humanoid function.
     *
     * \param model, const, kinematic chain typically produced by the function humanoid(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after humanoid(model).
     */
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void humanoidGeometries(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            GeometryModel & geom);
#endif
    
    /** \brief Create a humanoid kinematic tree with 6-DOF limbs and random joint placements.
     *
     * This method is only meant to be used in unittest. Due to random placement and masses,
     * the resulting model is likely to not correspond to any physically-plausible model. 
     * You may want to consider pinocchio::humanoid and pinocchio::humanoidGeometries functions that
     * rather define a plain and non-random model. 
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint translation + roll-pitch-yaw.
     * This changes the size of the configuration space (33 vs 32).
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void humanoidRandom(ModelTpl<Scalar,Options,JointCollectionTpl> & model, bool usingFF = true);
   
  } // namespace buildModels
} // namespace pinocchio

#include "pinocchio/parsers/sample-models.hxx"

#endif // ifndef __pinocchio_sample_models_hpp__
