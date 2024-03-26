//
// Copyright (c) 2024 INRIA CNRS
//

#ifndef __pinocchio_parsers_mjcf_hpp__
#define __pinocchio_parsers_mjcf_hpp__

#include "pinocchio/multibody/model.hpp"

namespace pinocchio
{
  namespace mjcf
  {  

    ///
    /// \brief Build the model from an XML stream with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] xmlStream xml stream containing MJCF model
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose = false);
    
    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a fixed joint as root of the model tree
    ///
    /// \param[in] xmlStream The xml stream containing the MJCF Model.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl <Scalar, Options, JointCollectionTpl> &
    buildModel(const std::string &filename, 
                ModelTpl<Scalar,Options,JointCollectionTpl> & model, 
                const bool verbose = false);

  } // namespace mjcf
} // namespace pinocchio

#include "pinocchio/parsers/mjcf/model.hxx"

#endif // ifndef __pinocchio_parsers_mjcf_hpp__
