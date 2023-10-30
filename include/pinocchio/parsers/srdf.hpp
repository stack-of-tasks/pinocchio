//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_parser_srdf_hpp__
#define __pinocchio_parser_srdf_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{
  namespace srdf
  {
    
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geom_model Model of the geometries.
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairs(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              GeometryModel & geom_model,
                              const std::string & filename,
                              const bool verbose = false);
    
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geom_model Model of the geometries.
    /// \param[in] xmlString constaining the XML SRDF string.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairsFromXML(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     GeometryModel & geom_model,
                                     const std::string & xmlString,
                                     const bool verbose = false);
    
    ///
    /// \brief Get the reference configurations of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect. The reference configurations are
    ///        saved in a map indexed by the configuration name (model.referenceConfigurations).
    ///
    /// \param[in] model The Model for which we want the reference configs.
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void
    loadReferenceConfigurations(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                const std::string & filename,
                                const bool verbose = false);

    ///
    /// \brief Get the reference configurations of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect. The reference configurations are
    ///        saved in a map indexed by the configuration name (model.referenceConfigurations).
    ///
    /// \param[in] model The Model for which we want the reference configs.
    /// \param[in] xmlStream a stream containing the content of a SRDF.
    /// \param[in] verbose Verbosity mode.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void
    loadReferenceConfigurationsFromXML(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       std::istream & xmlStream,
                                       const bool verbose = false);
      
    ///
    /// \brief Load the rotor params of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model The Model for which we want the rotor parmeters
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode.
    ///
    /// \return Boolean whether it loads or not.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bool loadRotorParameters(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const std::string & filename,
                             const bool verbose = false);
    
  }
} // namespace pinocchio

#include "pinocchio/parsers/srdf.hxx"

#endif // ifndef __pinocchio_parser_srdf_hpp__
