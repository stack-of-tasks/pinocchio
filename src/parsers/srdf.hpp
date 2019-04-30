//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_parser_srdf_hpp__
#define __pinocchio_parser_srdf_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{
  namespace srdf
  {
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geomModel Model of the geometries.
    /// \param[out] data_geom Data containing the active collision pairs.
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairs(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              GeometryModel & geomModel,
                              const std::string & filename,
                              const bool verbose = false);
    
    /// \copydoc removeCollisionPairs
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    PINOCCHIO_DEPRECATED
    void removeCollisionPairsFromSrdf(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      GeometryModel & geomModel,
                                      const std::string & filename,
                                      const bool verbose = false)
    {
      removeCollisionPairs(model,geomModel,filename,verbose);
    }
    
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geomModel Model of the geometries.
    /// \param[out] data_geom Data containing the active collision pairs.
    /// \param[in] xmlString constaining the XML SRDF string.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairsFromXML(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     GeometryModel & geomModel,
                                     const std::string & xmlString,
                                     const bool verbose = false);
    
    /// \copydoc removeCollisionPairsFromXML
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    PINOCCHIO_DEPRECATED
    void removeCollisionPairsFromSrdfString(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                            GeometryModel & geomModel,
                                            const std::string & xmlString,
                                            const bool verbose = false)
    {
      removeCollisionPairsFromXML(model,geomModel,xmlString,verbose);
    }
    
#endif // ifdef PINOCCHIO_WITH_HPP_FCL

    ///
    /// \brief Get the neutral configuration of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model The Model for which we want the neutral config
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode.
    ///
    /// \return The neutral configuration as an eigen vector
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    PINOCCHIO_DEPRECATED
    typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType
    getNeutralConfiguration(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            const std::string & filename,
                            const bool verbose = false);
    
    /// \copydoc pinocchio::srdf::getNeutralConfiguration
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    PINOCCHIO_DEPRECATED
    typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType
    getNeutralConfigurationFromSrdf(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                    const std::string & filename,
                                    const bool verbose = false)
    { return getNeutralConfiguration(model,filename,verbose); }


    ///
    /// \brief Get the reference configurations of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect. The reference configurations are
    ///        saved in a map indexed by the configuration name (model.referenceConfigurations).
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
    
    /// \copydoc pinocchio::srdf::loadRotorParameters
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    PINOCCHIO_DEPRECATED
    bool loadRotorParamsFromSrdf(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 const std::string & filename,
                                 const bool verbose = false)
    {
      return loadRotorParameters(model,filename,verbose);
    }
    
  }
} // namespace pinocchio

#include "pinocchio/parsers/srdf.hxx"

#endif // ifndef __pinocchio_parser_srdf_hpp__
