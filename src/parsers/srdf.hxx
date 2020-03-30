//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_parser_srdf_hxx__
#define __pinocchio_parser_srdf_hxx__

#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <iostream>

// Read XML file with boost
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <sstream>
#include <boost/foreach.hpp>

namespace pinocchio
{
  namespace srdf
  {
    namespace details
    {
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      void removeCollisionPairs(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                GeometryModel & geom_model,
                                std::istream & stream,
                                const bool verbose = false)
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        
        // Read xml stream
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(stream, pt);

        // Iterate over collision pairs
        BOOST_FOREACH(const ptree::value_type & v, pt.get_child("robot"))
        {
          if (v.first == "disable_collisions")
          {
            const std::string link1 = v.second.get<std::string>("<xmlattr>.link1");
            const std::string link2 = v.second.get<std::string>("<xmlattr>.link2");

            // Check first if the two bodies exist in model
            if (!model.existBodyName(link1) || !model.existBodyName(link2))
            {
              if (verbose)
                std::cout << "It seems that " << link1 << " or " << link2 << " do not exist in model. Skip." << std::endl;
              continue;
            }

            const typename Model::FrameIndex frame_id1 = model.getBodyId(link1);
            const typename Model::FrameIndex frame_id2 = model.getBodyId(link2);

            // Malformed SRDF
            if (frame_id1 == frame_id2)
            {
              if (verbose)
                std::cout << "Cannot disable collision between " << link1 << " and " << link2 << std::endl;
              continue;
            }

            typedef GeometryModel::CollisionPairVector CollisionPairVector;
            bool didRemove = false;
            for(CollisionPairVector::iterator _colPair = geom_model.collisionPairs.begin();
                _colPair != geom_model.collisionPairs.end(); ) {
              const CollisionPair& colPair (*_colPair);
              bool remove =
              (
               (geom_model.geometryObjects[colPair.first ].parentFrame == frame_id1)
               && (geom_model.geometryObjects[colPair.second].parentFrame == frame_id2)
               ) || (
                     (geom_model.geometryObjects[colPair.second].parentFrame == frame_id1)
                     && (geom_model.geometryObjects[colPair.first ].parentFrame == frame_id2)
                     );
              
              if (remove) {
                _colPair = geom_model.collisionPairs.erase(_colPair);
                didRemove = true;
              } else {
                ++_colPair;
              }
            }
            if(didRemove && verbose)
              std::cout << "Remove collision pair (" << link1 << "," << link2 << ")" << std::endl;

          }
        } // BOOST_FOREACH
      }
    } // namespace details

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairs(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              GeometryModel & geom_model,
                              const std::string & filename,
                              const bool verbose)
    {
      // Check extension
      const std::string extension = filename.substr(filename.find_last_of('.')+1);
      if (extension != "srdf")
      {
        const std::string exception_message (filename + " does not have the right extension.");
        throw std::invalid_argument(exception_message);
      }
      
      // Open file
      std::ifstream srdf_stream(filename.c_str());
      if (! srdf_stream.is_open())
      {
        const std::string exception_message (filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }

      details::removeCollisionPairs(model, geom_model, srdf_stream, verbose);
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairsFromXML(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     GeometryModel & geom_model,
                                     const std::string & xmlString,
                                     const bool verbose)
    {
      std::istringstream srdf_stream(xmlString);
      details::removeCollisionPairs(model, geom_model, srdf_stream, verbose);
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bool loadRotorParameters(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const std::string & filename,
                             const bool verbose)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::JointModel JointModel;
      
      // Check extension
      const std::string extension = filename.substr(filename.find_last_of('.')+1);
      if (extension != "srdf")
      {
        const std::string exception_message (filename + " does not have the right extension.");
        throw std::invalid_argument(exception_message);
      }
      
      // Open file
      std::ifstream srdf_stream(filename.c_str());
      if (! srdf_stream.is_open())
      {
        const std::string exception_message (filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
      
      // Read xml stream
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(srdf_stream, pt);
      
      // Iterate over all tags directly children of robot
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child("robot"))
      {
        // if we encounter a tag rotor_params
        if (v.first == "rotor_params")
        {
          // Iterate over all the joint tags
          BOOST_FOREACH(const ptree::value_type & joint, v.second)
          {
            if (joint.first == "joint")
            {
              std::string joint_name = joint.second.get<std::string>("<xmlattr>.name");
              const Scalar rotor_mass = (Scalar)joint.second.get<double>("<xmlattr>.mass");
              const Scalar rotor_gr = (Scalar)joint.second.get<double>("<xmlattr>.gear_ratio");
              if (verbose)
              {
                std::cout << "(" << joint_name << " , " <<
                rotor_mass << " , " << rotor_gr << ")" << std::endl;
              }
              // Search in model the joint and its config id
              typename Model::JointIndex joint_id = model.getJointId(joint_name);

              if (joint_id != model.joints.size()) // != model.njoints
              {
                const JointModel & joint = model.joints[joint_id];
                PINOCCHIO_CHECK_INPUT_ARGUMENT(joint.nv()==1);
                model.rotorInertia(joint.idx_v()) = rotor_mass;
                model.rotorGearRatio(joint.idx_v()) = rotor_gr;  // joint with 1 dof
              }
              else
              {
                if (verbose) std::cout << "The Joint " << joint_name << " was not found in model" << std::endl;
              }
            }
          }
          return true; 
        }
      }
      PINOCCHIO_CHECK_INPUT_ARGUMENT(false, "no rotor params found in the SRDF file");  
      return false; // warning : uninitialized vector is returned
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    struct LoadReferenceConfigurationStep
    : fusion::JointUnaryVisitorBase< LoadReferenceConfigurationStep<Scalar,Options,JointCollectionTpl> >
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::ConfigVectorType ConfigVectorType;
      
      typedef boost::fusion::vector<const std::string&,
                                    const ConfigVectorType&,
                                    ConfigVectorType&> ArgsType;

      template<typename JointModel>
      static void algo(const JointModelBase<JointModel> & joint,
                       const std::string& joint_name,
                       const ConfigVectorType& fromXML,
                       ConfigVectorType& config)
      {
        _algo (joint.derived(), joint_name, fromXML, config);
      }

      private:
      template<int axis>
      static void _algo (const JointModelRevoluteUnboundedTpl<Scalar,Options,axis> & joint,
                         const std::string& joint_name,
                         const ConfigVectorType& fromXML,
                         ConfigVectorType& config)
      {
        typedef JointModelRevoluteUnboundedTpl<Scalar,Options,axis> JointModelRUB;
        PINOCCHIO_STATIC_ASSERT(JointModelRUB::NQ == 2, JOINT_MODEL_REVOLUTE_SHOULD_HAVE_2_PARAMETERS);
        if (fromXML.size() != 1)
          std::cerr << "Could not read joint config (" << joint_name << " , " << fromXML.transpose() << ")" << std::endl;
        else {
          SINCOS(fromXML[0],
                 &config[joint.idx_q()+1],
                 &config[joint.idx_q()+0]);
        }
      }

      template<typename JointModel>
      static void _algo (const JointModel & joint,
                         const std::string& joint_name,
                         const ConfigVectorType& fromXML,
                         ConfigVectorType& config)
      {
        if (joint.nq() != fromXML.size())
          std::cerr << "Could not read joint config (" << joint_name << " , " << fromXML.transpose() << ")" << std::endl;
        else
          config.segment(joint.idx_q(),joint.nq()) = fromXML;
      }
    };
 

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void
    loadReferenceConfigurations(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                const std::string & filename,
                                const bool verbose)
    {
      // Check extension
      const std::string extension = filename.substr(filename.find_last_of('.')+1);
      if (extension != "srdf")
      {
        const std::string exception_message (filename + " does not have the right extension.");
        throw std::invalid_argument(exception_message);
      }
      
      // Open file
      std::ifstream srdf_stream(filename.c_str());
      if (! srdf_stream.is_open())
      {
        const std::string exception_message (filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }

      loadReferenceConfigurationsFromXML (model, srdf_stream, verbose);
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void
    loadReferenceConfigurationsFromXML(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       std::istream & xmlStream,
                                       const bool verbose)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::JointModel JointModel;
      
      // Read xml stream
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(xmlStream, pt);
      
      // Iterate over all tags directly children of robot
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child("robot"))
      {
        // if we encounter a tag group_state
        if (v.first == "group_state")
        {
          const std::string name = v.second.get<std::string>("<xmlattr>.name");
          typename Model::ConfigVectorType ref_config (model.nq);
          neutral (model, ref_config);
          
          // Iterate over all the joint tags
          BOOST_FOREACH(const ptree::value_type & joint_tag, v.second)
          {
            if (joint_tag.first == "joint")
            {
              std::string joint_name = joint_tag.second.get<std::string>("<xmlattr>.name");
              typename Model::JointIndex joint_id = model.getJointId(joint_name);

              // Search in model the joint and its config id
              if (joint_id != model.joints.size()) // != model.njoints
              {
                const JointModel & joint = model.joints[joint_id];
                typename Model::ConfigVectorType joint_config(joint.nq());
                const std::string joint_val = joint_tag.second.get<std::string>("<xmlattr>.value");
                std::istringstream config_string(joint_val);
                std::vector<double> config_vec((std::istream_iterator<double>(config_string)), std::istream_iterator<double>());
                joint_config = Eigen::Map<Eigen::VectorXd>(config_vec.data(), (Eigen::DenseIndex)config_vec.size());

                typedef LoadReferenceConfigurationStep<Scalar, Options, JointCollectionTpl> LoadReferenceConfigurationStep_t;
                LoadReferenceConfigurationStep_t::run (joint,
                    typename LoadReferenceConfigurationStep_t::ArgsType (joint_name, joint_config, ref_config));
                if (verbose)
                {
                  std::cout << "(" << joint_name << " , " << joint_config.transpose() << ")" << std::endl;
                }
              }
              else
              {
                if (verbose) std::cout << "The Joint " << joint_name << " was not found in model" << std::endl;
              }

            }
          }          

          if ( !model.referenceConfigurations.insert(std::make_pair(name, ref_config)).second)
          {
            //  Element already present...
            if (verbose) std::cout << "The reference configuration "
                                   << name << " has been defined multiple times. "
                                   <<"Only the last instance of "<<name<<" is being used."
                                   <<std::endl;
          }
        }
      } // BOOST_FOREACH
    }
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_parser_srdf_hxx__
