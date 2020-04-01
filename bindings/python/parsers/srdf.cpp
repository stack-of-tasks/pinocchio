//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/bindings/python/parsers/srdf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    BOOST_PYTHON_FUNCTION_OVERLOADS(removeCollisionPairs_overload,
                                    srdf::removeCollisionPairs,
                                    3,4)
  
    BOOST_PYTHON_FUNCTION_OVERLOADS(removeCollisionPairsFromXML_overload,
                                    srdf::removeCollisionPairsFromXML,
                                    3,4)

    BOOST_PYTHON_FUNCTION_OVERLOADS(loadReferenceConfigurations_overload,
                                    srdf::loadReferenceConfigurations,
                                    2,3)
      
    void loadReferenceConfigurationsFromXML(Model & model,
                                            const std::string & xmlStream,
                                            const bool verbose = false)
    {
      std::istringstream iss (xmlStream);
      pinocchio::srdf::loadReferenceConfigurationsFromXML(model, iss, verbose);
    }

    BOOST_PYTHON_FUNCTION_OVERLOADS(loadReferenceConfigurationsFromXML_overload,
                                    loadReferenceConfigurationsFromXML,
                                    2,3)
      
    BOOST_PYTHON_FUNCTION_OVERLOADS(loadRotorParameters_overload,
                                    srdf::loadRotorParameters,
                                    2,3)
  
    void exposeSRDFParser()
    {

      bp::def("removeCollisionPairs",
              static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(&srdf::removeCollisionPairs),
              removeCollisionPairs_overload(bp::args("model", "geom_model","srdf_filename", "verbose"),
                                            "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n""Parameters:\n"
                                            "Parameters:\n"
                                            "\tmodel: model of the robot\n"
                                            "\tgeom_model: geometry model of the robot\n"
                                            "\tsrdf_filename: path to the SRDF file containing the collision pairs to remove\n"
                                            "\tverbose: [optional] display to the current terminal some internal information"
                                            ));
      
      bp::def("removeCollisionPairsFromXML",
              static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(&srdf::removeCollisionPairsFromXML),
              removeCollisionPairsFromXML_overload(bp::args("model", "geom_model","srdf_xml_stream", "verbose"),
                                                   "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n""Parameters:\n"
                                                   "Parameters:\n"
                                                   "\tmodel: model of the robot\n"
                                                   "\tgeom_model: geometry model of the robot\n"
                                                   "\tsrdf_xml_stream: XML stream containing the SRDF information with the collision pairs to remove\n"
                                                   "\tverbose: [optional] display to the current terminal some internal information"
                                                   ));

      bp::def("loadReferenceConfigurations",
              static_cast<void (*)(Model &, const std::string &, const bool)>(&srdf::loadReferenceConfigurations),
              loadReferenceConfigurations_overload(bp::args("model","srdf_filename","verbose"),
                                                   "Retrieve all the reference configurations of a given model from the SRDF file.\n"
                                                   "Parameters:\n"
                                                   "\tmodel: model of the robot\n"
                                                   "\tsrdf_filename: path to the SRDF file containing the reference configurations\n"
                                                   "\tverbose: [optional] display to the current terminal some internal information"
                                                   ));

      bp::def("loadReferenceConfigurationsFromXML",
              static_cast<void (*)(Model &, const std::string &, const bool)>(&srdf::loadReferenceConfigurations),
              loadReferenceConfigurationsFromXML_overload(bp::args("model","srdf_xml_stream","verbose"),
                                                          "Retrieve all the reference configurations of a given model from the SRDF file.\n"
                                                          "Parameters:\n"
                                                          "\tmodel: model of the robot\n"
                                                          "\tsrdf_xml_stream: XML stream containing the SRDF information with the reference configurations\n"
                                                          "\tverbose: [optional] display to the current terminal some internal information"
                                                          ));
      
      bp::def("loadRotorParameters",
              static_cast<bool (*)(Model &, const std::string &, const bool)>(&srdf::loadRotorParameters),
              loadRotorParameters_overload(bp::args("model","srdf_filename","verbose"),
                                           "Load the rotor parameters of a given model from a SRDF file.\n"
                                           "Results are stored in model.rotorInertia and model.rotorGearRatio."
                                           "Parameters:\n"
                                           "\tmodel: model of the robot\n"
                                           "\tsrdf_filename: path to the SRDF file containing the rotor parameters\n"
                                           "\tverbose: [optional] display to the current terminal some internal information"
                                           ));
    }
  }
}
