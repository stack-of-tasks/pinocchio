//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/bindings/python/parsers/urdf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
  
#ifdef PINOCCHIO_WITH_URDFDOM
    
    Model buildModelFromUrdf(const std::string & filename)
    {
      Model model;
      pinocchio::urdf::buildModel(filename, model);
      return model;
    }
    
    Model & buildModelFromUrdf(const std::string & filename,
                               Model & model)
    {
      return pinocchio::urdf::buildModel(filename, model);
    }

    Model buildModelFromUrdf(const std::string & filename,
                             bp::object & root_joint_object)
    {
      JointModelVariant root_joint = bp::extract<JointModelVariant>(root_joint_object)();
      Model model;
      pinocchio::urdf::buildModel(filename, root_joint, model);
      return model;
    }
    
    Model & buildModelFromUrdf(const std::string & filename,
                               bp::object & root_joint_object,
                               Model & model)
    {
      JointModelVariant root_joint = bp::extract<JointModelVariant>(root_joint_object)();
      return pinocchio::urdf::buildModel(filename, root_joint, model);
    }
    
    Model buildModelFromXML(const std::string & XMLstream,
                            bp::object & root_joint_object)
    {
      JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
      Model model;
      pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
      return model;
    }
    
    Model & buildModelFromXML(const std::string & XMLstream,
                              bp::object & root_joint_object,
                              Model & model)
    {
      JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
      pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
      return model;
    }
    
    Model buildModelFromXML(const std::string & XMLstream)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(XMLstream, model);
      return model;
    }
    
    Model & buildModelFromXML(const std::string & XMLstream,
                              Model & model)
    {
      pinocchio::urdf::buildModelFromXML(XMLstream, model);
      return model;
    }
  
#endif
  
    void exposeURDFModel()
    {
        
#ifdef PINOCCHIO_WITH_URDFDOM
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &, bp::object &)> (pinocchio::python::buildModelFromUrdf),
              bp::args("urdf_filename","root_joint"),
              "Parse the URDF file given in input and return a pinocchio Model starting with the given root joint."
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &)> (pinocchio::python::buildModelFromUrdf),
              bp::args("urdf_filename"),
              "Parse the URDF file given in input and return a pinocchio Model."
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model & (*) (const std::string &, Model &)> (pinocchio::python::buildModelFromUrdf),
              bp::args("urdf_filename","model"),
              "Append to a given model a URDF structure given by its filename.",
              bp::return_internal_reference<2>()
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model & (*) (const std::string &, bp::object &, Model &)> (pinocchio::python::buildModelFromUrdf),
              bp::args("urdf_filename","root_joint","model"),
              "Append to a given model a URDF structure given by its filename and the root joint.",
              bp::return_internal_reference<3>()
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model (*) (const std::string &, bp::object &)> (pinocchio::python::buildModelFromXML),
              bp::args("urdf_xml_stream","root_joint"),
              "Parse the URDF XML stream given in input and return a pinocchio Model starting with the given root joint."
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model & (*) (const std::string &, bp::object &, Model &)> (pinocchio::python::buildModelFromXML),
              bp::args("urdf_xml_stream","root_joint","model"),
              "Parse the URDF XML stream given in input and append it to the input model with the given interfacing joint.",
              bp::return_internal_reference<3>()
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model (*) (const std::string &)> (pinocchio::python::buildModelFromXML),
              bp::args("urdf_xml_stream"),
              "Parse the URDF XML stream given in input and return a pinocchio Model."
              );
              
      bp::def("buildModelFromXML",
              static_cast <Model & (*) (const std::string &, Model &)> (pinocchio::python::buildModelFromXML),
              bp::args("urdf_xml_stream","model"),
              "Parse the URDF XML stream given in input and append it to the input model.",
              bp::return_internal_reference<2>()
              );
#endif
    }
  }
}
