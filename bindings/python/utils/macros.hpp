//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_utils_macros_hpp__
#define __pinocchio_python_utils_macros_hpp__

#define PINOCCHIO_ADD_PROPERTY(STRUCT_NAME,PROPERTY_NAME,DOC)  \
      def_readwrite(#PROPERTY_NAME,                            \
      &STRUCT_NAME::PROPERTY_NAME,                             \
      DOC)
      
#define PINOCCHIO_ADD_PROPERTY_READONLY(STRUCT_NAME,PROPERTY_NAME,DOC)  \
      def_readonly(#PROPERTY_NAME,                                      \
      &STRUCT_NAME::PROPERTY_NAME,                                      \
      DOC)
      
#define PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(STRUCT_NAME,PROPERTY_NAME,DOC)                 \
      add_property(#PROPERTY_NAME,                                                             \
      make_getter(&STRUCT_NAME::PROPERTY_NAME,                                                 \
                  ::boost::python::return_value_policy< ::boost::python::return_by_value>()),  \
      DOC)

#endif // ifndef __pinocchio_python_utils_macros_hpp__
