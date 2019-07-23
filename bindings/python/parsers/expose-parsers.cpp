//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include <Python.h>

#include "pinocchio/bindings/python/parsers/parsers.hpp"
#include "pinocchio/bindings/python/parsers/sample-models.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeParsers()
    {
      ParsersPythonVisitor::expose();
      SampleModelsPythonVisitor::expose();
    }
    
  } // namespace python
} // namespace pinocchio

