//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_parsers_urdf_hpp__
#define __pinocchio_python_parsers_urdf_hpp__

namespace pinocchio
{
  namespace python
  {
    void exposeURDFModel();
    void exposeURDFGeometry();
  
    inline void exposeURDFParser()
    {
      exposeURDFModel();
      exposeURDFGeometry();
    }
  
  }
}

#endif // ifndef __pinocchio_python_parsers_urdf_hpp__
