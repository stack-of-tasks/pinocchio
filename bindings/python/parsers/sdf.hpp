//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_python_parsers_sdf_hpp__
#define __pinocchio_python_parsers_sdf_hpp__

namespace pinocchio
{
  namespace python
  {
    void exposeSDFModel();
    void exposeSDFGeometry();
  
    inline void exposeSDFParser()
    {
      exposeSDFModel();
      exposeSDFGeometry();
    }
  }
}

#endif // ifndef __pinocchio_python_parsers_sdf_hpp__
