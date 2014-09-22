#include "pinocchio/python/python.hpp"
#include "pinocchio/python/se3.hpp"
#include "pinocchio/python/force.hpp"
#include "pinocchio/python/motion.hpp"
#include "pinocchio/python/inertia.hpp"

namespace se3
{
  namespace python
  {
    void exposeSE3()
    {
      SE3PythonVisitor<SE3>::expose();
    }
    void exposeForce()
    {
      ForcePythonVisitor<Force>::expose();
    }
    void exposeMotion()
    {
      MotionPythonVisitor<Motion>::expose();
    }
    void exposeInertia()
    {
      InertiaPythonVisitor<Inertia>::expose();
    }
    void exposeStatics()
    {
 

    }
    
    

  }} // namespace se3::python
