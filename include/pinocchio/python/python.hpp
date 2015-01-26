#ifndef __se3_python_python_hpp__
#define __se3_python_python_hpp__

namespace se3
{
  namespace python
  {
    void exposeSE3();
    void exposeForce();
    void exposeMotion();
    void exposeInertia();

    void exposeModel();
    void exposeAlgorithms();
    void exposeParsers();

  }} // namespace se3::python

#endif // ifndef __se3_python_python_hpp__

