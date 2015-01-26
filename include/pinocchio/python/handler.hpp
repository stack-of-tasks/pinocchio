#ifndef __se3_python_handler_hpp__
#define __se3_python_handler_hpp__

#include <boost/shared_ptr.hpp>

namespace se3
{
  namespace python
  {

    /* This handler is designed first for holding the C++ Model object, then
     * generalized for the C++ Data object. It might be used as well for
     * handling other object whose ownership is shared between python and C++.
     *
     * There might be several way of building and owning a model object:
     *   - build from C++ (as static or dynamic, wathever), and owned by
     * C++. In that case, use a ModelHandler with no ownership ModelHandler(
     * Model*,false).
     *   - build from C++ but owned by python. Python would be in charge of
     * destroying it when it is done. In that case, use
     * ModelHandler(Model*,true). Take care not to destroy the object by
     * yourself, otherwise there might have access to deallocated memory and
     * double destruction.
     *   - build and managed by shared_ptr. It is the best way, in the sense
     * that the object is manage both by C++ and python. It will be released
     * only when both are done with it.  In that case, simply give Python the
     * shared_ptr ModelHandler( shared_ptr<Model> ).
     * 
     * It is not possible to construct a Model from python, because of Eigen
     * memory alignement.  The python visitor does not define any
     * constructor. Instead, some static makers (function allocating memory and
     * returning a pointer Model *) are implemented.  When Python is making
     * such a Model, it is stored in a shared_ptr and can be access directly
     * from C++. By default, when passing a Model to python, it is kept as a
     * raw pointer. Access is then done from python to the C++ memory without
     * any guarantee. If you want to enforce memory access, prefer transmitting
     * a shared_ptr to python.
     */
    template<typename CppObject>
    struct Handler
    {
      typedef boost::shared_ptr<CppObject> SmartPtr_t;
      typedef CppObject * Ptr_t;

      SmartPtr_t smptr;
      Ptr_t rawptr;
      bool smart;

      Handler(CppObject * cppobj,bool transmitOwnership=false)
	: smptr( transmitOwnership ? cppobj : NULL )
	, rawptr( cppobj )
	, smart( transmitOwnership ) {}
      Handler( SmartPtr_t cppobj )
	: smptr(cppobj), rawptr(NULL), smart(true) {}
      ~Handler()
      {    
	//std::cout << "Destroy cppobj handler " << std::endl;
	if( (!smart) && (rawptr!=NULL) ) delete rawptr;
      }

      CppObject *       ptr()              { return smart ?  smptr.get() :  rawptr; }
      const CppObject * ptr()        const { return smart ?  smptr.get() :  rawptr; }
      CppObject *       operator->()       { return ptr(); }
      const CppObject * operator->() const { return ptr(); }

      CppObject &       get()              { return smart ? *smptr       : *rawptr; }
      const CppObject & get()        const { return smart ? *smptr       : *rawptr; }
      CppObject &       operator*()        { return get(); }
      const CppObject&  operator*()  const { return get(); }
    };


  }} // namespace se3::python

#endif // ifndef __se3_python_handler_hpp__

