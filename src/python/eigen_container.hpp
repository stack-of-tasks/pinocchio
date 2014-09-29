#ifndef __se3_python_eigen_container_hpp__
#define __se3_python_eigen_container_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/python/return_internal_reference.hpp>

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    /* Expose a std::vector containing aligned (Eigen) objects, whose unaligned
     * correspondance is defined in eigenpy::UnalignedEquivalent.
     * Simply call the "expose" method from inside the python module. 
     */
    template< typename EigenObject >
    struct PyWraperForAlignedStdVector
      : public boost::python::def_visitor< PyWraperForAlignedStdVector<EigenObject> >
    {
      typedef typename eigenpy::UnalignedEquivalent<EigenObject>::type EigenObject_fx;
      typedef std::vector<EigenObject> stdVectorAligned;
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def("__getitem__", &PyWraperForAlignedStdVector::getItem)
	  .def("__setitem__", &PyWraperForAlignedStdVector::setItem)
	  .def("__len__",     &PyWraperForAlignedStdVector::length)
	  ;
      }

      static EigenObject getItem( const stdVectorAligned & Ys,int i)
      {
	assert( Ys.size()<INT_MAX );
	if( i<0 ) i = int(Ys.size())+i;
	assert( (i>=0) && (i<int(Ys.size())) );
	return Ys[i]; 
      }

      static void setItem( stdVectorAligned & Ys,
			   int i,const EigenObject_fx & Y)
      { 
	assert( Ys.size()<INT_MAX );
	if( i<0 ) i = int(Ys.size())+i;
	assert( (i>=0) && (i<int(Ys.size())) );
	Ys[i] = Y; 
      }
      static typename stdVectorAligned::size_type length( const stdVectorAligned & Ys )
      { return Ys.size(); }

      static void expose(const std::string & className)
      {
	bp::class_<stdVectorAligned>(className.c_str())
	  .def(PyWraperForAlignedStdVector());
      }
    };

  }} // namespace se3::python

#endif // ifndef __se3_python_eigen_container_hpp__

