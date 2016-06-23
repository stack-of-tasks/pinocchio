//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_python_model_hpp__
#define __se3_python_model_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/python/se3.hpp"
#include "pinocchio/python/eigen_container.hpp"
#include "pinocchio/python/handler.hpp"


namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    typedef Handler<Model> ModelHandler;

    struct ModelPythonVisitor
      : public boost::python::def_visitor< ModelPythonVisitor >
    {
    public:
      typedef Model::Index Index;
      typedef Model::JointIndex JointIndex;
      typedef Model::FrameIndex FrameIndex;
      typedef eigenpy::UnalignedEquivalent<Motion>::type Motion_fx;
      typedef eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      typedef eigenpy::UnalignedEquivalent<Inertia>::type Inertia_fx;

      struct add_joint_and_body_visitor : public boost::static_visitor<Model::Index>
      {
        ModelHandler & _model;
        Model::JointIndex & _index_parent;
        const SE3_fx & _placement;
        const Inertia_fx & _inertia;
        const std::string & _jName;
        const std::string & _bName;

        add_joint_and_body_visitor( ModelHandler & model,
                          Model::JointIndex & idx, const SE3_fx & placement,
                          const Inertia_fx & Y, const std::string & jointName,
                          const std::string & bodyName)
                        : _model(model)
                        , _index_parent(idx)
                        , _placement(placement)
                        , _inertia(Y)
                        , _jName(jointName)
                        , _bName(bodyName)
        {}

        template <typename T> Model::Index operator()( T & operand ) const
        {
          return _model->addJointAndBody(_index_parent, operand, _placement, _inertia, _jName, _bName);
        }
      };

    public:

      /* --- Convert From C++ to Python ------------------------------------- */
      // static PyObject* convert(Model const& modelConstRef)
      // {
      //  Model * ptr = const_cast<Model*>(&modelConstRef);
      //  return boost::python::incref(boost::python::object(ModelHandler(ptr)).ptr());
      // }
      static PyObject* convert(ModelHandler::SmartPtr_t const& ptr)
      {
        return boost::python::incref(boost::python::object(ModelHandler(ptr)).ptr());
      }

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
          .def("getBodyId",&ModelPythonVisitor::getBodyId)
          .def("getJointId",&ModelPythonVisitor::getJointId)
          .def("createData",&ModelPythonVisitor::createData)

          .def("__str__",&ModelPythonVisitor::toString)

          .add_property("nq", &ModelPythonVisitor::nq)
          .add_property("nv", &ModelPythonVisitor::nv)
          .add_property("njoint", &ModelPythonVisitor::njoint)
          .add_property("nbody", &ModelPythonVisitor::nbody)
          .add_property("inertias",
            bp::make_function(&ModelPythonVisitor::inertias,
                  bp::return_internal_reference<>())  )
          .add_property("jointPlacements",
            bp::make_function(&ModelPythonVisitor::jointPlacements,
                  bp::return_internal_reference<>())  )
          .add_property("joints",
            bp::make_function(&ModelPythonVisitor::joints,
                  bp::return_internal_reference<>())  )
          .add_property("parents", 
            bp::make_function(&ModelPythonVisitor::parents,
                  bp::return_internal_reference<>())  )
          .add_property("names",
            bp::make_function(&ModelPythonVisitor::names,
                  bp::return_internal_reference<>())  )
          .def("addJointAndBody",&ModelPythonVisitor::addJointAndBodyToModel)


          .add_property("effortLimit", bp::make_function(&ModelPythonVisitor::effortLimit), "Joint max effort")
          .add_property("velocityLimit", bp::make_function(&ModelPythonVisitor::velocityLimit), "Joint max velocity")
          .add_property("lowerPositionLimit", bp::make_function(&ModelPythonVisitor::lowerPositionLimit), "Limit for joint lower position")
          .add_property("upperPositionLimit", bp::make_function(&ModelPythonVisitor::upperPositionLimit), "Limit for joint upper position")

          .def("getFrameParent", &ModelPythonVisitor::getFrameParent)
          .def("getFramePlacement", &ModelPythonVisitor::getFramePlacement)
          .def("addFrame", &ModelPythonVisitor::addFrame)
          .add_property("frames", bp::make_function(&ModelPythonVisitor::operationalFrames, bp::return_internal_reference<>()) )

          .add_property("gravity",&ModelPythonVisitor::gravity,&ModelPythonVisitor::setGravity)
          .def("BuildEmptyModel",&ModelPythonVisitor::maker_empty)
          .staticmethod("BuildEmptyModel")
          .def("BuildHumanoidSimple",&ModelPythonVisitor::maker_humanoidSimple)
          .staticmethod("BuildHumanoidSimple")
          ;
      }

      static Model::Index getBodyId( const ModelHandler & modelPtr, const std::string & name )
      { return  modelPtr->getBodyId(name); }
      static Model::Index getJointId( const ModelHandler & modelPtr, const std::string & name )
      { return  modelPtr->getJointId(name); }
      static boost::shared_ptr<Data> createData(const ModelHandler& m )
      { return boost::shared_ptr<Data>( new Data(*m) );      } 
      
      static int nq( ModelHandler & m ) { return m->nq; }
      static int nv( ModelHandler & m ) { return m->nv; }
      static int njoint( ModelHandler & m ) { return m->njoint; }
      static int nbody( ModelHandler & m ) { return m->nbody; }
      static std::vector<Inertia> & inertias( ModelHandler & m ) { return m->inertias; }
      static std::vector<SE3> & jointPlacements( ModelHandler & m ) { return m->jointPlacements; }
      static JointModelVector & joints( ModelHandler & m ) { return m->joints; }
      static std::vector<Model::JointIndex> & parents( ModelHandler & m ) { return m->parents; }
      static std::vector<std::string> & names ( ModelHandler & m ) { return m->names; }

      static Model::Index addJointAndBodyToModel(ModelHandler & modelPtr,
                                          Model::JointIndex idx, bp::object joint,
                                          const SE3_fx & placement,
                                          const Inertia_fx & Y,
                                          const std::string & jointName,
                                          const std::string & bodyName)
      { 
        JointModelVariant variant = bp::extract<JointModelVariant> (joint);
        return boost::apply_visitor(add_joint_and_body_visitor(modelPtr, idx, placement, Y, jointName, bodyName), variant);
      }


      static Eigen::VectorXd effortLimit(ModelHandler & m) {return m->effortLimit;}
      static Eigen::VectorXd velocityLimit(ModelHandler & m) {return m->velocityLimit;}
      static Eigen::VectorXd lowerPositionLimit(ModelHandler & m) {return m->lowerPositionLimit;}
      static Eigen::VectorXd upperPositionLimit(ModelHandler & m) {return m->upperPositionLimit;}

      static Model::JointIndex  getFrameParent( ModelHandler & m, const std::string & name ) { return m->getFrameParent(name); }
      static SE3  getFramePlacement( ModelHandler & m, const std::string & name ) { return m->getFramePlacement(name); }
      static void  addFrame( ModelHandler & m, const std::string & frameName, const JointIndex parent, const SE3_fx & placementWrtParent )
      {
        m->addFrame(frameName, parent, placementWrtParent);
      }
      static std::vector<Frame> & operationalFrames (ModelHandler & m ) { return m->frames;}

      static Motion gravity( ModelHandler & m ) { return m->gravity; }
      static void setGravity( ModelHandler & m,const Motion_fx & g ) { m->gravity = g; }


      static ModelHandler maker_empty()
      {
        return ModelHandler( new Model(),true );
      }
      static ModelHandler maker_humanoidSimple()
      {
        Model * model = new Model();
        buildModels::humanoidSimple(*model);
        return ModelHandler( model,true );
      }

      static std::string toString(const ModelHandler& m) 
      {   std::ostringstream s; s << *m; return s.str();       }

      ///
      /// \brief Provide equivalent to python list index function for
      ///        vectors.
      ///
      /// \param[in] x The input vector.
      /// \param[in] v The value of to look for in the vector.
      ///
      /// \return The index of the matching element of the vector. If
      ///         no element is found, return the size of the vector.
      ///
      template<typename T>
      static Model::Index index(std::vector<T> const& x,
                                typename std::vector<T>::value_type const& v)
      {
        Model::Index i = 0;
        for(typename std::vector<T>::const_iterator it = x.begin(); it != x.end(); ++it, ++i)
        {
          if(*it == v)
          {
            return i;
          }
        }
        return x.size();
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_< std::vector<Index> >("StdVec_Index")
          .def(bp::vector_indexing_suite< std::vector<Index> >());
        bp::class_< std::vector<std::string> >("StdVec_StdString")
          .def(bp::vector_indexing_suite< std::vector<std::string> >())
          .def("index", &ModelPythonVisitor::index<std::string>);
        bp::class_< std::vector<bool> >("StdVec_Bool")
          .def(bp::vector_indexing_suite< std::vector<bool> >());
        bp::class_< std::vector<double> >("StdVec_double")
          .def(bp::vector_indexing_suite< std::vector<double> >());
        bp::class_< JointModelVector >("StdVec_JointModelVector")
          .def(bp::vector_indexing_suite< JointModelVector, true >());

        bp::class_<ModelHandler>("Model",
                                 "Articulated rigid body model (const)",
                                 bp::no_init)
          .def(ModelPythonVisitor());
      
        /* Not sure if it is a good idea to enable automatic
         * conversion. Prevent it for now */
        //bp::to_python_converter< Model,ModelPythonVisitor >();
        bp::to_python_converter< ModelHandler::SmartPtr_t,ModelPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_model_hpp__

