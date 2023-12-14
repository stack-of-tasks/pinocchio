//
// Copyright (c) 2017-2021 CNRS INRIA
//

#ifndef __pinocchio_python_geometry_object_hpp__
#define __pinocchio_python_geometry_object_hpp__

#include <boost/python.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/multibody/geometry.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryObject)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    namespace
    {
      /// Convert GeometryMaterial boost variant to a Python object.
      /// This converter copy the GeometryMaterial content.
      struct GeometryMaterialValueToObject : boost::static_visitor<PyObject*>
      {
        static result_type convert(GeometryMaterial const & gm)
        {
          return apply_visitor(GeometryMaterialValueToObject(), gm);
        }

        template<typename T>
        result_type operator()(T & t) const
        {
          return bp::incref(bp::object(t).ptr());
        }
      };

      /// Convert GeometryMaterial boost variant to a Python object.
      /// This converter return the GeometryMaterial reference.
      /// The code the create the reference holder is taken from \see boost::python::to_python_indirect.
      struct GeometryMaterialRefToObject : boost::static_visitor<PyObject*>
      {
        static result_type convert(GeometryMaterial const & gm)
        {
          return apply_visitor(GeometryMaterialRefToObject(), gm);
        }

        template<typename T>
        result_type operator()(T & t) const
        {
          return bp::detail::make_reference_holder::execute(&t);
        }
      };

      /// Converter used in \see ReturnInternalVariant.
      /// This is inspired by \see boost::python::reference_existing_object.
      ///  It will call GeometryMaterialRefToObject to extract the variant reference.
      struct GeometryMaterialConverter
      {
        template <class T>
        struct apply
        {
          struct type
          {
            inline PyObject* operator()(const GeometryMaterial& gm) const
            {
              return GeometryMaterialRefToObject::convert(gm);
            }

#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
            inline PyTypeObject const* get_pytype()const
            {
              return bp::converter::registered_pytype<GeometryMaterial>::get_pytype();
            }
#endif
          };
        };
      };

      /// Variant of \see boost::python::return_internal_reference that
      /// extract GeometryMaterial variant before converting it into a PyObject*
      struct GeometryMaterialReturnInternalVariant : bp::return_internal_reference<> {
       public:
        typedef GeometryMaterialConverter result_converter;
      };
    }



    struct GeometryObjectPythonVisitor
    : public boost::python::def_visitor< GeometryObjectPythonVisitor >
    {

      typedef GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;

      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string,FrameIndex,JointIndex,CollisionGeometryPtr,SE3,
                      bp::optional<std::string,Eigen::Vector3d,bool,Eigen::Vector4d,std::string,GeometryMaterial> >
             (
             bp::args("self","name","parent_frame","parent_joint","collision_geometry",
                      "placement", "mesh_path", "mesh_scale", "override_material", "mesh_color", "mesh_texture_path",
                      "mesh_material"),
             "Full constructor of a GeometryObject."))
        .def(bp::init<std::string,JointIndex,CollisionGeometryPtr,SE3,
                      bp::optional<std::string,Eigen::Vector3d,bool,Eigen::Vector4d,std::string,GeometryMaterial> >
             (
              bp::args("self","name","parent_joint","collision_geometry",
                       "placement", "mesh_path", "mesh_scale", "override_material", "mesh_color", "mesh_texture_path",
                       "mesh_material"),
              "Reduced constructor of a GeometryObject. This constructor does not require to specify the parent frame index."
              ))
        .def(bp::init<const GeometryObject&>
             (
              bp::args("self","otherGeometryObject"),
              "Copy constructor"
              ))
        .add_property("meshScale",
                      bp::make_getter(&GeometryObject::meshScale,
                                      bp::return_internal_reference<>()),
                      bp::make_setter(&GeometryObject::meshScale),
                       "Scaling parameter of the mesh.")
        .add_property("meshColor",
                      bp::make_getter(&GeometryObject::meshColor,
                                      bp::return_internal_reference<>()),
                      bp::make_setter(&GeometryObject::meshColor),
                      "Color rgba of the mesh.")
        .def_readwrite("geometry", &GeometryObject::geometry,
                       "The FCL CollisionGeometry associated to the given GeometryObject.")
        .def_readwrite("name", &GeometryObject::name,
                       "Name associated to the given GeometryObject.")
        .def_readwrite("parentJoint", &GeometryObject::parentJoint,
                       "Index of the parent joint.")
        .def_readwrite("parentFrame", &GeometryObject::parentFrame,
                       "Index of the parent frame.")
        .def_readwrite("placement",&GeometryObject::placement,
                       "Position of geometry object in parent joint's frame.")
        .def_readwrite("meshPath", &GeometryObject::meshPath,
                       "Path to the mesh file.")
        .def_readwrite("overrideMaterial", &GeometryObject::overrideMaterial,
                       "Boolean that tells whether material information is stored inside the given GeometryObject.")
        .def_readwrite("meshTexturePath", &GeometryObject::meshTexturePath,
                       "Path to the mesh texture file.")
        .def_readwrite("disableCollision", &GeometryObject::disableCollision,
                       "If true, no collision or distance check will be done between the Geometry and any other geometry.")
        .add_property("meshMaterial",
                      bp::make_getter(&GeometryObject::meshMaterial,
                                      GeometryMaterialReturnInternalVariant()),
                      bp::make_setter(&GeometryObject::meshMaterial),
                      "Material associated to the mesh (applied only if overrideMaterial is True)")

        .def(bp::self == bp::self)
        .def(bp::self != bp::self)

#ifdef PINOCCHIO_WITH_HPP_FCL
          .def("CreateCapsule", &GeometryObjectPythonVisitor::maker_capsule)
          .staticmethod("CreateCapsule")
#endif // PINOCCHIO_WITH_HPP_FCL
        ;

        // Check registration
        {
          const bp::type_info info = bp::type_id<CollisionGeometryPtr>();
          const bp::converter::registration* reg = bp::converter::registry::query(info);
          // We just need to check if the type shared_ptr<CollisionGeometry> exist in the registry
          if(!reg)
            bp::register_ptr_to_python<CollisionGeometryPtr>();
        }
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      static GeometryObject maker_capsule(const double radius, const double length)
      {
        return GeometryObject("",FrameIndex(0),JointIndex(0),
                              pinocchio::shared_ptr<fcl::CollisionGeometry>(new fcl::Capsule(radius, length)),
                              SE3::Identity());

      }
#endif // PINOCCHIO_WITH_HPP_FCL

      static GeometryMaterial get_content(const GeometryMaterial& gm)
      {
        return gm;
      }

      static void expose()
      {
        /// Define material types
        bp::class_<GeometryNoMaterial>("GeometryNoMaterial", bp::init<>())
        .def(bp::init<GeometryNoMaterial>());
        bp::class_<GeometryPhongMaterial>("GeometryPhongMaterial", bp::init<>())
        .def(bp::init<GeometryPhongMaterial>())
        .def(bp::init<Eigen::Vector4d, Eigen::Vector4d, double>())
        .add_property("meshEmissionColor",
                      bp::make_getter(&GeometryPhongMaterial::meshEmissionColor,
                                      bp::return_internal_reference<>()),
                      bp::make_setter(&GeometryPhongMaterial::meshEmissionColor),
                      "RGBA emission (ambient) color value of the mesh")
        .add_property("meshSpecularColor",
                      bp::make_getter(&GeometryPhongMaterial::meshSpecularColor,
                                      bp::return_internal_reference<>()),
                      bp::make_setter(&GeometryPhongMaterial::meshSpecularColor),
                      "RGBA specular value of the mesh")
        .def_readwrite("meshShininess", &GeometryPhongMaterial::meshShininess,
                       "Shininess associated to the specular lighting model (between 0 and 1)");

        /// Define material conversion from C++ variant to python object
        bp::to_python_converter<GeometryMaterial, GeometryMaterialValueToObject>();

        /// Define material conversion from python object to C++ object
        bp::implicitly_convertible<GeometryNoMaterial, GeometryMaterial>();
        bp::implicitly_convertible<GeometryPhongMaterial, GeometryMaterial>();

        bp::class_<GeometryObject>("GeometryObject",
                                   "A wrapper on a collision geometry including its parent joint, parent frame, placement in parent joint's frame.\n\n",
                                   bp::no_init
                                   )
        .def(GeometryObjectPythonVisitor())
        ;

        bp::enum_<GeometryType>("GeometryType")
        .value("VISUAL",VISUAL)
        .value("COLLISION",COLLISION)
        .export_values()
        ;
      }

    };


  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_object_hpp__
