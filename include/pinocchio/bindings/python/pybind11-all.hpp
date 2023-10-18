// No header guard on purpose because the file can be included several times
// with different value for preprocessor variables SCALAR, OPTIONS and
// JOINT_MODEL_COLLECTION

/// \page default_type_caster Predefined casters
///
/// At the time of writting, this exposes
/// \li ModelTpl
/// \li DataTpl
/// \li SE3Tpl
/// \li MotionTpl
/// \li ForceTpl
/// \li GeometryModel
/// \li (not yet) GeometryData

#if !defined SCALAR or !defined OPTIONS or !defined JOINT_MODEL_COLLECTION
#error \
    "You must define SCALAR, OPTIONS and JOINT_MODEL_COLLECTION before including this file."
#endif

#include <pinocchio/bindings/python/pybind11.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

// Required to be able to pass argument with commas to macros
#define _SINGLE_ARG(...) __VA_ARGS__
#define _PINOCCHIO_PYBIND11_EXPOSE(type, name)  \
  PINOCCHIO_PYBIND11_ADD_ALL_CONVERT_TYPE(_SINGLE_ARG(type)) \
  PINOCCHIO_PYBIND11_TYPE_CASTER(_SINGLE_ARG(type), name)


_PINOCCHIO_PYBIND11_EXPOSE(_SINGLE_ARG(::pinocchio::SE3Tpl<SCALAR,OPTIONS>),
                           _("pinocchio.pinocchio_pywrap.SE3"))
_PINOCCHIO_PYBIND11_EXPOSE(_SINGLE_ARG(::pinocchio::MotionTpl<SCALAR, OPTIONS>),
                           _("pinocchio.pinocchio_pywrap.Motion"))
_PINOCCHIO_PYBIND11_EXPOSE(_SINGLE_ARG(::pinocchio::ForceTpl<SCALAR, OPTIONS>),
                           _("pinocchio.pinocchio_pywrap.Force"))

_PINOCCHIO_PYBIND11_EXPOSE(
    _SINGLE_ARG(::pinocchio::ModelTpl<SCALAR, OPTIONS, JOINT_MODEL_COLLECTION>),
    _("pinocchio.pinocchio_pywrap.Model"))
_PINOCCHIO_PYBIND11_EXPOSE(
    _SINGLE_ARG(::pinocchio::DataTpl<SCALAR, OPTIONS, JOINT_MODEL_COLLECTION>),
    _("pinocchio.pinocchio_pywrap.Model"))

_PINOCCHIO_PYBIND11_EXPOSE(::pinocchio::GeometryModel,
                           _("pinocchio.pinocchio_pywrap.GeometryModel"))
// \todo this triggers a warning because GeometryData has
// a copy constructor and no operator=
// _PINOCCHIO_PYBIND11_EXPOSE(::pinocchio::GeometryData,
//                           _("pinocchio.pinocchio_pywrap.GeometryData"))

#undef _PINOCCHIO_PYBIND11_EXPOSE
#undef _SINGLE_ARG
