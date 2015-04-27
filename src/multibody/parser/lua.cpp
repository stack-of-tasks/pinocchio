
#include "pinocchio/multibody/parser/lua/lua_tables.hpp"
#include "pinocchio/multibody/parser/lua.hpp"

#include <lua.hpp>
#include <iostream>
#include <map>
#include <sstream>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/model.hpp"

typedef se3::SE3::Vector3 Vector3;
typedef se3::SE3::Matrix3 Matrix3;

template<> Vector3 LuaTableNode::getDefault<Vector3> (const Vector3 & default_value)
{
  Vector3 result (default_value);

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    if (vector_table.length() != 3) {
      std::cerr << "LuaModel Error: invalid 3d vector!" << std::endl;
      abort();
    }

    result[0] = vector_table[1];
    result[1] = vector_table[2];
    result[2] = vector_table[3];
  }

  stackRestore();

  return result;
}

template<> Matrix3 LuaTableNode::getDefault<Matrix3> (const Matrix3 & default_value)
{
  Matrix3 result (default_value);

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    if (vector_table.length() != 3) {
      std::cerr << "LuaModel Error: invalid 3d matrix!" << std::endl;
      abort();
    }

    if (vector_table[1].length() != 3
        || vector_table[2].length() != 3
        || vector_table[3].length() != 3) {
      std::cerr << "LuaModel Error: invalid 3d matrix!" << std::endl;
      abort();
    }

    result(0,0) = vector_table[1][1];
    result(0,1) = vector_table[1][2];
    result(0,2) = vector_table[1][3];

    result(1,0) = vector_table[2][1];
    result(1,1) = vector_table[2][2];
    result(1,2) = vector_table[2][3];

    result(2,0) = vector_table[3][1];
    result(2,1) = vector_table[3][2];
    result(2,2) = vector_table[3][3];
  }

  stackRestore();

  return result;
}

template<> se3::SE3 LuaTableNode::getDefault<se3::SE3> (const se3::SE3 & default_value)
{
  se3::SE3 result (default_value);

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    result.translation() = vector_table["r"].getDefault<Vector3> (Vector3::Zero (3));
    result.rotation().transpose() = vector_table["E"].getDefault<Matrix3> (Matrix3::Identity (3,3));
  }

  stackRestore();

  return result;
}

template<> se3::Inertia LuaTableNode::getDefault<se3::Inertia> (const se3::Inertia & default_value)
{
  se3::Inertia result (default_value);

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    double mass = 0.;
    Vector3 com;
    Matrix3 inertia_matrix;

    mass = vector_table["mass"];
    com = vector_table["com"].getDefault<Vector3> (Vector3::Zero ());
    inertia_matrix = vector_table["inertia"].getDefault<Matrix3> (Matrix3::Identity ());

    result = se3::Inertia (mass, com, inertia_matrix);
  }

  stackRestore();

  return result;
}

namespace se3
{
  namespace lua
  {
    bool LuaModelReadFromTable (LuaTable & model_table, Model & model, bool freeFlyer, bool verbose)
    {
      typedef std::map<std::string, Model::Index> mapStringIndex_t;
      mapStringIndex_t body_table_id_map;
      mapStringIndex_t fixed_body_table_id_map;

      typedef std::map<std::string, SE3> mapStringSE3_t;
      mapStringSE3_t fixed_placement_map;

      if (model_table["gravity"].exists())
      {
        model.gravity.linear() = model_table["gravity"].get<Vector3> ();

        if (verbose)
          std::cout << "gravity = " << model.gravity.linear().transpose() << std::endl;
      }

      if (! model_table["frames"].exists())
      {
        std::cerr << "Frames table missing from model table - Abort" << std::endl;
        abort();
      }

      size_t frame_count = model_table["frames"].length();

      body_table_id_map["ROOT"] = 0;

      for (int i = 1; i <= (int) frame_count; i++) {

        std::stringstream body_name_default;
        body_name_default << "body " << i;
        std::string body_name = model_table["frames"][i]["name"].getDefault<std::string> (body_name_default.str());

        std::string parent_name;
        if (model_table["frames"][i]["parent"].exists ())
        {
          parent_name = model_table["frames"][i]["parent"].get<std::string> ();
        }
        else if (i == 1) // first body of the tree
        {
          parent_name = "ROOT";
        }
        else // no parent defined
        {
          std::cerr << "Parent not defined for frame " << i << "." << std::endl;
          abort();
        }

        Model::Index parent_id;
        SE3 fixed_placement_offset (SE3::Identity());;

        if (body_table_id_map.find (parent_name) != body_table_id_map.end ())
        {
          parent_id = body_table_id_map[parent_name];
        }
        else if (fixed_body_table_id_map.find(parent_name) != fixed_body_table_id_map.end ()) // the parent is fixed in the kinematic tree
        {
          parent_id = fixed_body_table_id_map[parent_name];
          fixed_placement_offset = fixed_placement_map[parent_name];
        }
        else
        {
          std::cerr << parent_name << " is not in the tree." << std::endl;
          abort ();
        }


        std::stringstream joint_name_default;
        joint_name_default << "joint " << i;
        std::string joint_name = model_table["frames"][i]["joint_name"].getDefault<std::string> (joint_name_default.str());

        
        SE3 joint_placement = model_table["frames"][i]["joint_frame"].getDefault<SE3> (SE3::Identity ());
        SE3 global_placement (fixed_placement_offset * joint_placement); // placement due to the existence of fixed bodies

        if (! model_table["frames"][i]["body"].exists()) {
          std::cerr << "body field not defined for frame " << i << "." << std::endl;
          abort();
        }

        Inertia Y = model_table["frames"][i]["body"].getDefault<Inertia> (Inertia::Identity ());

        std::string joint_type;
        if (model_table["frames"][i]["joint"].exists())
        {
          if (model_table["frames"][i]["joint"].length() == 1)
            joint_type = model_table["frames"][i]["joint"][1].getDefault<std::string> ("");
          else
            std::cerr << "Joint compouned not implemented yet. Take only the first joint." << std::endl;
        }
        else if (i == 1) // first body of the tree
        {
          if (freeFlyer)
            joint_type = "jointFreeFlyer";
          else
          {
            std::cerr << "The first segment is defined without any definition of joint type relatily to the world." << std::endl;
            abort ();
          }
        }
        else
        {
          std::cerr << "joint field not defined for frame " << i << "." << std::endl;
          abort();
        }


        Model::Index body_id;
        bool is_body_fixed = false;
        if (joint_type == "jointRX")
        {
          body_id = model.addBody (parent_id, JointModelRX (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointRY")
        {
          body_id = model.addBody (parent_id, JointModelRY (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointRZ")
        {
          body_id = model.addBody (parent_id, JointModelRZ (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointPX")
        {
          body_id = model.addBody (parent_id, JointModelPX (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointPY")
        {
          body_id = model.addBody (parent_id, JointModelPY (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointPZ")
        {
          body_id = model.addBody (parent_id, JointModelPZ (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointFreeFlyer")
        {
          body_id = model.addBody (parent_id, JointModelFreeFlyer (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointSpherical")
        {
          body_id = model.addBody (parent_id, JointModelSpherical (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointSphericalZYX")
        {
          body_id = model.addBody (parent_id, JointModelSphericalZYX (), global_placement, Y, joint_name, body_name, false);
        }
        else if (joint_type == "jointFixed")
        {
          model.mergeFixedBody(parent_id, global_placement, Y);

          fixed_body_table_id_map[body_name] = parent_id;
          fixed_placement_map[body_name] = global_placement;

          body_id = model.addFixedBody(parent_id, global_placement, body_name, false);
          is_body_fixed = true;
        }
        else
        {
          std::cerr << joint_type << " is not supported.." << std::endl;
          abort ();
        }

        body_table_id_map[body_name] = body_id;

        if (verbose) {
          std::cout << "==== Added Body ====" << std::endl;
          std::cout << "  body name  : " << body_name << std::endl;
          if (! is_body_fixed)
            std::cout << "  body id    : " << body_id << std::endl;
          else
            std::cout << "  fixed body id    : " << body_id << std::endl;
          std::cout << "  closest parent id  : " << parent_id << std::endl;
          std::cout << "  joint frame:\n" << joint_placement << std::endl;
          std::cout << "  joint type : " << joint_type << std::endl;
          std::cout << "  joint name : " << joint_name << std::endl;
          std::cout << "  body inertia:\n" << Y << std::endl;
        }
      }
      
      return true;
    }

    Model buildModel (const std::string & filename, bool freeFlyer, bool verbose)
    {
      Model model;

      LuaTable model_table = LuaTable::fromFile (filename.c_str ());
      LuaModelReadFromTable (model_table, model, freeFlyer, verbose);

      return model;
    }

  } // namespace lua

} // namespace se3
