/*
 * LuaTables++
 * Code comming from https://bitbucket.org/rbdl/rbdl
 * Copyright (c) 2013-2014 Martin Felis <martin@fyxs.org>
 * Copyright (c) 2015 Justin Carpentier <jcarpent@laas.fr>
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __se3_parser_lua_tables_hpp____
#define __se3_parser_lua_tables_hpp____

#include <iostream>
#include <assert.h>
#include <cstdlib>
#include <string>
#include <vector>


// Forward declaration for Lua
extern "C" {
  struct lua_State;
}

struct LuaKey {
  enum Type {
    String,
    Integer
  };

  Type type;
  int int_value;
  std::string string_value;

  bool operator<( const LuaKey & rhs ) const {
    if (type == String && rhs.type == Integer) {
      return false;
    } else if (type == Integer && rhs.type == String) {
      return true;
    } else if (type == Integer && rhs.type == Integer) {
      return int_value < rhs.int_value;
    }

    return string_value < rhs.string_value;
  }

  LuaKey (const char* key_value) :
  type (String),
  int_value (0),
  string_value (key_value) { }
  LuaKey (int key_value) :
  type (Integer),
  int_value (key_value),
  string_value ("") {}
};

inline std::ostream & operator<<(std::ostream & output, const LuaKey & key) {
  if (key.type == LuaKey::Integer)
    output << key.int_value << "(int)";
  else
    output << key.string_value << "(string)";
  return output;
}
struct LuaTable;
struct LuaTableNode;

struct LuaTableNode {
  LuaTableNode() :
  parent (NULL),
  luaTable (NULL),
  key("")
  { }
  LuaTableNode operator[](const char* child_str) {
    LuaTableNode child_node;

    child_node.luaTable = luaTable;
    child_node.parent = this;
    child_node.key = LuaKey (child_str);

    return child_node;
  }
  LuaTableNode operator[](int child_index) {
    LuaTableNode child_node;

    child_node.luaTable = luaTable;
    child_node.parent = this;
    child_node.key = LuaKey (child_index);

    return child_node;
  }
  bool stackQueryValue();
  void stackPushKey();
  void stackCreateValue();
  void stackRestore();
  LuaTable stackQueryTable();
  LuaTable stackCreateLuaTable();

  std::vector<LuaKey> getKeyStack();
  std::string keyStackToString();

  bool exists();
  void remove();
  size_t length();
  std::vector<LuaKey> keys();

  // Templates for setters and getters. Can be specialized for custom
  // types.
  template <typename T>
  void set (const T & value);
  template <typename T>
  T getDefault (const T & default_value);

  template <typename T>
  T get() {
    if (!exists()) {
      std::cerr << "Error: could not find value " << keyStackToString() << "." << std::endl;
      abort();
    }
    return getDefault (T());
  }

  // convenience operators (assignment, conversion, comparison)
  template <typename T>
  void operator=(const T & value) {
    set<T>(value);
  }
  template <typename T>
  operator T() {
    return get<T>();
  }
  template <typename T>
  bool operator==(T value) {
    return value == get<T>();
  }
  template <typename T>
  bool operator!=(T value) {
    return value != get<T>();
  }

  LuaTableNode *parent;
  LuaTable *luaTable;
  LuaKey key;
  int stackTop;
};

template<typename T>
bool operator==(T value, LuaTableNode node) {
  return value == (T) node;
}
template<typename T>
bool operator!=(T value, LuaTableNode node) {
  return value != (T) node;
}

template<> bool LuaTableNode::getDefault<bool>(const bool & default_value);
template<> double LuaTableNode::getDefault<double>(const double & default_value);
template<> float LuaTableNode::getDefault<float>(const float & default_value);
template<> std::string LuaTableNode::getDefault<std::string>(const std::string & default_value);

template<> void LuaTableNode::set<bool>(const bool & value);
template<> void LuaTableNode::set<float>(const float & value);
template<> void LuaTableNode::set<double>(const double & value);
template<> void LuaTableNode::set<std::string>(const std::string & value);

struct LuaTable {
  LuaTable() :
  filename (""),
  L (NULL),
  deleteLuaState (false)
  {}
  LuaTable & operator= (const LuaTable & luatable);
  ~LuaTable();

  LuaTableNode operator[] (const char* key) {
    LuaTableNode root_node;
    root_node.key = LuaKey (key);
    root_node.parent = NULL;
    root_node.luaTable = this;
    
    return root_node;
  }
  LuaTableNode operator[] (int key) {
    LuaTableNode root_node;
    root_node.key = LuaKey (key);
    root_node.parent = NULL;
    root_node.luaTable = this;
    
    return root_node;
  }
  int length();
  void addSearchPath (const char* path);
  std::string serialize ();
  std::string orderedSerialize ();
  
  static LuaTable fromFile (const char *_filename);
  static LuaTable fromLuaExpression (const char* lua_expr);
  static LuaTable fromLuaState (lua_State *L);
  
  std::string filename;
  lua_State *L;
  bool deleteLuaState;
};


#endif // ifndef __se3_parser_lua_tables_hpp____
