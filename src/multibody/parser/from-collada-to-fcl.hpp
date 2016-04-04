//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio and is mainly inspired
// by software hpp-model-urdf
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

#ifndef __se3_collada_to_fcl_hpp__
#define __se3_collada_to_fcl_hpp__

#include <limits>

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <assimp/DefaultLogger.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>

#include "pinocchio/tools/file-explorer.hpp"
#include <boost/filesystem.hpp>

#include <exception>

namespace se3
{
  typedef fcl::BVHModel< fcl::OBBRSS > PolyhedronType;
  typedef boost::shared_ptr <PolyhedronType> Polyhedron_ptr;
  
  struct TriangleAndVertices
  {
    void clear()
    {
      vertices_.clear ();
      triangles_.clear ();
    }
    std::vector <fcl::Vec3f> vertices_;
    std::vector <fcl::Triangle> triangles_;
  };
  
  
  /**
   * @brief      Recursive procedure for building a mesh
   *
   * @param[in]  scale           Scale to apply when reading the ressource
   * @param[in]  scene           Pointer to the assimp scene
   * @param[in]  node            Current node of the scene
   * @param      subMeshIndexes  Submesh triangles indexes interval
   * @param[in]  mesh            The mesh that must be built
   * @param      tv              Triangles and Vertices of the mesh submodels
   */
  inline void buildMesh (const ::urdf::Vector3 & scale,
                         const aiScene* scene,
                         const aiNode* node,
                         std::vector<unsigned> & subMeshIndexes,
                         const Polyhedron_ptr & mesh,
                         TriangleAndVertices & tv)
  {
    if (!node) return;
    
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while (pnode)
    {
      // Don't convert to y-up orientation, which is what the root node in
      // Assimp does
      if (pnode->mParent != NULL)
      {
        transform = pnode->mTransformation * transform;
      }
      pnode = pnode->mParent;
    }
    
    for (uint32_t i = 0; i < node->mNumMeshes; i++)
    {
      aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
      
      unsigned oldNbPoints = (unsigned) mesh->num_vertices;
      unsigned oldNbTriangles = (unsigned) mesh->num_tris;
      
      // Add the vertices
      for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
      {
        aiVector3D p = input_mesh->mVertices[j];
        p *= transform;
        tv.vertices_.push_back (fcl::Vec3f (p.x * scale.x,
                                            p.y * scale.y,
                                            p.z * scale.z));
      }
      
      // add the indices
      for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
      {
        aiFace& face = input_mesh->mFaces[j];
        // FIXME: can add only triangular faces.
        tv.triangles_.push_back (fcl::Triangle( oldNbPoints + face.mIndices[0],
                                               oldNbPoints + face.mIndices[1],
                                               oldNbPoints + face.mIndices[2]));
      }
      
      // Save submesh triangles indexes interval.
      if (subMeshIndexes.size () == 0)
      {
        subMeshIndexes.push_back (0);
      }
      
      subMeshIndexes.push_back (oldNbTriangles + input_mesh->mNumFaces);
    }
    
    for (uint32_t i=0; i < node->mNumChildren; ++i)
    {
      buildMesh(scale, scene, node->mChildren[i], subMeshIndexes, mesh, tv);
    }
  }
  
  
  
  /**
   * @brief      Convert an assimp scene to a mesh
   *
   * @param[in]  name   File (ressource) transformed into an assimp scene in loa
   * @param[in]  scale  Scale to apply when reading the ressource
   * @param[in]  scene  Pointer to the assimp scene
   * @param[out] mesh  The mesh that must be built
   */
  inline void meshFromAssimpScene (const std::string & name,
                                   const ::urdf::Vector3 & scale,
                                   const aiScene* scene,
                                   const Polyhedron_ptr & mesh) throw (std::invalid_argument)
  {
    TriangleAndVertices tv;
    
    if (!scene->HasMeshes())
      throw std::invalid_argument (std::string ("No meshes found in file ")+name);
    
    std::vector<unsigned> subMeshIndexes;
    int res = mesh->beginModel ();
    
    if (res != fcl::BVH_OK)
    {
      std::ostringstream error;
      error << "fcl BVHReturnCode = " << res;
      throw std::runtime_error (error.str ());
    }
      
    tv.clear();
      
    buildMesh (scale, scene, scene->mRootNode, subMeshIndexes, mesh, tv);
    mesh->addSubModel (tv.vertices_, tv.triangles_);
      
    mesh->endModel ();
  }
      
      
      
  /**
   * @brief      Read a mesh file and convert it to a polyhedral mesh
   *
   * @param[in]  resource_path  Path to the ressource mesh file to be read
   * @param[in]  scale          Scale to apply when reading the ressource
   * @param[out] polyhedron     The resulted polyhedron
   */
  inline void loadPolyhedronFromResource (const std::string & resource_path,
                                          const ::urdf::Vector3 & scale,
                                          const Polyhedron_ptr & polyhedron) throw (std::invalid_argument)
  {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(resource_path.c_str(), aiProcess_SortByPType| aiProcess_GenNormals|
                                             aiProcess_Triangulate|aiProcess_GenUVCoords|
                                             aiProcess_FlipUVs);
    if (!scene)
    {
      const std::string exception_message (std::string ("Could not load resource ") + resource_path + std::string("\n") +
                                           importer.GetErrorString () + std::string("\n") +
                                           "Hint: the mesh directory may be wrong.");
      throw std::invalid_argument(exception_message);
    }
    
    meshFromAssimpScene (resource_path, scale, scene, polyhedron);
  }
  
  
  /**
   * @brief      Transform a package://.. mesh path to an absolute path, searching for a valid file 
   *             in a list of package directories
   *
   * @param[in]  urdf_mesh_path  The path given in the urdf file
   * @param[in]  package_dirs    A list of packages directories where to search for meshes
   *
   * @return     The absolute path to the mesh file
   */
   inline std::string convertURDFMeshPathToAbsolutePath(const std::string & urdf_mesh_path,
                                                         const std::vector<std::string> & package_dirs)
   {
    // if exists p1/mesh, absolutePath = p1/mesh,
    // else if exists p2/mesh, absolutePath = p2/mesh
    // else return an empty string that will provoke an error in loadPolyhedronFromResource()
    namespace bf = boost::filesystem;

    std::string absolutePath;
    // concatenate package_path with mesh filename
    for (int i = 0; i < package_dirs.size(); ++i)
    {
      if ( bf::exists( bf::path(package_dirs[i] +  urdf_mesh_path.substr(9, urdf_mesh_path.size()))))
      {
          absolutePath = std::string( package_dirs[i] + urdf_mesh_path.substr(9, urdf_mesh_path.size())
                                       );
        break;
      }
    }
    return absolutePath;
   }

} // namespace se3


#endif // __se3_collada_to_fcl_hpp__
