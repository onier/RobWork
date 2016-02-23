/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SaverAssimp.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Object.hpp>

#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;

SaverAssimp::SaverAssimp() {
}

SaverAssimp::~SaverAssimp() {
}

void SaverAssimp::save(Model3D::Ptr model, const std::string& filename) {
	std::string ext;
	if (filename.length() >= 3) {
		ext = StringUtil::toLower(filename.substr(filename.size()-3,std::string::npos));
		if (ext != "obj" && ext != "stl" && ext != "ply")
			ext = "collada";
	}
	/*
	Assimp::Exporter exporter;
	std::size_t formats = exporter.GetExportFormatCount();
	for (std::size_t i = 0; i < formats; i++) {
		const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
		std::cout << std::string(desc->id) << " " << std::string(desc->description) << " " << std::string(desc->fileExtension) << std::endl;
	}*/

	// Trick Assimp into creating a scene
	// (aiScene and aiMaterial constructors not available, so base scene is created from manual created OBJ file)
	std::string dummyFile;
	{
		std::stringstream stream;
		const std::vector<Model3D::Material> &rwmaterials = model->getMaterials();
		for (std::size_t i = 0; i < rwmaterials.size(); i++) {
			stream << "newmtl " << rwmaterials[i].name;
			stream << "Ka " << rwmaterials[i].ambient[0] << " " << rwmaterials[i].ambient[1] << " " << rwmaterials[i].ambient[2] << " " << rwmaterials[i].ambient[3];
			stream << "Kd " << rwmaterials[i].rgb[0] << " " << rwmaterials[i].rgb[1] << " " << rwmaterials[i].rgb[2] << " " << rwmaterials[i].rgb[3];
			stream << "Ks " << rwmaterials[i].specular[0] << " " << rwmaterials[i].specular[1] << " " << rwmaterials[i].specular[2] << " " << rwmaterials[i].specular[3];
			stream << "Ns " << rwmaterials[i].shininess;
			stream << "d " << rwmaterials[i].transparency;
			stream << "illum 1";
		}
		dummyFile = stream.str();
	}
	aiScene* scene = NULL;
	{
		Assimp::Importer importer;
		const aiScene* dummyScene = importer.ReadFileFromMemory( dummyFile.c_str(), dummyFile.size(), 0u, ".obj");
		if (dummyScene == NULL)
			RW_THROW("SaverAssimp: Could not create Assimp scene. " << importer.GetErrorString());
		aiCopyScene(dummyScene, &scene);
	}

	// Add meshes
	std::vector<Model3D::Object3D::Ptr> objects = model->getObjects();
	aiMesh** meshes = new aiMesh*[objects.size()];
	for (std::size_t i = 0; i < objects.size(); i++) {
		Model3D::Object3D::Ptr object = objects[i];
		aiMesh* mesh = new aiMesh();
		meshes[i] = mesh;
		mesh->mName = object->_name;
		aiVector3D* vertices = new aiVector3D[object->_vertices.size()];
		aiVector3D* normals = new aiVector3D[object->_vertices.size()];
		for (std::size_t j = 0; j < object->_vertices.size(); j++) {
			const Vector3D<float> &vertex = object->_vertices[j];
			const Vector3D<float> &normal = object->_normals[j];
			vertices[j].x = vertex[0];
			vertices[j].y = vertex[1];
			vertices[j].z = vertex[2];
			normals[j].x = normal[0];
			normals[j].y = normal[1];
			normals[j].z = normal[2];
		}
		mesh->mNumVertices = object->_vertices.size();
		mesh->mVertices = vertices;
		mesh->mNormals = normals;

		aiFace* faces = new aiFace[object->_faces.size()];
		for (std::size_t j = 0; j < object->_faces.size(); j++) {
			const IndexedTriangle<uint16_t> &face = object->_faces[j];
			faces[j].mNumIndices = 3;
			unsigned int* indices = new unsigned int[3];
			indices[0] = face.getVertexIdx(0);
			indices[1] = face.getVertexIdx(1);
			indices[2] = face.getVertexIdx(2);
			faces[j].mIndices = indices;
		}
		mesh->mNumFaces = object->_faces.size();
		mesh->mFaces = faces;
	}
	scene->mNumMeshes = objects.size();
	scene->mMeshes = meshes;

	// Root node
	scene->mRootNode->mName = model->getName();
	unsigned int* meshIDs = new unsigned int[objects.size()];
	for (std::size_t i = 0; i < objects.size(); i++) {
		meshIDs[i] = i;
	}
	scene->mRootNode->mMeshes = meshIDs;
	scene->mRootNode->mNumMeshes = objects.size();

	Assimp::Exporter exporter;
	aiReturn res = exporter.Export(scene, ext, filename, 0u);

	// Clean up
	delete[] scene->mMeshes;
	scene->mMeshes = NULL;
	scene->mNumMeshes = 0;
	delete[] scene->mMaterials;
	scene->mMaterials = NULL;
	scene->mNumMaterials = 0;

	if (res != aiReturn_SUCCESS)
		RW_THROW("SaverAssimp not able to save: " << std::string(exporter.GetErrorString()));
}
