/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

/*
 * Model3D.hpp
 *
 *  Created on: 02-07-2009
 *      Author: jimali
 */

#ifndef MODEL3D_HPP_
#define MODEL3D_HPP_

#include "RWGLTexture.hpp"

#include <vector>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>

namespace rwlibs {
namespace drawable {

/**
 * @brief a 3d model that has geometry but also material, color and texture information.
 * the model can be composed of multiple objects
 */
class Model3D {
public:
	Model3D();
	virtual ~Model3D();

public:
    // Holds the material info
    // TODO: add color support for non textured polys
    struct Material {
    	Material():name(""), textured(false), simplergb(true){};
        Material(const std::string& nam, float r, float g, float b, float a=1.0):
            name(nam), textured(false), simplergb(true)
        {
            rgb[0] = r;
            rgb[1] = g;
            rgb[2] = b;
            rgb[3] = a;
        }
        std::string name;	// The material's name
        short int texId;	// The texture (this is the only outside reference in this class)
        bool textured;	// whether or not it is textured
        bool simplergb;

        /** Red, Green, Blue color components */
        float rgb[4];
        /** Ambient color as RGB */
        float ambient[4];
        /** Emissive color as RGB */
        float emissive[4];
        /** Specular color as RGB */
        float specular[4];

        /** The shininess \f$\in [0,128] \f$ */
        float shininess;
        /** Transparency \f$ in [0, 1]\f$ */
        float transparency;

    };

    // I sort the mesh by material so that I won't have to switch textures a great deal
    struct MaterialFaces {
        // Index to our vertex array of all the faces that use this material
        std::vector<rw::geometry::IndexedTriangle<> > _subFaces;
        //int numSubFaces;            // The number of faces
        int _matIndex;               // An index to our materials
    };

    struct MaterialPolys {
        // Index to our vertex array of all the faces that use this material
        std::vector<rw::geometry::IndexedPolygonN<> > _subPolys;
        //int numSubFaces;            // The number of faces
        int _matIndex;               // An index to our materials
    };

	struct Object3D {
	    Object3D(const std::string& name):
	        _name(name),
	        parentObj(-1),
	        _texture(-1),
	        _texOffset(0,0),
	        _texRepeat(0,0){};

	    bool hasTexture() const{ return _texture>=0;};

		std::string _name;
		//rw::geometry::IndexedTriMeshN0<float> *_mesh;

		std::vector<rw::math::Vector3D<float> > _vertices;
        std::vector<rw::math::Vector3D<float> > _normals;
        std::vector<rw::math::Vector2D<float> > _texCoords;

        // the normal is implicit and defined as the a vertex normal with
        // index as its vertex
        std::vector<rw::geometry::IndexedTriangle<> > _faces;
        // the normal is explicit and only defined for the Face
        //std::vector<rw::geometry::IndexedTriangleN1<float> > _faces1;
        // the normal is explicit and defined for each vertex
        //std::vector<rw::geometry::IndexedTriangleN3<float> > _faces3;
		// todo: perhaps a list of polytopes also
		std::vector<rw::geometry::IndexedPolygonN<> > _polys;

		int _texture;
		std::vector<MaterialFaces*> _matFaces;
		std::vector<MaterialPolys*> _matPolys;
		rw::math::Transform3D<float> _transform;
		int parentObj;
		std::vector<Object3D*> _kids;
		rw::math::Vector2D<float> _texOffset, _texRepeat;

	};

public:
	int addObject(Object3D* obj);
	int addMaterial(const Material& mat);
	void removeObject(const std::string& name);

	std::vector<Material>& getMaterials(){ return _materials; };
	std::vector<Object3D*>& getObjects(){ return _objects; };

	const rw::math::Transform3D<>& getTransform(){ return _transform;};
	void setTransform(const rw::math::Transform3D<>& t3d){ _transform = t3d;};
//private:
	rw::math::Transform3D<> _transform;
    std::vector<Material> _materials; // The array of materials
    std::vector<Object3D*> _objects; // The array of objects in the model
    std::vector<RWGLTexture*> _textures;

    int totalVerts;			// Total number of vertices in the model
    int totalFaces;			// Total number of faces in the model
};


typedef rw::common::Ptr<Model3D> Model3DPtr;

}
}
#endif /* MODEL3D_HPP_ */
