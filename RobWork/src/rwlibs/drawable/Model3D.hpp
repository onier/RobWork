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
        std::string _name;	// The material's name
        RWGLTexture tex;	// The texture (this is the only outside reference in this class)
        bool textured;	// whether or not it is textured
        unsigned char color[4]; // color info
    };

	struct Object3D {
		std::string _name;
		rw::geometry::IndexedTriMesh<float> _mesh;
		std::vector<float> _texCoords;
		bool _textured;
		std::vector<MaterialFaces> _matFaces;
		rw::math::Transform3D<> _transform;
	};

public:
	void addObject(Object3D* obj);
	void addMaterial(Material* mat);
	void removeObject(const std::string& name);
	const std::vector<Material>& getMaterials();
	const std::vector<Object3D>& getObjects();

private:
	rw::math::Transform3D<> _transform;
    std::vector<Material> _materials; // The array of materials
    std::vector<Object3D> _objects; // The array of objects in the model

    int totalVerts;			// Total number of vertices in the model
    int totalFaces;			// Total number of faces in the model
};

#endif /* MODEL3D_HPP_ */
