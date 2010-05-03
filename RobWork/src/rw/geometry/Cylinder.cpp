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

#include "Cylinder.hpp"

#include <rw/math/Constants.hpp>
#include <rw/common/macros.hpp>
#include "PlainTriMesh.hpp"


using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

namespace {
	std::string toString(float dx, float dy, int dz)
    {
        if (dz < 0) RW_THROW("Negative number of faces " << dz);

		std::stringstream str;
		str << "Cylinder " << dx << " " << dy << " " << dz;
		return str.str();
 	}
}

Cylinder::Cylinder():
	_radius(1),_height(1)
{
}


Cylinder::Cylinder(float radius, float height):
	_radius(radius),_height(height)
{
}

Cylinder::~Cylinder(){}


TriMeshPtr Cylinder::createMesh(int resolution){
	int level = resolution;
	if(resolution<0)
		level = 16; // default

	PlainTriMeshN0d *mesh = new PlainTriMeshN0d(4*level);

	float z = _height/2.0f;

	for (int i = 0; i < level; i++) {
		//Construct Triangles for curved surface
		float x1 = (float)(_radius * cos(i * 2 * Pi/level));
		float y1 = (float)(_radius * sin(i * 2 * Pi/level));

		float x2 = (float)(_radius * cos((i+1) * 2 * Pi/level));
		float y2 = (float)(_radius * sin((i+1) * 2 * Pi/level));

		Vector3D<> p1(x1, y1, z);
		Vector3D<> p2(x1, y1, -z);
		Vector3D<> p3(x2, y2, z);
		Vector3D<> p4(x2, y2, -z);

		(*mesh)[i*4+0] = TriangleN0<>(p1,p2,p3);
		(*mesh)[i*4+1] = TriangleN0<>(p2,p4,p3);
		//_faces.push_back(Face<float>(p1, p2, p3));
		//_faces.push_back(Face<float>(p2, p4, p3));

		//Construct triangles for the end-plates
		Vector3D<> p5(0, 0,  z);
		Vector3D<> p6(0, 0, -z);
		(*mesh)[i*4+2] = TriangleN0<>(p1,p3,p5);
		(*mesh)[i*4+3] = TriangleN0<>(p6,p4,p2);
		/*_faces.push_back(
			Face<float>(
				Vector3D<float>(x1, y1, z),
				Vector3D<float>(x2, y2, z),
				Vector3D<float>(0,0,z)));

		_faces.push_back(
			Face<float>(
				Vector3D<float>(0,0,-z),
				Vector3D<float>(x2, y2, -z),
				Vector3D<float>(x1, y1, -z)));
				*/
	}
	return ownedPtr(mesh);
}

const rw::math::Q& Cylinder::getParameters(){
	Q q(2);
	q(0) = _height;
	q(1) = _radius;
	return q;
}
