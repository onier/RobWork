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

#include "Sphere.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include "PlainTriMesh.hpp"

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;

namespace {
	// this is a helper function for createsphere
	// if you just want to make a sphere, just call createsphere
	void spherehelper(const std::vector<Triangle<> >& triangles, std::vector<Triangle<> >& newtriangles, double radius)
	{
		newtriangles.clear();
		std::vector<Triangle<> >::const_iterator i = triangles.begin();
		while (i != triangles.end()) {
			const Triangle<>& t = *i++;
			Vector3D<> a = t[0], b = t[1], c = t[2];
			Vector3D<> v1(a[0]+b[0], a[1]+b[1], a[2]+b[2]);
			Vector3D<> v2(a[0]+c[0], a[1]+c[1], a[2]+c[2]);
			Vector3D<> v3(b[0]+c[0], b[1]+c[1], b[2]+c[2]);
			v1 = normalize(v1)*radius;
			v2 = normalize(v2)*radius;
			v3 = normalize(v3)*radius;

			newtriangles.push_back( Triangle<>(a, v1, v2) );
			newtriangles.push_back( Triangle<>(c, v2, v3) );
			newtriangles.push_back( Triangle<>(b, v3, v1) );
			newtriangles.push_back( Triangle<>(v1, v3, v2));
		}
	}
}

// levels specifies how many levels of detail we will have
// levels should be 0 or greater
// there will be 4^(levels+1) faces in there sphere
//std::vector<Triangle> createsphere(int levels)

TriMesh::Ptr Sphere::createMesh(int resolution) const{
	std::vector<Triangle<> > triangles, triangles_dst, *trimesh_dst, *trimesh_src;
  
  /* Let's find the helper granulation value of the sphere generator.
   * Old version of this code used value 6 for granulation, which was
   * supposed to match the detail achieved by other primitives
   * with resolution of 20.
   * It should be that the default value for resolution results in the same
   * level of detail as in the previous version. Should resolution parameter
   * increase the number of faces linearly?
   * 
   *    for level=20 faces=16384
   *      -> faces = resolution/20 * 16384
   *    if faces = 4^(granulation+1)
   *      -> granulation = log_4(faces)-1
   */
  double faces = resolution/20.0 * 16384;
  double granulation = log(faces) / log(4) - 1;

	// build a tetrahedron
	Vector3D<> v0(0.0, 0.0, 1.0);
	Vector3D<> v1(2.0*sqrt(2.0)/3.0, 0.0, -1.0/3.0);
	Vector3D<> v2(-sqrt(2.0)/3.0, sqrt(6.0)/3.0, -1.0/3.0);
	Vector3D<> v3(-sqrt(2.0)/3.0, -sqrt(6.0)/3.0, -1.0/3.0);

	triangles.push_back( Triangle<>(v0*_radius, v1*_radius, v2*_radius) );
	triangles.push_back( Triangle<>(v0*_radius, v2*_radius, v3*_radius) );
	triangles.push_back( Triangle<>(v0*_radius, v3*_radius, v1*_radius) );
	triangles.push_back( Triangle<>(v1*_radius, v3*_radius, v2*_radius) );

	trimesh_dst = &triangles_dst;
	trimesh_src = &triangles;
	for (int ctr = 0; ctr < granulation; ctr++){
	    spherehelper(*trimesh_src, *trimesh_dst, _radius);
		std::swap(trimesh_src, trimesh_dst);
		trimesh_dst->clear();
	}
	PlainTriMeshD *mesh = new PlainTriMeshD(*trimesh_src);
	return ownedPtr(mesh);
}

void Sphere::setParameters(const rw::math::Q& q) {
	if(q.size()!=1) {
		RW_THROW("Size of parameter list must equal 1!");
	}
	
	_radius = q(0);
}
