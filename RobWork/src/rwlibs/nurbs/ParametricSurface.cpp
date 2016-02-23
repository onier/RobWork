/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ParametricSurface.hpp"

#include <rw/geometry/PlainTriMesh.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::nurbs;

GeometryData::GeometryType ParametricSurface::getType() const {
	return GeometryData::UserType;
}

Ptr<TriMesh> ParametricSurface::getTriMesh(bool forceCopy) {
	return createTriMesh(10,10);
}

Ptr<TriMesh> ParametricSurface::createTriMesh(unsigned int resU, unsigned int resV) {
	double dU = 1./resU;
	double dV = 1./resV;
	PlainTriMeshN1D *mesh = new PlainTriMeshN1D(2*resU*resV);

	for (unsigned int i = 1; i <= resU; i++) {
		for (unsigned int j = 1; j <= resV; j++) {
			Vector3D<> val1 = evaluate(dU*(i-1),dV*(j-1));
			Vector3D<> val2 = evaluate(dU*(i-1),dV*(j  ));
			Vector3D<> val3 = evaluate(dU*(i  ),dV*(j-1));
			Vector3D<> val4 = evaluate(dU*(i  ),dV*(j  ));
			Vector3D<> normal1 = normal(2./3.*dU*(i-1)+1./3.*dU*(i  ),2./3.*dV*(i-1)+1./3.*dV*(i  ));
			Vector3D<> normal2 = normal(1./3.*dU*(i-1)+2./3.*dU*(i  ),1./3.*dV*(i-1)+2./3.*dV*(i  ));
			(*mesh)[2*((i-1)*resV+j-1)  ] = TriangleN1<double>(val1,val2,val3,normal1);
			(*mesh)[2*((i-1)*resV+j-1)+1] = TriangleN1<double>(val2,val4,val3,normal2);
		}
	}

	return ownedPtr( mesh );
}
