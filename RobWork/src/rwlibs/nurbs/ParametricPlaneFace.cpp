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

#include <rw/geometry/PlainTriMesh.hpp>

#include "ParametricPlaneFace.hpp"

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::nurbs;

ParametricPlaneFace::ParametricPlaneFace(Transform3D<> location, double sizeX, double sizeY):
	_location(location),
	_sizeX(sizeX),
	_sizeY(sizeY)
{
}

TriMesh::Ptr ParametricPlaneFace::getTriMesh(bool forceCopy)
{
	rw::geometry::PlainTriMesh<>::Ptr mesh = ownedPtr( new rw::geometry::PlainTriMesh<>(2) );
	Vector3D<> x = _location.R().getCol(0);
	Vector3D<> y = _location.R().getCol(1);

	(*mesh)[0][0] = _location.P()+x*(-0.5)*_sizeX+y*(-0.5)*_sizeY;
	(*mesh)[0][1] = _location.P()+x*(-0.5)*_sizeX+y*( 0.5)*_sizeY;
	(*mesh)[0][2] = _location.P()+x*( 0.5)*_sizeX+y*(-0.5)*_sizeY;

	(*mesh)[1][0] = _location.P()+x*( 0.5)*_sizeX+y*( 0.5)*_sizeY;
	(*mesh)[1][1] = _location.P()+x*(-0.5)*_sizeX+y*( 0.5)*_sizeY;
	(*mesh)[1][2] = _location.P()+x*( 0.5)*_sizeX+y*(-0.5)*_sizeY;

	return mesh;
}

bool ParametricPlaneFace::isConvex(){
	return true;
}

Vector3D<> ParametricPlaneFace::evaluate(double u, double v)
{
	return _location.P()+_location.R().getCol(0)*(-0.5+u)*_sizeX+_location.R().getCol(1)*(-0.5+u)*_sizeY;
}

Vector3D<> ParametricPlaneFace::normal(double u, double v) {
	return _location.R().getCol(2);
}


std::vector<Vector3D<> > ParametricPlaneFace::getCorners() {
	std::vector<Vector3D<> > vec;
	Vector3D<> x = _location.R().getCol(0);
	Vector3D<> y = _location.R().getCol(1);
	vec.push_back(_location.P()+x*(-0.5)*_sizeX+y*(-0.5)*_sizeY);
	vec.push_back(_location.P()+x*(-0.5)*_sizeX+y*( 0.5)*_sizeY);
	vec.push_back(_location.P()+x*( 0.5)*_sizeX+y*( 0.5)*_sizeY);
	vec.push_back(_location.P()+x*( 0.5)*_sizeX+y*(-0.5)*_sizeY);
	return vec;
}
