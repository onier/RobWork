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

#include "BRep.hpp"
#include "ParametricPlaneFace.hpp"
#include "ParametricRoundedEdge.hpp"

#include <rw/geometry/Line.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::nurbs;

BRep::BRep()
{
}

BRep::BRep(Box::Ptr box, double roundingRadius)
{
	Q par = box->getParameters();
	double x = par[0];
	double y = par[1];
	double z = par[2];

	Vector3D<> xDir = Vector3D<>::x();
	Vector3D<> yDir = Vector3D<>::y();
	Vector3D<> zDir = Vector3D<>::z();

	Transform3D<> locationBottom(	Vector3D<>( 0,	 0,		-z/2),	Rotation3D<>( xDir,-yDir,-zDir));
	Transform3D<> locationTop(		Vector3D<>( 0,	 0,		 z/2),	Rotation3D<>( xDir, yDir, zDir));
	Transform3D<> locationLeft(		Vector3D<>( 0,	-y/2,	 0),	Rotation3D<>( xDir, zDir,-yDir));
	Transform3D<> locationRight(	Vector3D<>( 0,	 y/2,	 0),	Rotation3D<>( xDir,-zDir, yDir));
	Transform3D<> locationFront(	Vector3D<>(-x/2, 0,		 0),	Rotation3D<>( yDir, zDir,-xDir));
	Transform3D<> locationBack(		Vector3D<>( x/2, 0,		 0),	Rotation3D<>( yDir,-zDir, xDir));

	addFace( ownedPtr( new ParametricPlaneFace(locationTop,		x-2*roundingRadius, y-2*roundingRadius) ) );
	addFace( ownedPtr( new ParametricPlaneFace(locationBottom,	x-2*roundingRadius, y-2*roundingRadius) ) );
	addFace( ownedPtr( new ParametricPlaneFace(locationLeft,	x-2*roundingRadius, z-2*roundingRadius) ) );
	addFace( ownedPtr( new ParametricPlaneFace(locationRight,	x-2*roundingRadius, z-2*roundingRadius) ) );
	addFace( ownedPtr( new ParametricPlaneFace(locationFront,	y-2*roundingRadius, z-2*roundingRadius) ) );
	addFace( ownedPtr( new ParametricPlaneFace(locationBack,	y-2*roundingRadius, z-2*roundingRadius) ) );

	Line* line;
	// Edges about x-direction
	line = new Line(Vector3D<>(-x/2+roundingRadius, -y/2+roundingRadius, z/2-roundingRadius), Vector3D<>(x/2-roundingRadius, -y/2+roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, zDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius,  y/2-roundingRadius, z/2-roundingRadius), Vector3D<>(x/2-roundingRadius,  y/2-roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, yDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius, -y/2+roundingRadius, -z/2+roundingRadius), Vector3D<>(x/2-roundingRadius, -y/2+roundingRadius, -z/2+roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, -yDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius,  y/2-roundingRadius, -z/2+roundingRadius), Vector3D<>(x/2-roundingRadius,  y/2-roundingRadius, -z/2+roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, -zDir, roundingRadius) ) );

	// Edges about y-direction
	line = new Line(Vector3D<>( x/2-roundingRadius, -y/2+roundingRadius, z/2-roundingRadius), Vector3D<>(x/2-roundingRadius, y/2-roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, zDir, roundingRadius) ) );
	line = new Line(Vector3D<>( x/2-roundingRadius, -y/2+roundingRadius, -z/2+roundingRadius), Vector3D<>(x/2-roundingRadius,  y/2-roundingRadius, -z/2+roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, xDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius, -y/2+roundingRadius,  z/2-roundingRadius), Vector3D<>(-x/2+roundingRadius, y/2-roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, -xDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius, -y/2+roundingRadius, -z/2+roundingRadius), Vector3D<>(-x/2+roundingRadius,  y/2-roundingRadius, -z/2+roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, -zDir, roundingRadius) ) );

	// Edges about z-direction
	line = new Line(Vector3D<>( x/2-roundingRadius, -y/2+roundingRadius, -z/2+roundingRadius), Vector3D<>(x/2-roundingRadius, -y/2+roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, -yDir, roundingRadius) ) );
	line = new Line(Vector3D<>( x/2-roundingRadius, y/2-roundingRadius, -z/2+roundingRadius), Vector3D<>(x/2-roundingRadius,  y/2-roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, xDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius, y/2-roundingRadius,  -z/2+roundingRadius), Vector3D<>(-x/2+roundingRadius, y/2-roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, yDir, roundingRadius) ) );
	line = new Line(Vector3D<>(-x/2+roundingRadius, -y/2+roundingRadius, -z/2+roundingRadius), Vector3D<>(-x/2+roundingRadius,  -y/2+roundingRadius, z/2-roundingRadius));
	addFace( ownedPtr( new ParametricRoundedEdge(line, -xDir, roundingRadius) ) );
}

BRep::~BRep() {
}

GeometryData::GeometryType BRep::getType() const {
	return GeometryData::UserType;
}

TriMesh::Ptr BRep::getTriMesh(bool forceCopy) {
	return NULL;
}

bool BRep::isConvex() {
	return false;
}

void BRep::addFace(ParametricSurface::Ptr surface)
{
	Face face;
	face.surface = surface;
	_faces.push_back(face);
}

std::list<BRep::Face> BRep::getFaces() {
	return _faces;
}

std::list<BRep::Edge> BRep::getEdges() {
	return _edges;
}

std::list<BRep::Vertex> BRep::getVertices() {
	return _vertices;
}

std::list<ParametricSurface::Ptr> BRep::getSurfaces()
{
	std::list<ParametricSurface::Ptr> newList;
	for (std::list<Face>::iterator it = _faces.begin(); it != _faces.end(); it++) {
		newList.push_back((*it).surface);
	}
	return newList;
}

std::list<ParametricCurve::Ptr> BRep::getCurves() {
	std::list<ParametricCurve::Ptr> newList;
	for (std::list<Edge>::iterator it = _edges.begin(); it != _edges.end(); it++) {
		newList.push_back((*it).curve);
	}
	return newList;
}

std::list<Vector3D<> > BRep::getPoints() {
	std::list<Vector3D<> > newList;
	for (std::list<Vertex>::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		newList.push_back((*it).point);
	}
	return newList;
}
