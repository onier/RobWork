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

#include "RenderNURBSSurface.hpp"
#include <rw/math/Vector3D.hpp>

using namespace rw::math;
using namespace rw::graphics;
using namespace rwlibs::nurbs;

RenderNURBSSurface::RenderNURBSSurface(const GeometricNURBSSurface &surface):
	_surface(surface),
	_r(0),_g(0),_b(0)
{
}

RenderNURBSSurface::~RenderNURBSSurface() {
}

void RenderNURBSSurface::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const {
	glPushMatrix();

	glColor4f(_r, _g, _b, (float)alpha);

	std::vector<double> knotsU = _surface.getKnotsU();
	std::vector<double> knotsV = _surface.getKnotsV();
	GLfloat knotsUGL[knotsU.size()];
	GLfloat knotsVGL[knotsV.size()];
	for (std::size_t i = 0; i < knotsU.size(); i++)
		knotsUGL[i] = knotsU[i];
	for (std::size_t i = 0; i < knotsV.size(); i++)
		knotsVGL[i] = knotsV[i];

	std::size_t orderU = _surface.getOrderU();
	std::size_t orderV = _surface.getOrderV();

	GeometricNURBSSurface::PointMapHomogeneous controlPoints = _surface.getPointsHomogeneous();
	GLfloat ctlpoints[controlPoints.size()][controlPoints[0].size()][4];
	for (std::size_t i = 0; i < controlPoints.size(); i++) {
		for (std::size_t j = 0; j < controlPoints[i].size(); j++) {
			ctlpoints[i][j][0] = controlPoints[i][j][0];
			ctlpoints[i][j][1] = controlPoints[i][j][1];
			ctlpoints[i][j][2] = controlPoints[i][j][2];
			ctlpoints[i][j][3] = controlPoints[i][j][3];
		}
	}
	switch(type){
	case DrawableNode::SOLID:
	{
		GLUnurbsObj* nurbs = gluNewNurbsRenderer();
		glColor4f(_r, _g, _b, (float)alpha);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs,
				knotsU.size(),knotsUGL,knotsV.size(),knotsVGL, //Knots u,v
				controlPoints[0].size()*4,4,//Stride u,v
				&ctlpoints[0][0][0],
				orderU,orderV,//Order u,v
				GL_MAP2_VERTEX_4);
		// To add light: try to call again with GL_MAP2_NORMAL - or maybe better call it before!
		gluEndSurface(nurbs);
		gluDeleteNurbsRenderer(nurbs);
	}
	break;
	case DrawableNode::OUTLINE: // Draw nice frame
	{
		glPointSize(5.0);
		glDisable(GL_LIGHTING);
		glColor3f(1.0, 1.0, 0.0);
		glBegin(GL_POINTS);
		for (unsigned int i = 0; i <= 4; i++) {
			for (unsigned int j = 0; j <= 4; j++) {
				glVertex3f(ctlpoints[i][j][0],
						ctlpoints[i][j][1], ctlpoints[i][j][2]);
			}
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}
	break;
	case DrawableNode::WIRE: // Draw nice frame

		break;
	}

	glPopMatrix();
}

void RenderNURBSSurface::setColor(float r, float g, float b) {
    _r = r;
    _g = g;
    _b = b;
}

Vector3D<float> RenderNURBSSurface::getColor() const {
	return Vector3D<float>(_r,_g,_b);
}
