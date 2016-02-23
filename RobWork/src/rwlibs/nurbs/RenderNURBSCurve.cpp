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

#include "RenderNURBSCurve.hpp"

using namespace rw::math;
using namespace rw::graphics;
using namespace rwlibs::nurbs;

RenderNURBSCurve::RenderNURBSCurve(const GeometricNURBSCurve &curve):
	_curve(curve),
	_r(0),_g(0),_b(0)
{
}

RenderNURBSCurve::~RenderNURBSCurve() {
}

void RenderNURBSCurve::draw(const DrawableNode::RenderInfo& info,
    		DrawableNode::DrawType type,
    		double alpha) const {
	glPushMatrix();

	std::vector<double> knots = _curve.getKnots();
	GLfloat knotsGL[knots.size()];
	for (std::size_t i = 0; i < knots.size(); i++)
		knotsGL[i] = knots[i];

	unsigned int order = _curve.getOrder();

	std::vector<VectorND<3> > controlPoints = _curve.getPoints();
	GLfloat ctlpoints[controlPoints.size()][4];
	std::vector<double> weights = _curve.getWeights();
	for (std::size_t i = 0; i < controlPoints.size(); i++) {
			ctlpoints[i][0] = controlPoints[i][0]*weights[i];
			ctlpoints[i][1] = controlPoints[i][1]*weights[i];
			ctlpoints[i][2] = controlPoints[i][2]*weights[i];
			ctlpoints[i][3] = weights[i];
	}
	switch(type){
	case DrawableNode::SOLID:
	{
		Vector3D<float> color = getColor();
		GLUnurbsObj* nurbs = gluNewNurbsRenderer();
		glColor4f(color[0], color[1], color[2], (float)alpha);
		gluBeginCurve(nurbs);
		gluNurbsCurve(nurbs,
				knots.size(),knotsGL, //Knots
				4,//Stride
				&ctlpoints[0][0],
				order,//Order
				GL_MAP1_VERTEX_4);
		gluEndCurve(nurbs);
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
				glVertex3f(ctlpoints[i][0],
						ctlpoints[i][1], ctlpoints[i][2]);
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

void RenderNURBSCurve::setColor(float r, float g, float b) {
	_r = r;
	_g = g;
	_b = b;
}

Vector3D<float> RenderNURBSCurve::getColor() const {
	return Vector3D<float>(_r,_g,_b);
}
