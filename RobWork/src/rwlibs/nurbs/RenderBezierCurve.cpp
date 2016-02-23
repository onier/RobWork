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

#include "RenderBezierCurve.hpp"

using namespace rw::math;
using namespace rw::graphics;
using namespace rwlibs::nurbs;

RenderBezierCurve::RenderBezierCurve(const GeometricBezierCurve &curve):
	_curve(curve)
{
	GLint* ptr = new GLint();
	glGetIntegerv(GL_MAX_EVAL_ORDER, ptr);
	if (*ptr < (int)curve.getOrder()) {
		std::stringstream str;
		str << "RenderBezierCurve can show a BezierCurve of maximum order ";
		str << *ptr;
		str << ". (GL_MAX_EVAL_ORDER)";
		RW_THROW(str.str());
	}
	delete ptr;
}

RenderBezierCurve::~RenderBezierCurve() {
}

void RenderBezierCurve::draw(const DrawableNode::RenderInfo& info,
    		DrawableNode::DrawType type,
    		double alpha) const {
	glPushMatrix();

	std::vector<VectorND<4> > points = _curve.getPointsHomogeneous();
	GLfloat ctlpoints[points.size()][4];
	for (std::size_t i = 0; i < points.size(); i++) {
			ctlpoints[i][0] = points[i][0];
			ctlpoints[i][1] = points[i][1];
			ctlpoints[i][2] = points[i][2];
			ctlpoints[i][3] = points[i][3];
	}

	GLfloat knotsGL[2*_curve.getOrder()];
	for (std::size_t i = 0; i < _curve.getOrder(); i++)
		knotsGL[i] = 0;
	for (std::size_t i = 0; i < _curve.getOrder(); i++)
		knotsGL[_curve.getOrder()+i] = 1;

	switch(type){
	case DrawableNode::SOLID:
	{
		Vector3D<float> color = getColor();
		glColor4f(color[0], color[1], color[2], (float)alpha);

		GLUnurbsObj* nurbs = gluNewNurbsRenderer();
		glColor4f(color[0], color[1], color[2], (float)alpha);
		gluBeginCurve(nurbs);
		gluNurbsCurve(nurbs,
				2*_curve.getOrder(),knotsGL, //Knots
				4,//Stride
				&ctlpoints[0][0],
				_curve.getOrder(),//Order
				GL_MAP1_VERTEX_4);
		gluEndCurve(nurbs);
		gluDeleteNurbsRenderer(nurbs);
	}
	break;
	case DrawableNode::OUTLINE: // Draw nice frame
	{
		glColor4f(0.,0.,0.,1.);
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < points.size(); i++)
			glVertex3f (ctlpoints[i][0]/ctlpoints[i][3],ctlpoints[i][1]/ctlpoints[i][3],ctlpoints[i][2]/ctlpoints[i][3]);
		glEnd();
		glColor4f(1.,1.,0.,1.);
		glPointSize(5.0);
		glBegin (GL_POINTS);
		for (unsigned int i = 0; i < points.size(); i++)
			glVertex3f (ctlpoints[i][0]/ctlpoints[i][3],ctlpoints[i][1]/ctlpoints[i][3],ctlpoints[i][2]/ctlpoints[i][3]);
		glEnd();
	}
	break;
	case DrawableNode::WIRE: // Draw nice frame

		break;
	}

	glPopMatrix();
}

void RenderBezierCurve::setColor(float r, float g, float b) {
	_r = r;
	_g = g;
	_b = b;
}

Vector3D<float> RenderBezierCurve::getColor() const {
	return Vector3D<float>(_r,_g,_b);
}
