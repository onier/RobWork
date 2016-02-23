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

#include "RenderBezierSurface.hpp"

using namespace rw::math;
using namespace rw::graphics;
using namespace rwlibs::nurbs;

RenderBezierSurface::RenderBezierSurface(const GeometricBezierSurface &surface):
	_surface(surface),
	_r(0),_g(0),_b(0)
{
	GLint* ptr = new GLint();
	glGetIntegerv(GL_MAX_EVAL_ORDER, ptr);
	if (*ptr < (int)surface.getOrder().first || *ptr < (int)surface.getOrder().second) {
		std::stringstream str;
		str << "RenderBezierSurface can show a Bezier surface of maximum order ";
		str << *ptr;
		str << ". (GL_MAX_EVAL_ORDER)";
		RW_THROW(str.str());
	}
	delete ptr;

	for (std::size_t i = 0; i < _surface.getOrder().first; i++) {
		double u = ((double)i)/((double)_surface.getOrder().first-1);
		RenderBezierCurve* renderCurve = new RenderBezierCurve(_surface.getCurveU(u));
		renderCurve->setColor(_r,_g,_b);
		_curvesU.push_back(renderCurve);
	}
	for (std::size_t j = 0; j < _surface.getOrder().second; j++) {
		double v = ((double)j)/((double)_surface.getOrder().second-1);
		RenderBezierCurve* renderCurve = new RenderBezierCurve(_surface.getCurveV(v));
		renderCurve->setColor(_r,_g,_b);
		_curvesV.push_back(renderCurve);
	}
}

RenderBezierSurface::~RenderBezierSurface() {
	for (std::size_t i = 0; i < _curvesU.size(); i++)
		delete _curvesU[i];
	_curvesU.clear();
	for (std::size_t i = 0; i < _curvesV.size(); i++)
		delete _curvesV[i];
	_curvesV.clear();
}

void RenderBezierSurface::draw(const DrawableNode::RenderInfo& info,
    		DrawableNode::DrawType type,
    		double alpha) const {
	std::vector<std::vector<VectorND<4> > > points = _surface.getPointsHomogeneous();
	GLfloat ctlpoints[points.size()][points[0].size()][4];
	for (std::size_t i = 0; i < points.size(); i++) {
		for (std::size_t j = 0; j < points[i].size(); j++) {
			ctlpoints[i][j][0] = points[i][j][0];
			ctlpoints[i][j][1] = points[i][j][1];
			ctlpoints[i][j][2] = points[i][j][2];
			ctlpoints[i][j][3] = points[i][j][3];
		}
	}

	GLfloat knotsUGL[2*_surface.getOrder().first];
	GLfloat knotsVGL[2*_surface.getOrder().second];
	for (std::size_t i = 0; i < _surface.getOrder().first; i++)
		knotsUGL[i] = 0;
	for (std::size_t i = 0; i < _surface.getOrder().first; i++)
		knotsUGL[_surface.getOrder().first+i] = 1;
	for (std::size_t i = 0; i < _surface.getOrder().second; i++)
		knotsVGL[i] = 0;
	for (std::size_t i = 0; i < _surface.getOrder().second; i++)
		knotsVGL[_surface.getOrder().second+i] = 1;

	switch(type){
	case DrawableNode::SOLID:
	{
		glPushMatrix();
		Vector3D<float> color = getColor();
		glColor4f(color[0], color[1], color[2], (float)alpha);

		GLUnurbsObj* nurbs = gluNewNurbsRenderer();
		glColor4f(color[0], color[1], color[2], (float)alpha);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs,
				2*_surface.getOrder().first,knotsUGL,2*_surface.getOrder().second,knotsVGL, //Knots
				points[0].size()*4,4,//Stride u,v
				&ctlpoints[0][0][0],
				_surface.getOrder().first,_surface.getOrder().second,//Order
				GL_MAP2_VERTEX_4);
		gluEndSurface(nurbs);
		gluDeleteNurbsRenderer(nurbs);
		glPopMatrix();
	}
	break;
	case DrawableNode::OUTLINE: // Draw nice frame
	{
		glPushMatrix();
		glColor4f(0.,0.,0.,1.);
		for (unsigned int i = 0; i < points.size(); i++) {
			glBegin(GL_LINE_STRIP);
			for (unsigned int j = 0; j < points[i].size(); j++)
				glVertex3f (ctlpoints[i][j][0]/ctlpoints[i][j][3],ctlpoints[i][j][1]/ctlpoints[i][j][3],ctlpoints[i][j][2]/ctlpoints[i][j][3]);
			glEnd();
		}
		if (points.size() > 0) {
			for (unsigned int j = 0; j < points[0].size(); j++) {
				glBegin(GL_LINE_STRIP);
				for (unsigned int i = 0; i < points.size(); i++)
					glVertex3f (ctlpoints[i][j][0]/ctlpoints[i][j][3],ctlpoints[i][j][1]/ctlpoints[i][j][3],ctlpoints[i][j][2]/ctlpoints[i][j][3]);
				glEnd();
			}
		}
		glColor4f(1.,1.,0.,1.);
		glPointSize(5.0);
		glBegin (GL_POINTS);
		for (unsigned int i = 0; i < points.size(); i++)
			for (unsigned int j = 0; j < points[i].size(); j++)
				glVertex3f (ctlpoints[i][j][0]/ctlpoints[i][j][3],ctlpoints[i][j][1]/ctlpoints[i][j][3],ctlpoints[i][j][2]/ctlpoints[i][j][3]);
		glEnd();
		glPopMatrix();
	}
	break;
	case DrawableNode::WIRE: // Draw nice frame
	{
		for (std::size_t i = 0; i < _curvesU.size(); i++) {
			_curvesU[i]->draw(info,DrawableNode::SOLID,alpha);
		}
		for (std::size_t i = 0; i < _curvesV.size(); i++) {
			_curvesV[i]->draw(info,DrawableNode::SOLID,alpha);
		}
	}
	break;
	}
}

void RenderBezierSurface::setColor(float r, float g, float b) {
	_r = r;
	_g = g;
	_b = b;
}

Vector3D<float> RenderBezierSurface::getColor() const {
	return Vector3D<float>(_r,_g,_b);
}
