/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RenderContacts.hpp"
#include "Contact.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::graphics;
using namespace rwsim::contacts;

struct RenderContacts::GLData {
	GLData(): quadratic(gluNewQuadric()) {}
	~GLData() {
		gluDeleteQuadric(quadratic);
	}
	GLUquadricObj* quadratic;
};

RenderContacts::RenderContacts():
	_gl(new GLData()),
	_showPoints(std::make_pair(true,true)),
	_showNormals(std::make_pair(true,true))
{
	setSphereRadius();
	setNormalLength();
}

RenderContacts::RenderContacts(const std::vector<Contact> &contacts):
	_contacts(contacts),
	_gl(new GLData()),
	_showPoints(std::make_pair(true,true)),
	_showNormals(std::make_pair(true,true))
{
	setSphereRadius();
	setNormalLength();
}

RenderContacts::~RenderContacts() {
	delete _gl;
}

void RenderContacts::setContacts(const std::vector<Contact> &contacts) {
	_contacts = contacts;
}

std::vector<Contact> RenderContacts::getContacts() const {
	return _contacts;
}

void RenderContacts::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const {
	BOOST_FOREACH(const Contact& con, _contacts){
		Vector3D<> posA = con.getPointA();
		Vector3D<> posB = con.getPointB();
		Vector3D<> n = normalize(con.getNormal())*_normalLength;

		if (_showPoints.first || _showNormals.first) {
			glPushMatrix();
			glTranslatef((GLfloat)posA(0),(GLfloat)posA(1),(GLfloat)posA(2));

			if (_showPoints.first) {
				glColor3f(_colorPoint[0], _colorPoint[1], _colorPoint[2]);
				gluSphere( _gl->quadratic, _sphereRadius, 32, 32);
			}

			if (_showNormals.first) {
				glBegin(GL_LINES);
				glColor3f(_colorNormal[0], _colorNormal[1], _colorNormal[2]);
				glVertex3d(0,0,0);
				glVertex3d(n(0),n(1),n(2));
				glEnd();
			}

			glPopMatrix();
		}

		if (_showPoints.second || _showNormals.second) {
			glPushMatrix();
			glTranslatef((GLfloat)posB(0),(GLfloat)posB(1),(GLfloat)posB(2));

			if (_showPoints.second) {
				glColor3f(_colorPoint[0], _colorPoint[1], _colorPoint[2]);
				gluSphere( _gl->quadratic, _sphereRadius, 32, 32);
			}

			if (_showNormals.second) {
				glBegin(GL_LINES);
				glColor3f(_colorNormal[0], _colorNormal[1], _colorNormal[2]);
				glVertex3d(0,0,0);
				glVertex3d(-n(0),-n(1),-n(2));
				glEnd();
			}

			glPopMatrix();
		}
	}
}

void RenderContacts::setColorPoints(float r, float g, float b) {
	_colorPoint = Vector3D<float>(r,g,b);
}

void RenderContacts::setColorNormal(float r, float g, float b) {
	_colorNormal = Vector3D<float>(r,g,b);
}

Vector3D<float> RenderContacts::getColorPoint() const {
	return _colorPoint;
}

Vector3D<float> RenderContacts::getColorNormal() const {
	return _colorNormal;
}
