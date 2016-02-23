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

#include "RenderICM.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::graphics;
using namespace rws;

RenderICM::RenderICM():
	_active(false),
	_radius(0.0),
	_stepSize(10.0*Deg2Rad)
{
}

RenderICM::RenderICM(Vector3D<> icm, double radius, Vector3D<> normal, float angleres):
	_active(true),
	_icm(icm),
	_radius(radius),
	_normal(normalize(normal)),
	_stepSize(angleres*Deg2Rad)
{
}

RenderICM::~RenderICM(){}

void RenderICM::setICM(Vector3D<> icm, double radius, Vector3D<> normal){
	_icm = icm;
	_radius = radius;
	_normal = normalize(normal);
	_active = true;
}

void RenderICM::setInactive(){
	_active = false;
}

void RenderICM::setColor(double r, double g, double b){
	_color[0] = (float)r;
	_color[1] = (float)g;
	_color[2] = (float)b;
}

void RenderICM::draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {
	if (_active) {
		glColor3fv(_color);

		Rotation3D<> rot = EAA<>(_normal, _stepSize).toRotation3D();
		Vector3D<> nn( _normal(2), _normal(0), _normal(1));
		double r = _radius+0.05;
		Vector3D<> p = nn;
		glBegin(GL_LINE_LOOP);
		for (int i=0; i<360/10; i++){
			glVertex3f(_icm(0)+p(0)*r,_icm(1)+p(1)*r, _icm(2)+p(2)*r);
			p = rot*p;
		}
		glEnd( );
	}
}
