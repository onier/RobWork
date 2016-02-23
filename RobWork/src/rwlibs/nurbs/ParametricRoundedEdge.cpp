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

#include "ParametricRoundedEdge.hpp"
using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::nurbs;

ParametricRoundedEdge::ParametricRoundedEdge(Line* line, Vector3D<> dir, double roundedRadius, double angle):
	_line(line),
	_curve(NULL),
	_dir(dir),
	_radius(roundedRadius),
	_angle(angle)
{
}

ParametricRoundedEdge::ParametricRoundedEdge(ParametricCurve* curve, Vector3D<> dir, double roundedRadius, double angle):
	_line(NULL),
	_curve(curve),
	_dir(dir),
	_radius(roundedRadius),
	_angle(angle)
{
}

bool ParametricRoundedEdge::isConvex(){
	return true;
}

Vector3D<> ParametricRoundedEdge::evaluate(double u, double v)
{
	Vector3D<> centerDir, centerPos;
	if (_curve == NULL) {
		Vector3D<> p1 = _line->p1();
		Vector3D<> p2 = _line->p2();
		centerDir = normalize(p2-p1);
		centerPos = p1+u*(p2-p1);
	} else if (_line == NULL) {
		centerPos = _curve->evaluate(u);
		centerDir = normalize(_curve->evaluate(u+0.01) - centerPos);
	}
	Vector3D<> shellPos = EAA<>(centerDir,v*_angle/180.*rw::math::Pi).toRotation3D()*_dir;
	return centerPos+normalize(shellPos)*_radius;
}

Vector3D<> ParametricRoundedEdge::normal(double u, double v) {
	if (_curve == NULL) {
	Vector3D<> lineDir = _line->p2()-_line->p1();
	Vector3D<> shellPos = EAA<>(normalize(_line->p2()-_line->p1()),v*_angle/180.*rw::math::Pi).toRotation3D()*_dir;
	return normalize(cross(shellPos,lineDir));
	} else return Vector3D<>::z();
}

Line* ParametricRoundedEdge::getLine() {
	return _line;
}

Vector3D<> ParametricRoundedEdge::getDirection() {
	return _dir;
}

double ParametricRoundedEdge::getRadius() {
	return _radius;
}

double ParametricRoundedEdge::getAngle() {
	return _angle;
}
