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

#include "NURBSCurve.hpp"

using rw::common::Ptr;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::nurbs;

Ptr<TriMesh> NURBSCurve::getTriMesh(bool forceCopy) {
	return NULL;
}

bool NURBSCurve::isConvex() {
	return false;
}

NURBSCurve::NURBSCurve(
		unsigned int order,
		std::vector<rw::math::Vector3D<> > controlPoints,
		std::vector<double> knots,
		std::vector<double> weights):
	_order(order),
	_controlPoints(controlPoints),
	_knots(knots),
	_weights(weights)
{
}

NURBSCurve::~NURBSCurve() {
}

std::size_t NURBSCurve::getDegree() const {
	return _order-1;
}

std::size_t NURBSCurve::getOrder() const {
	return _order;
}

std::vector<Vector3D<> > NURBSCurve::getControlPoints() const {
	return _controlPoints;
}

std::vector<double> NURBSCurve::getKnots() const {
	return _knots;
}

std::vector<double> NURBSCurve::getFullKnots() const {
	std::vector<double> fullKnots;
	fullKnots.push_back(_knots.front());
	fullKnots.insert(fullKnots.end(),_knots.begin(),_knots.end());
	fullKnots.push_back(_knots.back());
	return fullKnots;
}

std::vector<double> NURBSCurve::getWeights() const {
	return _weights;
}

Vector3D<> NURBSCurve::evaluate(double t) {
	Vector3D<> sum = Vector3D<>::zero();
	double sumW = 0;
	std::vector<double> knots = getFullKnots();
	for (std::size_t i = 0; i < knots.size()-getDegree()-1; i++) {
		double basis = B(i,getDegree(),knots,t);
		if (t == knots.back() && i == knots.size()-getDegree()-2)
			basis = 1;
		sum += _weights[i]*_controlPoints[i]*basis;
		sumW += _weights[i]*basis;
	}
	return sum/sumW;
}

Vector3D<> NURBSCurve::evaluateDerivative(double t, unsigned int times) {
	return Vector3D<>::zero();
}

NURBSCurve* NURBSCurve::makeCircle(Vector3D<> center, Vector3D<> normal, double radius) {
	Vector3D<> dir1 = getPerpendicular(normal);
	Vector3D<> dir2 = normalize(cross(normal,dir1));
	unsigned int order = 3;
	std::vector<Vector3D<> > controlPoints;
	controlPoints.push_back(Vector3D<>(center+dir1*radius));
	controlPoints.push_back(Vector3D<>(center+dir1*radius+dir2*radius));
	controlPoints.push_back(Vector3D<>(center-dir1*radius+dir2*radius));
	controlPoints.push_back(Vector3D<>(center-dir1*radius));
	controlPoints.push_back(Vector3D<>(center-dir1*radius-dir2*radius));
	controlPoints.push_back(Vector3D<>(center+dir1*radius-dir2*radius));
	controlPoints.push_back(Vector3D<>(center+dir1*radius));
	double knotVal[] = {0,0,0.25,0.5,0.5,0.75,1,1};
	std::vector<double> knots(knotVal,knotVal+8);
	double weightsVal[] = {1,0.5,0.5,1,0.5,0.5,1};
	std::vector<double> weights(weightsVal,weightsVal+7);
	return new NURBSCurve(order,controlPoints,knots,weights);
}

double NURBSCurve::B(unsigned int i, unsigned int n, std::vector<double> knots, double t) {
	if (n == 0) {
		if (knots[i] <= t && t < knots[i+1])
			return 1;
		else return 0;
	} else {
		double res = 0;
		if (fabs(knots[i+n]-knots[i]) > 1e-9)
			res += (t-knots[i])/(knots[i+n]-knots[i])*B(i,n-1,knots,t);
		if (fabs(knots[i+n+1]-knots[i+1]) > 1e-9)
			res += (knots[i+n+1]-t)/(knots[i+n+1]-knots[i+1])*B(i+1,n-1,knots,t);
		return res;
	}
}

Vector3D<> NURBSCurve::getPerpendicular(Vector3D<> vec) {
	if (vec[0] < vec[1] && vec[0] < vec[2])
		return normalize(cross(vec,Vector3D<>::x()));
	else {
		if (vec[1] < vec[2])
			return normalize(cross(vec,Vector3D<>::y()));
		else
			return normalize(cross(vec,Vector3D<>::z()));
	}
}
