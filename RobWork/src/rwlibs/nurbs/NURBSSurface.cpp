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

#include "NURBSSurface.hpp"

#include <rw/geometry/Cone.hpp>
#include <rw/geometry/Sphere.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::nurbs;

bool NURBSSurface::isConvex() {
	return false;
}

Vector3D<> NURBSSurface::evaluate(double u, double v) {
	Vector3D<> sum = Vector3D<>::zero();
	std::vector<double> knotsU = getFullKnots().first;
	std::vector<double> knotsV = getFullKnots().second;
	double sumW = 0;
	for (std::size_t i = 0; i < knotsU.size()-getDegree().first-1; i++) {
		double basisI = B(i,getDegree().first,knotsU,u);
		if (u == knotsU.back() && i == knotsU.size()-getDegree().first-2)
			basisI = 1;
		for (std::size_t j = 0; j < knotsV.size()-getDegree().second-1; j++) {
			double basisJ = B(j,getDegree().second,knotsV,v);
			if (v == knotsV.back() && j == knotsV.size()-getDegree().second-2)
				basisJ = 1;
			double basis = basisI*basisJ;
			sum += _weights[i][j]*_controlPoints[i][j]*basis;
			sumW += _weights[i][j]*basis;
		}
	}
	return sum/sumW;
}

Vector3D<> NURBSSurface::normal(double u, double v) {
	return Vector3D<>::zero();
}

NURBSSurface::NURBSSurface(unsigned int orderU, unsigned int orderV,
		std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
		std::vector<double> knotsU, std::vector<double> knotsV,
		std::vector<std::vector<double> > weights) :
		_order(std::make_pair(orderU, orderV)),
		_controlPoints(controlPoints),
		_knots(std::make_pair(knotsU, knotsV)),
		_weights(weights)
{
	double min = _weights[0][0];
	double max = _weights[0][0];
	for (std::size_t u = 0; u < _weights.size(); u++) {
		for (std::size_t v = 0; v < _weights[0].size(); v++) {
			if (_weights[u][v] < min)
				min = _weights[u][v];
			if (_weights[u][v] > max)
				max = _weights[u][v];
		}
	}
	_weightRatio = max/min;
}

NURBSSurface::NURBSSurface(const NURBSSurface &surface):
	_order(surface.getOrder()),
	_controlPoints(surface.getControlPoints()),
	_knots(surface.getKnots()),
	_weights(surface.getWeights())
{
	double min = _weights[0][0];
	double max = _weights[0][0];
	for (std::size_t u = 0; u < _weights.size(); u++) {
		for (std::size_t v = 0; v < _weights[0].size(); v++) {
			if (_weights[u][v] < min)
				min = _weights[u][v];
			if (_weights[u][v] > max)
				max = _weights[u][v];
		}
	}
	_weightRatio = max/min;
}

NURBSSurface::~NURBSSurface() {
}

std::pair<std::size_t, std::size_t> NURBSSurface::getDegree() const {
	return std::make_pair(_order.first-1, _order.second-1);
}

std::pair<std::size_t, std::size_t> NURBSSurface::getOrder() const {
	return _order;
}

std::vector<std::vector<rw::math::Vector3D<> > > NURBSSurface::getControlPoints() const {
	return _controlPoints;
}

std::pair<std::vector<double>, std::vector<double> > NURBSSurface::getKnots() const {
	return _knots;
}

std::pair<std::vector<double>, std::vector<double> > NURBSSurface::getFullKnots() const {
	std::vector<double> fullKnotsU, fullKnotsV;
	fullKnotsU.push_back(_knots.first.front());
	fullKnotsU.insert(fullKnotsU.end(),_knots.first.begin(),_knots.first.end());
	fullKnotsU.push_back(_knots.first.back());
	fullKnotsV.push_back(_knots.second.front());
	fullKnotsV.insert(fullKnotsV.end(),_knots.second.begin(),_knots.second.end());
	fullKnotsV.push_back(_knots.second.back());
	return std::make_pair(fullKnotsU,fullKnotsV);
}

std::vector<std::vector<double> > NURBSSurface::getWeights() const {
	return _weights;
}

double NURBSSurface::B(unsigned int i, unsigned int n, std::vector<double> knots, double t) {
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

NURBSSurface::NURBSSurfacePair NURBSSurface::divideU(double knot) const {
	if (knot < _knots.first.front() || knot > _knots.first.back())
		RW_THROW("NURBSSurface::divideU: given knot value is outside range.");
	NURBSSurfacePair pair;
	NURBSSurface* copy = new NURBSSurface(*this);
	unsigned int index = findKnotIndex(_knots.first,knot);
	unsigned int mult = getMultiplicityU(index);
	for(unsigned int i = 0; i < _order.first-mult; i++) {
		copy->insertKnotU(knot);
	}
	std::vector<double> knotsU = copy->getKnots().first;
	std::vector<double> knotsU1(knotsU.begin(),knotsU.begin()+index+_order.first-mult);
	std::vector<double> knotsU2(knotsU.begin()+index,knotsU.end());
	std::vector<std::vector<Vector3D<> > > controlPoints = copy->getControlPoints();
	std::vector<std::vector<Vector3D<> > > controlPoints1(controlPoints.begin(),controlPoints.begin()+knotsU1.size()-_order.first+3);
	std::vector<std::vector<Vector3D<> > > controlPoints2(controlPoints.begin()+knotsU1.size()-_order.first+1,controlPoints.end());
	std::vector<std::vector<double> > weights = copy->getWeights();
	std::vector<std::vector<double> > weights1(weights.begin(),weights.begin()+knotsU1.size()-_order.first+3);
	std::vector<std::vector<double> > weights2(weights.begin()+knotsU1.size()-_order.first+1,weights.end());
	pair.first = ownedPtr( new NURBSSurface(_order.first,_order.second,controlPoints1,knotsU1,_knots.second,weights1) );
	pair.second = ownedPtr( new NURBSSurface(_order.first,_order.second,controlPoints2,knotsU2,_knots.second,weights2) );
	delete copy;
	return pair;
}

NURBSSurface::NURBSSurfacePair NURBSSurface::divideV(double knot) const {
	if (knot < _knots.second.front() || knot > _knots.second.back())
		RW_THROW("NURBSSurface::divideV: given knot value is outside range.");
	NURBSSurfacePair pair;
	NURBSSurface* copy = new NURBSSurface(*this);
	unsigned int index = findKnotIndex(_knots.second,knot);
	unsigned int mult = getMultiplicityV(index);
	for(unsigned int i = 0; i < _order.second-mult; i++) {
		copy->insertKnotV(knot);
	}
	std::vector<double> knotsV = copy->getKnots().second;
	std::vector<double> knotsV1(knotsV.begin(),knotsV.begin()+index+_order.second-mult);
	std::vector<double> knotsV2(knotsV.begin()+index,knotsV.end());
	std::vector<std::vector<Vector3D<> > > controlPoints = copy->getControlPoints();
	std::vector<std::vector<Vector3D<> > > controlPoints1(controlPoints.size());
	std::vector<std::vector<Vector3D<> > > controlPoints2(controlPoints.size());
	std::vector<std::vector<double> > weights = copy->getWeights();
	std::vector<std::vector<double> > weights1(weights.size());
	std::vector<std::vector<double> > weights2(weights.size());
	for (unsigned int i = 0; i < controlPoints.size(); i++) {
		std::vector<Vector3D<> > controlPoint1(controlPoints[i].begin(),controlPoints[i].begin()+knotsV1.size()-_order.second+3);
		std::vector<Vector3D<> > controlPoint2(controlPoints[i].begin()+knotsV1.size()-_order.second+1,controlPoints[i].end());
		std::vector<double> weight1(weights[i].begin(),weights[i].begin()+knotsV1.size()-_order.second+3);
		std::vector<double> weight2(weights[i].begin()+knotsV1.size()-_order.second+1,weights[i].end());
		controlPoints1[i] = controlPoint1;
		controlPoints2[i] = controlPoint2;
		weights1[i] = weight1;
		weights2[i] = weight2;
	}
	pair.first = ownedPtr( new NURBSSurface(_order.first,_order.second,controlPoints1,_knots.first,knotsV1,weights1) );
	pair.second = ownedPtr( new NURBSSurface(_order.first,_order.second,controlPoints2,_knots.second,knotsV2,weights2) );
	delete copy;
	return pair;
}

unsigned int NURBSSurface::getMultiplicityU(unsigned int knotIndex) const {
	unsigned int mult = 0;
	for (size_t i = knotIndex; i < _knots.first.size(); i++) {
		if (_knots.first[i] == _knots.first[knotIndex])
			mult++;
		else
			break;
	}
	for (size_t i = knotIndex-1; i > 0; i--) {
		if (_knots.first[i] == _knots.first[knotIndex])
			mult++;
		else
			break;
	}
	return mult;
}

unsigned int NURBSSurface::getMultiplicityV(unsigned int knotIndex) const {
	unsigned int mult = 0;
	for (size_t i = knotIndex; i < _knots.second.size(); i++) {
		if (_knots.second[i] == _knots.second[knotIndex])
			mult++;
		else
			break;
	}
	for (size_t i = knotIndex-1; i > 0; i--) {
		if (_knots.second[i] == _knots.second[knotIndex])
			mult++;
		else
			break;
	}
	return mult;
}

void NURBSSurface::insertKnotU(double val) {
	std::vector<double> knots = _knots.first;
	unsigned int index = findKnotIndex(knots,val);
	std::size_t pointsV = _controlPoints[0].size();

	std::vector<std::vector<Q> > P;
	for (std::size_t u = index-(_order.first-1); u <= index; u++) {
		std::vector<Q> Ps;
		for (std::size_t v = 0; v < pointsV; v++) {
			Vector3D<> cp = _controlPoints[u][v];
			double w = _weights[u][v];
			Q PVal(4,cp[0]*w,cp[1]*w,cp[2]*w,w);
			Ps.push_back(PVal);
		}
		P.push_back(Ps);
	}

	std::vector<double> a;
	for (std::size_t u = index-(_order.first-2); u <= index; u++) {
		a.push_back((val-knots[u-1])/(knots[u-1+_order.first-1]-knots[u-1]));
	}

	// Expand and make room for everything
	_knots.first.insert(_knots.first.begin()+index,val);
	std::vector<Vector3D<> > newVec(pointsV,Vector3D<>::zero());
	_controlPoints.insert(_controlPoints.begin()+index,newVec);
	std::vector<double> newWeights(pointsV,0.);
	_weights.insert(_weights.begin()+index,newWeights);

	std::size_t ind = 0;
	for (std::size_t u = index-(_order.first-2); u <= index; u++) {
		for (std::size_t v = 0; v < pointsV; v++) {
			Q Qq = (1-a[ind])*P[ind][v]+a[ind]*P[ind+1][v];
			Vector3D<> cp(Qq[0]/Qq[3],Qq[1]/Qq[3],Qq[2]/Qq[3]);
			_controlPoints[u][v] = cp;
			_weights[u][v] = Qq[3];
		}
		ind++;
	}
}

void NURBSSurface::insertKnotV(double val) {
	std::vector<double> knots = _knots.second;
	unsigned int index = findKnotIndex(knots,val);
	std::size_t pointsU = _controlPoints.size();

	std::vector<std::vector<Q> > P;
	for (std::size_t v = index-(_order.second-1); v <= index; v++) {
		std::vector<Q> Ps;
		for (std::size_t u = 0; u < pointsU; u++) {
			Vector3D<> cp = _controlPoints[u][v];
			double w = _weights[u][v];
			Q PVal(4,cp[0]*w,cp[1]*w,cp[2]*w,w);
			Ps.push_back(PVal);
		}
		P.push_back(Ps);
	}

	std::vector<double> a;
	for (std::size_t v = index-(_order.second-2); v <= index; v++) {
		a.push_back((val-knots[v-1])/(knots[v-1+_order.second-1]-knots[v-1]));
	}

	// Expand and make room for everything
	_knots.second.insert(_knots.second.begin()+index,val);
	for (unsigned int u = 0; u < pointsU; u++) {
		_controlPoints[u].insert(_controlPoints[u].begin()+index,Vector3D<>::zero());
		_weights[u].insert(_weights[u].begin()+index,0.);
	}

	std::size_t ind = 0;
	for (std::size_t v = index-(_order.second-2); v <= index; v++) {
		for (unsigned int u = 0; u < pointsU; u++) {
			Q Qq = (1-a[ind])*P[ind][u]+a[ind]*P[ind+1][u];
			Vector3D<> cp(Qq[0]/Qq[3],Qq[1]/Qq[3],Qq[2]/Qq[3]);
			_controlPoints[u][v] = cp;
			_weights[u][v] = Qq[3];
		}
		ind++;
	}
}

std::vector<Vector3D<> > NURBSSurface::getDerivativeBoundsU() const {
	std::vector<Vector3D<> > vectors;
	for (std::size_t i = 0; i < _controlPoints.size()-1; i++) {
		for (std::size_t h = 0; h < _controlPoints[0].size(); h++) {
			for (std::size_t k = 0; k < _controlPoints[0].size(); k++) {
				Vector3D<> vec = _weights[i+1][h]/_weights[i+1][k]*_controlPoints[i+1][h]-_weights[i][h]/_weights[i][k]*_controlPoints[i][h];
				vectors.push_back(normalize(vec));
			}
		}
	}
	return vectors;
}

std::vector<Vector3D<> > NURBSSurface::getDerivativeBoundsV() const {
	std::vector<Vector3D<> > vectors;
	for (std::size_t i = 0; i < _controlPoints[0].size()-1; i++) {
		for (std::size_t h = 0; h < _controlPoints.size(); h++) {
			for (std::size_t k = 0; k < _controlPoints.size(); k++) {
				Vector3D<> vec = _weights[h][i+1]/_weights[k][i+1]*_controlPoints[h][i+1]-_weights[h][i]/_weights[k][i]*_controlPoints[h][i];
				vectors.push_back(normalize(vec));
			}
		}
	}
	return vectors;
}

Geometry::Ptr NURBSSurface::getBoundingSphere() const {
	Vector3D<> avg = Vector3D<>::zero();
	for(std::size_t u = 0; u < _controlPoints.size(); u++) {
		for(std::size_t v = 0; v < _controlPoints[0].size(); v++) {
			avg += _controlPoints[u][v];
		}
	}
	avg /= _controlPoints.size()*_controlPoints[0].size();
	double maxDist = 0;
	for(std::size_t u = 0; u < _controlPoints.size(); u++) {
		for(std::size_t v = 0; v < _controlPoints[0].size(); v++) {
			double dist = (_controlPoints[u][v]-avg).norm2();
			if (dist > maxDist)
				maxDist = dist;
		}
	}
	GeometryData::Ptr data = ownedPtr( new Sphere(maxDist) );
	Transform3D<> transform(avg,Rotation3D<>::identity());
	return ownedPtr( new Geometry(data,transform) );
}

std::pair<rw::math::Vector3D<>, double> NURBSSurface::getTangentConeU() const {
	std::vector<Vector3D<> > dirs = getDerivativeBoundsU();
	Vector3D<> avg = Vector3D<>::zero();
	for(std::size_t i = 0; i < dirs.size(); i++) {
		avg += dirs[i];
	}
	avg /= dirs.size();
	avg = normalize(avg);
	double maxAngle = 0;
	for(std::size_t i = 0; i < dirs.size(); i++) {
		double angle = std::acos(dot(avg,dirs[i]));
		if (angle > maxAngle)
			maxAngle = angle;
	}
	return std::make_pair(avg, std::tan(maxAngle));
}

unsigned int NURBSSurface::findKnotIndex(const std::vector<double> &knots, double value) {
	std::size_t index = 0;
	for (std::size_t i = 1; i < knots.size(); i++) {
		if (knots[i-1] <= value && knots[i] > value)
			index = i;
	}
	if (knots.back() <= value)
		index = knots.size()-1;
	return index;
}
