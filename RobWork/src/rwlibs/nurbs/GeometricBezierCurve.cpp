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

#include "GeometricBezierCurve.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::nurbs;

GeometricBezierCurve::GeometricBezierCurve():
	BezierCurveRationalND<3>()
{
}

GeometricBezierCurve::GeometricBezierCurve(const std::vector<Vector3D<> > &points, const std::vector<double> &weights):
	BezierCurveRationalND<3>(ParametricUtility::toVectorND(points),weights)
{
}

GeometricBezierCurve::GeometricBezierCurve(const std::vector<VectorND<3> > &points, const std::vector<double> &weights):
	BezierCurveRationalND<3>(points,weights)
{
}

GeometricBezierCurve::GeometricBezierCurve(const std::vector<VectorND<4> > &pointsHomogeneous):
	BezierCurveRationalND<3>(pointsHomogeneous)
{
}

GeometricBezierCurve::GeometricBezierCurve(const GeometricBezierCurve &curve):
	BezierCurveRationalND<3>(curve.getPoints(),curve.getWeights())
{
}

GeometricBezierCurve::GeometricBezierCurve(const BezierCurveND<3> &curve):
	BezierCurveRationalND<3>(curve.getPoints())
{
}

GeometricBezierCurve::GeometricBezierCurve(const BezierCurveRationalND<3> &curve):
	BezierCurveRationalND<3>(curve.getPoints(),curve.getWeights())
{
}

GeometricBezierCurve::~GeometricBezierCurve()
{
}

GeometricBezierCurve* GeometricBezierCurve::elevate(std::size_t times) {
	GeometricBezierCurve* curve = new GeometricBezierCurve(*BezierCurveRationalND<3>::elevate(times));
	return curve;
}

GeometricBezierCurve& GeometricBezierCurve::operator=( const GeometricBezierCurve& rhs) {
	const BezierCurveRationalND<3>& bezierCurve = static_cast<BezierCurveRationalND<3> const &>(rhs);
	BezierCurveRationalND<3>::operator=(bezierCurve);
	return *this;
}

const GeometricBezierCurve GeometricBezierCurve::operator+( const GeometricBezierCurve& b) const {
	const BezierCurveRationalND<3>& bezierCurve = static_cast<BezierCurveRationalND<3> const &>(b);
	return BezierCurveRationalND<3>::operator+(bezierCurve);
}

Vector3D<> GeometricBezierCurve::evaluate(double t) const {
	return ParametricUtility::toVector3D(getValue(t));
}
