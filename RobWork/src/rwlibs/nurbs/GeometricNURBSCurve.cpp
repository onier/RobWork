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

#include "GeometricNURBSCurve.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::nurbs;

GeometricNURBSCurve::GeometricNURBSCurve():
	NURBSCurveND<3>()
{
}

GeometricNURBSCurve::GeometricNURBSCurve(const std::vector<Vector3D<> > &points, const std::vector<double> &weights, const std::vector<double> &knots, std::size_t order):
	NURBSCurveND<3>(ParametricUtility::toVectorND(points),weights,knots,order)
{
}

GeometricNURBSCurve::GeometricNURBSCurve(const std::vector<VectorND<3> > &points, const std::vector<double> &weights, const std::vector<double> &knots, std::size_t order):
	NURBSCurveND<3>(points,weights,knots,order)
{
}

GeometricNURBSCurve::GeometricNURBSCurve(const std::vector<VectorND<4> > &pointsHomogeneous, const std::vector<double> &knots, std::size_t order):
	NURBSCurveND<3>(pointsHomogeneous,knots,order)
{
}

GeometricNURBSCurve::GeometricNURBSCurve(const GeometricNURBSCurve &curve):
	NURBSCurveND<3>(curve.getPoints(),curve.getWeights(),curve.getKnots(),curve.getOrder())
{
}

GeometricNURBSCurve::GeometricNURBSCurve(const NUBSCurveND<3> &curve):
	NURBSCurveND<3>(curve.getPoints(),std::vector<double>(),curve.getKnots(),curve.getOrder())
{
}

GeometricNURBSCurve::GeometricNURBSCurve(const NURBSCurveND<3> &curve):
	NURBSCurveND<3>(curve.getPoints(),curve.getWeights(),curve.getKnots(),curve.getOrder())
{
}

GeometricNURBSCurve::~GeometricNURBSCurve()
{
}

GeometricNURBSCurve& GeometricNURBSCurve::operator=( const GeometricNURBSCurve& rhs) {
	const NURBSCurveND<3>& bezierCurve = static_cast<NURBSCurveND<3> const &>(rhs);
	NURBSCurveND<3>::operator=(bezierCurve);
	return *this;
}

Vector3D<> GeometricNURBSCurve::evaluate(double t) const {
	return ParametricUtility::toVector3D(getValue(t));
}
