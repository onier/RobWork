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

#include "GeometricBezierSurface.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::nurbs;

GeometricBezierSurface::GeometricBezierSurface():
	BezierSurfaceRationalND<3>()
{
}

GeometricBezierSurface::GeometricBezierSurface(const PointMap3D &points, const BezierSurfaceRationalND<3>::WeightMap &weights):
	//BezierSurfaceRationalND<3>(ParametricUtility::toVectorND(points),weights)
	BezierSurfaceRationalND<3>()
{
}

GeometricBezierSurface::GeometricBezierSurface(const BezierSurfaceRationalND<3>::PointMap &points, const BezierSurfaceRationalND<3>::WeightMap &weights):
	BezierSurfaceRationalND<3>(points,weights)
{
}

GeometricBezierSurface::GeometricBezierSurface(const BezierSurfaceRationalND<3>::PointMapHomogeneous &pointsHomogeneous):
	BezierSurfaceRationalND<3>(pointsHomogeneous)
{
}

GeometricBezierSurface::GeometricBezierSurface(const GeometricBezierSurface &surface):
	BezierSurfaceRationalND<3>(surface.getPoints(),surface.getWeights())
{
}

GeometricBezierSurface::GeometricBezierSurface(const BezierSurfaceND<3> &surface):
	BezierSurfaceRationalND<3>(surface.getPoints())
{
}

GeometricBezierSurface::GeometricBezierSurface(const BezierSurfaceRationalND<3> &surface):
	BezierSurfaceRationalND<3>(surface.getPoints(),surface.getWeights())
{
}

GeometricBezierSurface::~GeometricBezierSurface()
{
}

GeometricBezierSurface& GeometricBezierSurface::operator=( const GeometricBezierSurface& rhs) {
	const BezierSurfaceRationalND<3>& bezierSurface = static_cast<BezierSurfaceRationalND<3> const &>(rhs);
	BezierSurfaceRationalND<3>::operator=(bezierSurface);
	return *this;
}

Vector3D<> GeometricBezierSurface::evaluate(double u, double v) const {
	return ParametricUtility::toVector3D(getValue(u,v));
}
