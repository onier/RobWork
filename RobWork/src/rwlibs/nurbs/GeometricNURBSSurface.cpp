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

#include "GeometricNURBSSurface.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::nurbs;

GeometricNURBSSurface::GeometricNURBSSurface():
	NURBSSurfaceND<3>()
{
}

GeometricNURBSSurface::GeometricNURBSSurface(
		const PointMap3D &points,
		const NURBSSurfaceND<3>::WeightMap &weights,
		const std::vector<double> &knotsU,
		const std::vector<double> &knotsV,
		std::size_t orderU,
		std::size_t orderV
		):
	NURBSSurfaceND<3>(ParametricUtility::toVectorND(points),weights,knotsU,knotsV,orderU,orderV)
{
}

GeometricNURBSSurface::GeometricNURBSSurface(
		const NURBSSurfaceND<3>::PointMap &points,
		const NURBSSurfaceND<3>::WeightMap &weights,
		const std::vector<double> &knotsU,
		const std::vector<double> &knotsV,
		std::size_t orderU,
		std::size_t orderV
		):
	NURBSSurfaceND<3>(points,weights,knotsU,knotsV,orderU,orderV)
{
}

GeometricNURBSSurface::GeometricNURBSSurface(
		const NURBSSurfaceND<3>::PointMapHomogeneous &pointsHomogeneous,
		const std::vector<double> &knotsU,
		const std::vector<double> &knotsV,
		std::size_t orderU,
		std::size_t orderV
		):
	NURBSSurfaceND<3>(pointsHomogeneous,knotsU,knotsV,orderU,orderV)
{
}

GeometricNURBSSurface::GeometricNURBSSurface(const GeometricNURBSSurface &surface):
	NURBSSurfaceND<3>(surface.getPoints(),surface.getWeights(),surface.getKnotsU(),surface.getKnotsV(),surface.getOrderU(),surface.getOrderV())
{
}

GeometricNURBSSurface::GeometricNURBSSurface(const NUBSSurfaceND<3> &surface):
	NURBSSurfaceND<3>(surface.getPoints(),NURBSSurfaceND<3>::WeightMap(),surface.getKnotsU(),surface.getKnotsV(),surface.getOrderU(),surface.getOrderV())
{
}

GeometricNURBSSurface::GeometricNURBSSurface(const NURBSSurfaceND<3> &surface):
	NURBSSurfaceND<3>(surface.getPoints(),surface.getWeights(),surface.getKnotsU(),surface.getKnotsV(),surface.getOrderU(),surface.getOrderV())
{
}

GeometricNURBSSurface::~GeometricNURBSSurface()
{
}

GeometricNURBSSurface& GeometricNURBSSurface::operator=( const GeometricNURBSSurface& rhs) {
	const NURBSSurfaceND<3>& bezierSurface = static_cast<NURBSSurfaceND<3> const &>(rhs);
	NURBSSurfaceND<3>::operator=(bezierSurface);
	return *this;
}

Vector3D<> GeometricNURBSSurface::evaluate(double u, double v) const {
	return ParametricUtility::toVector3D(getValue(u,v));
}
