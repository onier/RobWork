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

#ifndef RWLIBS_NURBS_GEOMETRICNURBSSURFACE_HPP_
#define RWLIBS_NURBS_GEOMETRICNURBSSURFACE_HPP_

/**
 * @file GeometricNURBSSurface.hpp
 *
 * \copydoc rwlibs::nurbs::GeometricNURBSSurface
 */

#include "GeometricSurface.hpp"
#include "NURBSSurfaceND.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class GeometricNURBSSurface: public rwlibs::nurbs::GeometricSurface, public NURBSSurfaceND<3> {
public:
	typedef std::vector<std::vector<rw::math::Vector3D<> > > PointMap3D;

	GeometricNURBSSurface();
	GeometricNURBSSurface(
			const PointMap3D &points,
			const NURBSSurfaceND<3>::WeightMap &weights = NURBSSurfaceND<3>::WeightMap(),
			const std::vector<double> &knotsU = std::vector<double>(),
			const std::vector<double> &knotsV = std::vector<double>(),
			std::size_t orderU = 0,
			std::size_t orderV = 0
			);

	GeometricNURBSSurface(
			const NURBSSurfaceND<3>::PointMap &points,
			const NURBSSurfaceND<3>::WeightMap &weights = NURBSSurfaceND<3>::WeightMap(),
			const std::vector<double> &knotsU = std::vector<double>(),
			const std::vector<double> &knotsV = std::vector<double>(),
			std::size_t orderU = 0,
			std::size_t orderV = 0
			);

	GeometricNURBSSurface(
			const NURBSSurfaceND<3>::PointMapHomogeneous &pointsHomogeneous,
			const std::vector<double> &knotsU = std::vector<double>(),
			const std::vector<double> &knotsV = std::vector<double>(),
			std::size_t orderU = 0,
			std::size_t orderV = 0
			);

	GeometricNURBSSurface(const GeometricNURBSSurface &surface);
	GeometricNURBSSurface(const NUBSSurfaceND<3> &surface);
	GeometricNURBSSurface(const NURBSSurfaceND<3> &surface);
	virtual ~GeometricNURBSSurface();

	GeometricNURBSSurface& operator=( const GeometricNURBSSurface& rhs);

	virtual rw::math::Vector3D<> evaluate(double u, double v) const;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_GEOMETRICNURBSSURFACE_HPP_ */
