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

#ifndef RWLIBS_NURBS_GEOMETRICBEZIERSURFACE_HPP_
#define RWLIBS_NURBS_GEOMETRICBEZIERSURFACE_HPP_

/**
 * @file GeometricBezierSurface.hpp
 *
 * \copydoc rwlibs::nurbs::GeometricBezierSurface
 */

#include "GeometricSurface.hpp"
#include "BezierSurfaceRationalND.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class GeometricBezierSurface: public rwlibs::nurbs::GeometricSurface, public BezierSurfaceRationalND<3> {
public:
	typedef std::vector<std::vector<rw::math::Vector3D<> > > PointMap3D;

	GeometricBezierSurface();
	GeometricBezierSurface(const PointMap3D &points, const BezierSurfaceRationalND<3>::WeightMap &weights = BezierSurfaceRationalND<3>::WeightMap());
	GeometricBezierSurface(const BezierSurfaceRationalND<3>::PointMap &points, const BezierSurfaceRationalND<3>::WeightMap &weights = BezierSurfaceRationalND<3>::WeightMap());
	GeometricBezierSurface(const BezierSurfaceRationalND<3>::PointMapHomogeneous &pointsHomogeneous);
	GeometricBezierSurface(const GeometricBezierSurface &surface);
	GeometricBezierSurface(const BezierSurfaceND<3> &surface);
	GeometricBezierSurface(const BezierSurfaceRationalND<3> &surface);
	virtual ~GeometricBezierSurface();

	GeometricBezierSurface& operator=( const GeometricBezierSurface& rhs);

	virtual rw::math::Vector3D<> evaluate(double u, double v) const;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_GEOMETRICBEZIERSURFACE_HPP_ */
