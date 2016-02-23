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

#ifndef RWLIBS_NURBS_GEOMETRICNURBSCURVE_HPP_
#define RWLIBS_NURBS_GEOMETRICNURBSCURVE_HPP_

/**
 * @file GeometricNURBSCurve.hpp
 *
 * \copydoc rwlibs::nurbs::GeometricNURBSCurve
 */

#include "GeometricCurve.hpp"
#include "NURBSCurveND.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class GeometricNURBSCurve: public rwlibs::nurbs::GeometricCurve, public NURBSCurveND<3> {
public:
	GeometricNURBSCurve();
	GeometricNURBSCurve(const std::vector<rw::math::Vector3D<> > &points, const std::vector<double> &weights = std::vector<double>(), const std::vector<double> &knots = std::vector<double>(), std::size_t order = 0);
	GeometricNURBSCurve(const std::vector<rw::math::VectorND<3> > &points, const std::vector<double> &weights = std::vector<double>(), const std::vector<double> &knots = std::vector<double>(), std::size_t order = 0);
	GeometricNURBSCurve(const std::vector<rw::math::VectorND<4> > &pointsHomogeneous, const std::vector<double> &knots, std::size_t order);
	GeometricNURBSCurve(const GeometricNURBSCurve &curve);
	GeometricNURBSCurve(const NUBSCurveND<3> &curve);
	GeometricNURBSCurve(const NURBSCurveND<3> &curve);
	virtual ~GeometricNURBSCurve();

	GeometricNURBSCurve& operator=( const GeometricNURBSCurve& rhs);

	virtual rw::math::Vector3D<> evaluate(double t) const;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_GEOMETRICNURBSCURVE_HPP_ */
