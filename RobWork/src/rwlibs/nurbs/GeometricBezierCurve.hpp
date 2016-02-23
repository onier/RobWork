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

#ifndef RWLIBS_NURBS_GEOMETRICBEZIERCURVE_HPP_
#define RWLIBS_NURBS_GEOMETRICBEZIERCURVE_HPP_

/**
 * @file GeometricBezierCurve.hpp
 *
 * \copydoc rwlibs::nurbs::GeometricBezierCurve
 */

#include "GeometricCurve.hpp"
#include "BezierCurveRationalND.hpp"

namespace rwlibs {
namespace nurbs {

//! @addtogroup nurbs
//! @{
class GeometricBezierCurve: public rwlibs::nurbs::GeometricCurve, public BezierCurveRationalND<3> {
public:
	GeometricBezierCurve();
	GeometricBezierCurve(const std::vector<rw::math::Vector3D<> > &points, const std::vector<double> &weights = std::vector<double>());
	GeometricBezierCurve(const std::vector<rw::math::VectorND<3> > &points, const std::vector<double> &weights = std::vector<double>());
	GeometricBezierCurve(const std::vector<rw::math::VectorND<4> > &pointsHomogeneous);
	GeometricBezierCurve(const GeometricBezierCurve &curve);
	GeometricBezierCurve(const BezierCurveND<3> &curve);
	GeometricBezierCurve(const BezierCurveRationalND<3> &curve);
	virtual ~GeometricBezierCurve();

	GeometricBezierCurve* elevate(std::size_t times = 1);

	GeometricBezierCurve& operator=( const GeometricBezierCurve& rhs);
	const GeometricBezierCurve operator+( const GeometricBezierCurve& b) const;

	virtual rw::math::Vector3D<> evaluate(double t) const;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_GEOMETRICBEZIERCURVE_HPP_ */
