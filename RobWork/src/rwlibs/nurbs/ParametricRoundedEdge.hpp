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

#ifndef RWLIBS_NURBS_PARAMETRICROUNDEDEDGE_HPP_
#define RWLIBS_NURBS_PARAMETRICROUNDEDEDGE_HPP_

/**
 * @file ParametricRoundedEdge.hpp
 *
 * \copydoc rwlibs::nurbs::ParametricRoundedEdge
 */

#include <rw/geometry/Line.hpp>
#include "ParametricSurface.hpp"
#include "ParametricCurve.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class ParametricRoundedEdge: public rwlibs::nurbs::ParametricSurface {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ParametricRoundedEdge> Ptr;

	ParametricRoundedEdge(rw::geometry::Line* line, rw::math::Vector3D<> dir, double roundedRadius, double angle = 90.);
	ParametricRoundedEdge(ParametricCurve* curve, rw::math::Vector3D<> dir, double roundedRadius, double angle = 90.);

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::isConvex
	 */
	virtual bool isConvex();

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::evaluate
	 */
	virtual rw::math::Vector3D<> evaluate(double u, double v);

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::normal
	 */
	rw::math::Vector3D<> normal(double u, double v);

	rw::geometry::Line* getLine();
	rw::math::Vector3D<> getDirection();
	double getRadius();
	double getAngle();

private:
	rw::geometry::Line* _line;
	ParametricCurve* _curve;
	rw::math::Vector3D<> _dir;
	double _radius, _angle;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_PARAMETRICROUNDEDEDGE_HPP_ */
