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

#ifndef RWLIBS_NURBS_PARAMETRICPLANEFACE_HPP_
#define RWLIBS_NURBS_PARAMETRICPLANEFACE_HPP_

/**
 * @file ParametricPlaneFace.hpp
 *
 * \copydoc rwlibs::nurbs::ParametricPlaneFace
 */

#include "ParametricSurface.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class ParametricPlaneFace: public rwlibs::nurbs::ParametricSurface {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ParametricPlaneFace> Ptr;

	ParametricPlaneFace(rw::math::Transform3D<> location, double sizeX, double sizeY);

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::getTriMesh
	 */
	virtual rw::common::Ptr<rw::geometry::TriMesh> getTriMesh(bool forceCopy=true);

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
	std::vector<rw::math::Vector3D<> > getCorners();

private:
	rw::math::Transform3D<> _location;
	double _sizeX;
	double _sizeY;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_PARAMETRICPLANEFACE_HPP_ */
