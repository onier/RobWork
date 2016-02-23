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

#ifndef RWLIBS_NURBS_GEOMETRICCURVE_HPP_
#define RWLIBS_NURBS_GEOMETRICCURVE_HPP_

/**
 * @file GeometricCurve.hpp
 *
 * \copydoc rwlibs::nurbs::GeometricCurve
 */

#include <rw/geometry/GeometryData.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class GeometricCurve: public rw::geometry::GeometryData {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<GeometricCurve> Ptr;

	GeometricCurve() {}
	~GeometricCurve() {}

	/**
	 * @copydoc rw::geometry::GeometryData::getType
	 */
	virtual GeometryType getType() const {
		return GeometryData::UserType;
	}

	/**
	 * @copydoc rw::geometry::GeometryData::getTriMesh
	 */
	virtual rw::common::Ptr<rw::geometry::TriMesh> getTriMesh(bool forceCopy=true) {
		return NULL;
	}

	/**
	 * @copydoc rw::geometry::GeometryData::isConvex
	 */
	virtual bool isConvex() {
		return false;
	}

	virtual rw::math::Vector3D<> evaluate(double t) const = 0;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_GEOMETRICCURVE_HPP_ */
