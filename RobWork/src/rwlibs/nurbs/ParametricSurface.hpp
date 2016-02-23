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

#ifndef RWLIBS_NURBS_PARAMETRICSURFACE_HPP_
#define RWLIBS_NURBS_PARAMETRICSURFACE_HPP_

/**
 * @file ParametricSurface.hpp
 *
 * \copydoc rwlibs::nurbs::ParametricSurface
 */

#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class ParametricSurface: public rw::geometry::GeometryData {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<ParametricSurface> Ptr;

	/**
	 * @copydoc rw::geometry::GeometryData::getType
	 */
	virtual GeometryType getType() const;

	/**
	 * @copydoc rw::geometry::GeometryData::getTriMesh
	 */
	virtual rw::common::Ptr<rw::geometry::TriMesh> getTriMesh(bool forceCopy=true);

	/**
	 * @copydoc rw::geometry::GeometryData::isConvex
	 */
	virtual bool isConvex() = 0;

	virtual rw::math::Vector3D<> evaluate(double u, double v) = 0;
	virtual rw::math::Vector3D<> normal(double u, double v) = 0;

protected:
	virtual rw::common::Ptr<rw::geometry::TriMesh> createTriMesh(unsigned int resU, unsigned int resV);
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_PARAMETRICSURFACE_HPP_ */
