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

#ifndef RWLIBS_NURBS_GEOMETRICSURFACE_HPP_
#define RWLIBS_NURBS_GEOMETRICSURFACE_HPP_

/**
 * @file GeometricSurface.hpp
 *
 * \copydoc rwlibs::nurbs::GeometricSurface
 */

#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class GeometricSurface: public rw::geometry::GeometryData {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<GeometricSurface> Ptr;

	GeometricSurface() {}
	~GeometricSurface() {}

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
		const std::size_t SEGMENTS = 10;
		std::vector<std::vector<rw::math::Vector3D<> > > map;
		for (std::size_t i = 0; i <= SEGMENTS; i++) {
			map.push_back(std::vector<rw::math::Vector3D<> >());
			for (std::size_t j = 0; j <= SEGMENTS; j++) {
				double u = ((double)(i))/((double)SEGMENTS);
				double v = ((double)(j))/((double)SEGMENTS);
				map[i].push_back(evaluate(u,v));
			}
		}
		rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<> > mesh;
		for (std::size_t i = 0; i < SEGMENTS; i++) {
			for (std::size_t j = 0; j < SEGMENTS; j++) {
				 mesh.add( rw::geometry::TriangleN1<>(map[i][j],map[i+1][j],map[i][j+1]) );
				 mesh.add( rw::geometry::TriangleN1<>(map[i+1][j],map[i+1][j+1],map[i][j+1]) );
			}
		}
		return mesh.getTriMesh();
	}

	/**
	 * @copydoc rw::geometry::GeometryData::isConvex
	 */
	virtual bool isConvex() {
		return false;
	}

	virtual rw::math::Vector3D<> evaluate(double u, double v) const = 0;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_GEOMETRICSURFACE_HPP_ */
