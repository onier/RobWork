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

#ifndef RWLIBS_NURBS_BREP_HPP_
#define RWLIBS_NURBS_BREP_HPP_

/**
 * @file BRep.hpp
 *
 * \copydoc rwlibs::nurbs::BRep
 */

#include <list>

#include <rw/common/Ptr.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwlibs/nurbs/ParametricSurface.hpp>
#include <rwlibs/nurbs/ParametricCurve.hpp>

namespace rwlibs {
namespace nurbs {

//! @addtogroup nurbs
//! @{
class BRep: public rw::geometry::GeometryData {
public:
    //! @brief smart pointer type to this class
    typedef rw::common::Ptr<BRep> Ptr;

	struct Vertex {
		rw::math::Vector3D<> point;
	};

	struct Edge {
		std::pair<Vertex, Vertex> ends;
		rwlibs::nurbs::ParametricCurve::Ptr curve;
	};

	struct Face {
		std::vector<Edge> loop;
		rwlibs::nurbs::ParametricSurface::Ptr surface;
	};

	BRep();
	BRep(rw::geometry::Box::Ptr box, double roundingRadius = 0);
	~BRep();

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
	virtual bool isConvex();

	void addFace(rwlibs::nurbs::ParametricSurface::Ptr surface);

	std::list<Face> getFaces();
	std::list<Edge> getEdges();
	std::list<Vertex> getVertices();
	std::list<rwlibs::nurbs::ParametricSurface::Ptr> getSurfaces();
	std::list<rwlibs::nurbs::ParametricCurve::Ptr> getCurves();
	std::list<rw::math::Vector3D<> > getPoints();

private:
	std::list<Face> _faces;
	std::list<Edge> _edges;
	std::list<Vertex> _vertices;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_BREP_HPP_ */
