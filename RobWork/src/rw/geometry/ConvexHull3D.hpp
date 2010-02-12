/*
 * Hull3D.hpp
 *
 *  Created on: 30-03-2009
 *      Author: jimali
 */

#ifndef HULL3D_HPP_
#define HULL3D_HPP_

#include "Triangle.hpp"
#include "IndexedTriangle.hpp"
#include "IndexedTriMesh.hpp"
#include "PlainTriMesh.hpp"

#include <rw/math/Vector3D.hpp>
namespace rw {
namespace geometry {
class ConvexHull3D {
public:

	/**
	 * @brief rebuilts the hull
	 * @param vertices
	 */
	virtual void rebuild(const std::vector<rw::math::Vector3D<> >& vertices) = 0;

	/**
	 * @brief test if the given vertex is inside the convex hull
	 */
	virtual bool isInside(const rw::math::Vector3D<>& vertex) = 0;

	/**
	 * @brief if the vertex is inside the convex hull the minimum distance
	 * to any of the half-spaces of the hull is returned. If its not inside
	 * 0 is returned.
	 * @param vertex
	 * @return
	 */
	virtual double getMinDist(const rw::math::Vector3D<>& vertex) = 0;

	/**
	 * @brief create a plain trimesh from the hull facets
	 * @return the hull facets as a plain triangle mesh with normal information
	 */

	virtual rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<> >* toTriMesh() = 0;

};

}
}

#endif /* HULL3D_HPP_ */
