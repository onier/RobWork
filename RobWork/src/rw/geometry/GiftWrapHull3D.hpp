/*
 * GiftWrapHull.hpp
 *
 *  Created on: 30-03-2009
 *      Author: jimali
 */

#ifndef GIFTWRAPHULL3D_HPP_
#define GIFTWRAPHULL3D_HPP_

#include "ConvexHull3D.hpp"

#include "Triangle.hpp"
#include "PlainTriMesh.hpp"

#include <stack>
#include <set>
#include <vector>

namespace rw {
namespace geometry {


class GiftWrapHull3D : public ConvexHull3D {
public:
	GiftWrapHull3D(){};
	virtual ~GiftWrapHull3D(){};

	void rebuild(const std::vector<rw::math::Vector3D<> >& vertices);

	bool isInside(const rw::math::Vector3D<>& vertex);

	double getMinDist(const rw::math::Vector3D<>& vertex);

	rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<double> >* toTriMesh();

public:
	typedef std::pair<int,int> EdgeIdx;

	struct TriangleIdx {
		TriangleIdx(int v1, int v2, int v3, const rw::math::Vector3D<>& n):
			_n(n)
		{
			_vIdx[0]=v1;
			_vIdx[1]=v2;
			_vIdx[2]=v3;
		}
		int _vIdx[3];
		rw::math::Vector3D<> _n;
	};

private:


	int search(const EdgeIdx& edge, const rw::math::Vector3D<>& normal);

private:
	// vertices on the hull
	std::vector<rw::math::Vector3D<> > _vertices;
	// edges between vertices
	std::set<EdgeIdx> _edgeSet;
	std::stack<std::pair<EdgeIdx,int> > _edgeStack;
	// triangles composed of edges
	std::vector<TriangleIdx> _tris;
};

}
}
#endif /* GIFTWRAPHULL_HPP_ */
