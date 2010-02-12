/*
 * GiftWrapHull.hpp
 *
 *  Created on: 30-03-2009
 *      Author: jimali
 */

#ifndef GIFTWRAPHULL2D_HPP_
#define GIFTWRAPHULL2D_HPP_

#include "ConvexHull2D.hpp"
#include <stack>
#include <set>
#include <vector>

namespace rw {
namespace geometry {

class GiftWrapHull2D : public ConvexHull2D {
public:
	GiftWrapHull2D(){};
	virtual ~GiftWrapHull2D(){};

	void rebuild(const std::vector<rw::math::Vector2D<> >& vertices);

	bool isInside(const rw::math::Vector2D<>& vertex);

	double getMinDist(const rw::math::Vector2D<>& vertex);

	std::vector<rw::math::Vector2D<> >* toContour();

public:
	typedef std::pair<int,int> EdgeIdx;

private:


	int search(const EdgeIdx& edge);

private:
	// vertices on the hull
	std::vector<rw::math::Vector3D<> > _vertices;
	// edges between vertices
	std::set<EdgeIdx> _edgeSet;
	std::stack<std::pair<EdgeIdx,int> > _edgeStack;
	// triangles composed of edges

};
}
}
#endif /* GIFTWRAPHULL_HPP_ */
