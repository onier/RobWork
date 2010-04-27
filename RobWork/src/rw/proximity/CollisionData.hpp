/*
 * CollisionData.hpp
 *
 *  Created on: 23/04/2010
 *      Author: jimali
 */

#ifndef COLLISIONDATA_HPP_
#define COLLISIONDATA_HPP_

//#include "ProximityData.hpp"
#include "ProximityModel.hpp"
#include "ProximityCache.hpp"

namespace rw {
namespace proximity {

	struct CollisionPair{
		int geoIdxA, geoIdxB;
		std::vector<std::pair<int,int> > _geomPrimIds;
	};

	/***
	 * @brief A generic object for containing data that is essential in
	 * collision detection between two ProximityModels.
	 *
	 * example: collision result, cached variables for faster collision detection,
	 *
	 */
	class CollisionData {
	public:

		// transform from model a to model b
		rw::math::Transform3D<> _aTb;
		// the two models that where tested
		ProximityModel *a, *b;
		// the features that where colliding
		std::vector<CollisionPair> _collidePairs;

		ProximityCachePtr _cache;
	};

}
}

#endif /* COLLISIONDATA_HPP_ */
