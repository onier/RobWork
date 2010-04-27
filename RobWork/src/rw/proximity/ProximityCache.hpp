/*
 * ProximityCache.hpp
 *
 *  Created on: 23/04/2010
 *      Author: jimali
 */

#ifndef PROXIMITYCACHE_HPP_
#define PROXIMITYCACHE_HPP_

#include "ProximityStrategy.hpp"

namespace rw {
namespace proximity {


	class ProximityCache {
	public:
		ProximityCache(ProximityStrategy *owner):
			_owner(owner)
		{
		}
		virtual ~ProximityCache(){};
		virtual size_t size() const = 0;
		virtual void clear() = 0;

		ProximityStrategy *_owner;
	};

	typedef rw::common::Ptr<ProximityCache> ProximityCachePtr;

}
}

#endif /* PROXIMITYCACHE_HPP_ */
