/*
 * GraspStrategy.hpp
 *
 *  Created on: 27/05/2010
 *      Author: jimali
 */

#ifndef GRASPSTRATEGY_HPP_
#define GRASPSTRATEGY_HPP_

#include <rw/common/PropertyMap.hpp>
#include <rw/common/Ptr.hpp>
#include "StateSampler.hpp"

class GraspStrategy {
public:

	virtual StateSamplerPtr getSampler() = 0;

	virtual std::string getIdentifier() = 0;

	virtual rw::common::PropertyMap& getSettings() = 0;

	virtual void applySettings() = 0;
};

typedef rw::common::Ptr<GraspStrategy> GraspStrategyPtr;

#endif /* GRASPSTRATEGY_HPP_ */
