/*
 * GraspStrategy.hpp
 *
 *  Created on: 27/05/2010
 *      Author: jimali
 */

#ifndef GRASPSTRATEGY_HPP_
#define GRASPSTRATEGY_HPP_

class GraspStrategy {
public:

	virtual StateSamplerPtr getSampler() = 0;

	virtual std::string getIdentifier() = 0;

	virtual rw::common::PropertyMap& getSettings() = 0;

	virtual void applySettings() = 0;

};

#endif /* GRASPSTRATEGY_HPP_ */
