/*
 * GraspStrategyFactory.hpp
 *
 *  Created on: 27/05/2010
 *      Author: jimali
 */

#ifndef GRASPSTRATEGYFACTORY_HPP_
#define GRASPSTRATEGYFACTORY_HPP_

#include "GraspStrategy.hpp"

class GraspStrategyFactory {
public:

	/**
	 * @brief a list of available strategies.
	 */
	static std::vector<std::string> getAvailableStrategies();

	/**
	 * @brief instantiate a strategy with ID \b id.
	 * @param id [in] id of strategy
	 */
	static GraspStrategyPtr makeStrategy(const std::string& id);

};

#endif /* GRASPSTRATEGYFACTORY_HPP_ */
