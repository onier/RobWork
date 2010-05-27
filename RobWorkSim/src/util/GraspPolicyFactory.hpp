/*
 * GraspPolicyFactory.hpp
 *
 *  Created on: 27/05/2010
 *      Author: jimali
 */

#ifndef GRASPPOLICYFACTORY_HPP_
#define GRASPPOLICYFACTORY_HPP_

#include "GraspPolicy.hpp"

class GraspPolicyFactory {
public:

	std::vector<std::string> getAvailablePolicies();

	GraspPolicyPtr makePolicy(const std::string& id);

};

#endif /* GRASPPOLICYFACTORY_HPP_ */
