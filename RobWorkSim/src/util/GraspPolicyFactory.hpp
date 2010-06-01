/*
 * GraspPolicyFactory.hpp
 *
 *  Created on: 27/05/2010
 *      Author: jimali
 */

#ifndef GRASPPOLICYFACTORY_HPP_
#define GRASPPOLICYFACTORY_HPP_

#include <dynamics/DynamicWorkcell.hpp>
#include <rw/models/JointDevice.hpp>
#include "GraspPolicy.hpp"

class GraspPolicyFactory {
public:

	static std::vector<std::string> getAvailablePolicies();

	static GraspPolicyPtr makePolicy(
			const std::string& id,
			dynamics::DynamicWorkcell* dwc,
			rw::models::JointDevice* dev);

};

#endif /* GRASPPOLICYFACTORY_HPP_ */
