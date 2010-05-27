/*
 * GraspPolicy.hpp
 *
 *  Created on: 27/05/2010
 *      Author: jimali
 */

#ifndef GRASPPOLICY_HPP_
#define GRASPPOLICY_HPP_

/**
 * @brief a grasp policy defines how a grasp is executed from
 * some initial configuration.
 *
 * Typically this is some control
 * scheme that will close the fingers of a device into a grasp
 * of an object.
 *
 */
class GraspPolicy {
public:

	virtual rwlibs::simulation::SimulatedController* getController() = 0;

	virtual std::string getIdentifier() = 0;

	virtual rw::common::PropertyMap getSettings() = 0;

	virtual void applySettings() = 0;

};

typedef rw::common::Ptr<GraspPolicy> GraspPolicyPtr;


#endif /* GRASPPOLICY_HPP_ */
