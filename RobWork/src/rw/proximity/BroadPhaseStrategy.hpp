/*
 * BroadPhaseDetector.hpp
 *
 *  Created on: 24-03-2009
 *      Author: jimali
 */

#ifndef BROADPHASEDETECTOR_HPP_
#define BROADPHASEDETECTOR_HPP_

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

#include "CollisionSetup.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/Geometry.hpp>

namespace rw { namespace proximity {



/**
 * @brief describe the interphase of a broad phase proximity strategy or proximity culler.
 *
 *
 */
class BroadPhaseStrategy {
public:

	/**
	 * @brief
	 */
	virtual void reset(const rw::kinematics::State& state) = 0;

	/**
	 * @brief called once before
	 */
	virtual void update(const rw::kinematics::State& state) = 0;

	virtual const rw::kinematics::FramePair& next() = 0;

	virtual bool hasNext() = 0;

	virtual CollisionSetup& getCollisionSetup() = 0;

	virtual void addModel(const rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom) = 0;

	virtual void removeModel(const rw::kinematics::Frame* frame, const std::string& geoid) = 0;

};

typedef rw::common::Ptr<BroadPhaseStrategy> BroadPhaseStrategyPtr;


}
}

#endif /* BROADPHASEDETECTOR_HPP_ */
