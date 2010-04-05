/*
 * StaticListFilter.hpp
 *
 *  Created on: Apr 23, 2009
 *      Author: jimali
 */

#ifndef STATICLISTFILTER_HPP_
#define STATICLISTFILTER_HPP_

#include "BroadPhaseStrategy.hpp"
#include <rw/models/WorkCell.hpp>
#include "CollisionSetup.hpp"
#include <rw/kinematics/Frame.hpp>

namespace rw { namespace proximity {

/**
 * @brief a simple rule based broadphase strategy. A static frame pair list of
 * frame pairs that is to be checked for collision is maintained. The list is static in
 * the sense that it is not optimized to be changed, though the user can both add and remove
 * new geometries and rules.
 *
 * @note The framepair list is explicitly kept in this class which makes this broadphase strategy
 * infeasible for workcells with many objects. Consider a workcell with 100 objects, this
 * will in worst case make a list of 10000 framepairs.
 */
class StaticListFilter: public BroadPhaseStrategy {
public:

	/**
	 * @brief constructor
	 */
	StaticListFilter(rw::models::WorkCellPtr workcell);

	StaticListFilter(rw::models::WorkCellPtr workcell, const CollisionSetup& setup);

	StaticListFilter(rw::models::WorkCellPtr workcell, CollisionStrategyPtr strategy, const CollisionSetup& setup);

	/**
	 * @brief adds the \b framepair to the framelist
	 */
	void include(const kinematics::FramePair& framepair);

	/**
	 * @brief adds all possible framepairs containing the frame \b frame to the framelist
	 */
	void include(kinematics::Frame* frame);

	/**
	 * @brief removes the \b framepair from the framepair list
	 */
	void exclude(const kinematics::FramePair& framepair);

	/**
	 * @brief removes all possible framepairs containing the frame \b frame from the framelist
	 */
	void exclude(kinematics::Frame* frame);

	//////// interface inherited from BroadPhaseStrategy

	/**
	 * @copydoc BroadPhaseStrategy::reset
	 */
	virtual void reset(const rw::kinematics::State& state);

	/**
	 * @copydoc BroadPhaseStrategy::update
	 */
	virtual void update(const rw::kinematics::State& state);

	/**
	 * @copydoc BroadPhaseStrategy::next
	 */
	const rw::kinematics::FramePair& next();

	/**
	 * @copydoc BroadPhaseStrategy::hasNext
	 */
	bool hasNext();

	/**
	 *
	 */
	CollisionSetup& getCollisionSetup();

	void addModel(const rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom);

	void removeModel(const rw::kinematics::Frame* frame, const std::string& geoid);

};

typedef rw::common::Ptr<StaticListFilter> StaticListFilterPtr;
}
}

#endif /* STATICLISTFILTER_HPP_ */
