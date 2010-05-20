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
	StaticListFilter();
	StaticListFilter(kinematics::FramePairSet includeset);

	/**
	 * @brief constructor
	 */
	StaticListFilter(rw::models::WorkCellPtr workcell);

	StaticListFilter(rw::models::WorkCellPtr workcell, const CollisionSetup& setup);

	StaticListFilter(rw::models::WorkCellPtr workcell, CollisionStrategyPtr strategy, const CollisionSetup& setup);

	virtual ~StaticListFilter(){};

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

	void addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom);

	void removeModel(rw::kinematics::Frame* frame, const std::string& geoid);

	//// static methods that manipulate and create frame pair sets

    /**
       @brief The full set of pairs of frames for which to perform collision
       checking when given a workcell \b workcell and a collision setup \b
       setup for the workcell. The collision strategy \b strategy is used to
       verify if a frame has a model of if it can be safely excluded (unless
       other has been specified in \b setup).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
        const rw::models::WorkCell& workcell,
        CollisionStrategy& strategy,
        const CollisionSetup& setup);

    /**
       @brief TODO:Like makeFramePairSet(\b workcell, \b setup) where
       \b setup is the default collision setup registered for the workcell
       (or \b setup is the empty collision setup if no collision setup has
       been specified).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
        const rw::models::WorkCell& workcell,
        const CollisionSetup& setup);

    /**
       @brief Like makeFramePairSet(\b workcell, \b setup, \b setup) where
       \b setup is the default collision setup registered for the workcell
       (or \b setup is the empty collision setup if no collision setup has
       been specified).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
        const rw::models::WorkCell& workcell,
        CollisionStrategy& strategy);

    /**
       @brief Like makeFramePairSet(\b workcell, \b setup) where
       \b setup is the default collision setup registered for the workcell
       (or \b setup is the empty collision setup if no collision setup has
       been specified).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
    		const rw::models::WorkCell& workcell);


    /**
       @brief Assuming that \b device is the only active device, and that
       all other frames are fixed including DAF attachments, return the
       smallest set of pairs of frames that can be deduced to be necessary
       for collision checking.

       The function assumes that DAFs have been attached according to \b
       state.

       Unlike other versions of the makeFramePairSet() function, the
       function *does not* know about the collision setup of the \b workcell
       and *does not* care about DAFs. You may therefore want to take the
       intersection between the set returned here, and the maximum set of
       frames to include in collision checking that has been returned by
       another makeFramePairSet() function.
    */
    static
    kinematics::FramePairSet
    makeFramePairSet(
        const rw::models::Device& device,
        const rw::kinematics::State& state);

    /**
       @brief Pair of (staticSet, dynamicSet) where \b staticSet are the
       pairs of frames to use for collision checking \b controlledDevices
       against the static part of the workcell and \b dynamicSet are the
       pairs of frames to use for dynamically checking \b controlledDevices
       against movement in \b obstacleDevices.

       The construction of (staticSet, dynamicSet) assumes that the state of
       the rest of the workcell is otherwise given by \b state.

       @param workcellSet [in] The standard collision setup for the workcell.
       @param obstacleDevices [in] The set of devices serving as dynamic obstacles.
       @param controlledDevices [in] The set of devices for which planning is done.

       @param state [in] The fixed state relative to which \b
       obstacleDevices and \b controlledDevices move.

       @return (staticSet, dynamicSet) where \b staticSet contain \b
       workcellSet with all pairs removed that reference frames affected by
       \b obstacleDevices, and \b dynamicSet contain all pairs of frames of
       \b workcellSet that (for the same pair) refer to both the frames
       affected by \b obstacleDevices and the frames affected by \b
       controlledDevices.
    */
    static
    std::pair<kinematics::FramePairSet, kinematics::FramePairSet>
    makeStaticDynamicFramePairSet(
        const kinematics::FramePairSet& workcellSet,
        const std::vector<rw::models::DevicePtr>& obstacleDevices,
        const std::vector<rw::models::DevicePtr>& controlledDevices,
        const rw::kinematics::State& state);

    /**
       @brief Write to \b b the intersection of \b a and \b b.

       This is equivalent to erasing from \b b all elements of \b b that are
       not elements of \b a.
    */
    static
    void intersect(const kinematics::FramePairSet& a, kinematics::FramePairSet& b);

    /**
       @brief Write to \b a all elements of \b a that are also elements of
       \b b.
    */
    static
    void subtract(kinematics::FramePairSet& a, const kinematics::FramePairSet& b);

    /**
       @brief Write to \b b the union of the sets \b a and \b b.
    */
    static
    void frameSetUnion(const kinematics::FrameSet& a, kinematics::FrameSet& b);


    static
    kinematics::FramePairList getExcludePairList(const rw::models::WorkCell& workcell,
                                     const CollisionSetup& setup);

private:

	kinematics::FramePairSet _collisionPairs;
	CollisionSetup _csetup;

	// this is the states in this class
	kinematics::FramePair _pair;
	kinematics::FramePairSet::iterator _pos;
};

typedef rw::common::Ptr<StaticListFilter> StaticListFilterPtr;
}
}

#endif /* STATICLISTFILTER_HPP_ */
