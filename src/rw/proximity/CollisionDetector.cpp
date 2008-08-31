/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "CollisionDetector.hpp"
#include "CollisionStrategy.hpp"
#include "CollisionSetup.hpp"
#include "Proximity.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;

CollisionDetector::CollisionDetector(
    CollisionStrategyPtr strategy,
    const FramePairSet& pairs)
    :
    _strategy(strategy),
    _collisionPairs(pairs)
{
    RW_ASSERT(strategy);
}

CollisionDetector::CollisionDetector(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy,
    const CollisionSetup& setup)
    :
    _strategy(strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    _collisionPairs = Proximity::makeFramePairSet(*workcell, *strategy, setup);
}

CollisionDetector::CollisionDetector(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy)
    :
    _strategy(strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    _collisionPairs = Proximity::makeFramePairSet(*workcell, *strategy);
}

namespace
{
    bool pairCollides(
        CollisionStrategy& strategy,
        const FramePair& pair,
        const FKTable& fk)
    {
        const Frame* a = pair.first;
        const Frame* b = pair.second;
        return strategy.inCollision(a, fk.get(*a), b, fk.get(*b));
    }
}

bool CollisionDetector::inCollision(
    const State& state,
    FramePairSet* result,
    bool stopAtFirstContact) const
{
    FKTable fk(state);

    bool found = false;
    BOOST_FOREACH(const FramePair& pair, _collisionPairs) {
        if (pairCollides(*_strategy, pair, fk)) {
            found = true;
            if (result) {
                result->insert(pair);
                if (stopAtFirstContact) break;
            } else
                break;
        }
    }

    return found;
}

void CollisionDetector::setCollisionStrategy(CollisionStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    _strategy = strategy;
}

//----------------------------------------------------------------------
// Constructor functions

std::auto_ptr<CollisionDetector> CollisionDetector::make(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    return make(
        strategy,
        Proximity::makeFramePairSet(*workcell, *strategy));
}

std::auto_ptr<CollisionDetector> CollisionDetector::make(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy,
    const CollisionSetup& setup)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    return make(
        strategy,
        Proximity::makeFramePairSet(*workcell, *strategy, setup));
}

std::auto_ptr<CollisionDetector> CollisionDetector::make(
    CollisionStrategyPtr strategy,
    const FramePairSet& pairs)
{
    RW_ASSERT(strategy);

    typedef std::auto_ptr<CollisionDetector> T;
    return T(new CollisionDetector(strategy, pairs));
}
